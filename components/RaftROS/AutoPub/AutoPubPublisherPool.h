/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubPublisherPool — generation-safe publisher handles + latest-only mailboxes
//
// Bus data callbacks run on bus worker tasks while attach/detach, discovery
// and network I/O run on the SysMod loop task. This pool is the bounded
// handoff between them:
//
//   loop task   acquire()  -> handle (slot + generation), passed to the bus as
//                             the callback-info value instead of a heap pointer
//   bus task    produce()  -> validates the handle under the lock, lets the
//                             caller decode into scratch, and stores only the
//                             latest record in the slot's fixed mailbox
//   loop task   drain()    -> copies each pending record out under the lock and
//                             hands it to the backend outside the lock
//   loop task   release()  -> bumps the generation under the lock; once it
//                             returns no producer can reach the user pointer
//
// A stale handle (slot released or reused) is dropped, never dereferenced, so
// callbacks that race with detach or outlive their registration are harmless.
// acquire(), release() and drain() must all run on the same (loop) task:
// drain() passes the user pointer to the consumer outside the lock, which is
// only safe because release() cannot run concurrently with it.
//
// No allocation. `Lock` must provide `bool lock(uint32_t timeoutMs)` (with
// UINT32_MAX meaning wait forever) and `void unlock()`.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>

namespace RaftRuntime::AutoPub
{

/// @brief Slot + generation, packable into the pointer-sized callback-info
/// value that bus callbacks carry. Generation 0 is never issued, so an encoded
/// valid handle is never null.
struct AutoPubPublisherHandle
{
    static constexpr uint8_t INVALID_SLOT = 0xFF;
    static constexpr uint32_t GENERATION_MASK = 0x00FFFFFF;

    uint8_t slot = INVALID_SLOT;
    uint32_t generation = 0;

    bool isValid() const { return slot != INVALID_SLOT && generation != 0; }

    const void* toCallbackInfo() const
    {
        if (!isValid())
            return nullptr;
        return reinterpret_cast<const void*>(
            (static_cast<uintptr_t>(generation & GENERATION_MASK) << 8) | slot);
    }

    static AutoPubPublisherHandle fromCallbackInfo(const void* pCallbackInfo)
    {
        const uintptr_t value = reinterpret_cast<uintptr_t>(pCallbackInfo);
        AutoPubPublisherHandle handle;
        handle.slot = static_cast<uint8_t>(value & 0xFF);
        handle.generation = static_cast<uint32_t>(value >> 8) & GENERATION_MASK;
        return handle;
    }
};

/// @brief Outcome of one produce() call.
enum class AutoPubProduceResult : uint8_t
{
    Stored,       ///< Record stored as the slot's pending sample
    Overwritten,  ///< Record stored, replacing an undrained sample
    Empty,        ///< Fill produced nothing (no decode / no records)
    Stale,        ///< Handle invalid, released or reused
    Busy,         ///< Lock not obtained within the timeout
};

/// @brief One record handed to the drain consumer. `record` is pool scratch,
/// valid only for the duration of the consumer call.
struct AutoPubDrainedSample
{
    AutoPubPublisherHandle handle;
    void* pUser = nullptr;
    const uint8_t* record = nullptr;
    uint32_t length = 0;
    uint32_t produced = 0;     ///< Records stored in this slot since acquire
    uint32_t overwritten = 0;  ///< Records replaced before being drained
};

/// @brief Pool-wide diagnostic counters (monotonic, relaxed).
struct AutoPubPoolCounters
{
    uint32_t staleDrops = 0;
    uint32_t busyDrops = 0;
    uint32_t emptyFills = 0;
    uint32_t drainBusySkips = 0;  ///< Drain passes deferred because a producer held the lock
};

template<typename Lock, uint8_t Capacity, uint32_t MaxRecordSize>
class AutoPubPublisherPool
{
public:
    static_assert(Capacity > 0 && Capacity < AutoPubPublisherHandle::INVALID_SLOT,
                  "slot index must fit the handle encoding");
    static_assert(MaxRecordSize > 0, "mailbox record size must be non-zero");
    static constexpr uint32_t WAIT_FOREVER = UINT32_MAX;

    AutoPubPublisherPool() = default;
    AutoPubPublisherPool(const AutoPubPublisherPool&) = delete;
    AutoPubPublisherPool& operator=(const AutoPubPublisherPool&) = delete;

    /// @brief Claim a slot for `pUser` (loop task). Returns an invalid handle
    /// if the pool is full or `recordSize` is zero or exceeds MaxRecordSize.
    AutoPubPublisherHandle acquire(void* pUser, uint32_t recordSize)
    {
        AutoPubPublisherHandle handle;
        if (!pUser || recordSize == 0 || recordSize > MaxRecordSize)
            return handle;
        if (!_lock.lock(WAIT_FOREVER))
            return handle;
        for (uint8_t slot = 0; slot < Capacity; ++slot)
        {
            Entry& entry = _entries[slot];
            if (entry.inUse)
                continue;
            entry.generation = nextGeneration(entry.generation);
            entry.inUse = true;
            entry.pending = false;
            entry.pUser = pUser;
            entry.recordSize = recordSize;
            entry.length = 0;
            entry.produced = 0;
            entry.overwritten = 0;
            handle.slot = slot;
            handle.generation = entry.generation;
            _inUseCount.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        _lock.unlock();
        return handle;
    }

    /// @brief Invalidate a handle and discard any pending sample (loop task).
    /// Blocks until no producer holds the lock; afterwards the user pointer is
    /// unreachable from produce(), so the caller may free it.
    bool release(AutoPubPublisherHandle handle)
    {
        if (!handle.isValid() || handle.slot >= Capacity)
            return false;
        if (!_lock.lock(WAIT_FOREVER))
            return false;
        Entry& entry = _entries[handle.slot];
        const bool matched = entry.inUse && entry.generation == handle.generation;
        if (matched)
        {
            entry.generation = nextGeneration(entry.generation);
            entry.inUse = false;
            entry.pending = false;
            entry.pUser = nullptr;
            _inUseCount.fetch_sub(1, std::memory_order_relaxed);
        }
        _lock.unlock();
        return matched;
    }

    /// @brief Store the latest record for `handle` (any task). `fill` is
    /// called as `uint32_t fill(void* pUser, uint8_t* scratch, uint32_t recordSize)`
    /// with the lock held and must write at most `recordSize` bytes, returning
    /// the length written (0 = nothing to store). Keep it short and non-blocking.
    template<typename Fill>
    AutoPubProduceResult produce(AutoPubPublisherHandle handle, Fill&& fill,
                                 uint32_t lockTimeoutMs)
    {
        if (!handle.isValid() || handle.slot >= Capacity)
            return countStale();
        if (!_lock.lock(lockTimeoutMs))
        {
            _busyDrops.fetch_add(1, std::memory_order_relaxed);
            return AutoPubProduceResult::Busy;
        }
        Entry& entry = _entries[handle.slot];
        if (!entry.inUse || entry.generation != handle.generation)
        {
            _lock.unlock();
            return countStale();
        }
        // Fill into scratch so a failed or partial fill never corrupts a
        // pending sample that has not been drained yet.
        const uint32_t length = fill(entry.pUser, _fillScratch, entry.recordSize);
        if (length == 0 || length > entry.recordSize)
        {
            _lock.unlock();
            _emptyFills.fetch_add(1, std::memory_order_relaxed);
            return AutoPubProduceResult::Empty;
        }
        std::memcpy(entry.record, _fillScratch, length);
        entry.length = length;
        const bool overwrote = entry.pending;
        entry.pending = true;
        ++entry.produced;
        if (overwrote)
            ++entry.overwritten;
        _lock.unlock();
        return overwrote ? AutoPubProduceResult::Overwritten : AutoPubProduceResult::Stored;
    }

    /// @brief Hand every pending record to `consume(const AutoPubDrainedSample&)`
    /// (loop task). The lock is held only while copying each record out, and
    /// is never waited for: if a producer holds it (possibly preempted on
    /// another core), the rest of this pass is deferred — remaining samples
    /// stay pending for the next pass — so the loop task cannot inherit a
    /// bus-task stall. Each deferred pass is counted once.
    /// @return number of records consumed
    template<typename Consume>
    uint32_t drain(Consume&& consume)
    {
        uint32_t drained = 0;
        for (uint8_t slot = 0; slot < Capacity; ++slot)
        {
            AutoPubDrainedSample sample;
            if (!_lock.lock(0))
            {
                _drainBusySkips.fetch_add(1, std::memory_order_relaxed);
                break;
            }
            Entry& entry = _entries[slot];
            const bool ready = entry.inUse && entry.pending;
            if (ready)
            {
                std::memcpy(_drainScratch, entry.record, entry.length);
                sample.handle.slot = slot;
                sample.handle.generation = entry.generation;
                sample.pUser = entry.pUser;
                sample.record = _drainScratch;
                sample.length = entry.length;
                sample.produced = entry.produced;
                sample.overwritten = entry.overwritten;
                entry.pending = false;
            }
            _lock.unlock();
            if (ready)
            {
                consume(static_cast<const AutoPubDrainedSample&>(sample));
                ++drained;
            }
        }
        return drained;
    }

    uint8_t inUseCount() const { return _inUseCount.load(std::memory_order_relaxed); }

    AutoPubPoolCounters counters() const
    {
        AutoPubPoolCounters snapshot;
        snapshot.staleDrops = _staleDrops.load(std::memory_order_relaxed);
        snapshot.busyDrops = _busyDrops.load(std::memory_order_relaxed);
        snapshot.emptyFills = _emptyFills.load(std::memory_order_relaxed);
        snapshot.drainBusySkips = _drainBusySkips.load(std::memory_order_relaxed);
        return snapshot;
    }

    static constexpr uint8_t capacity() { return Capacity; }
    static constexpr uint32_t maxRecordSize() { return MaxRecordSize; }

private:
    struct Entry
    {
        void* pUser = nullptr;
        uint32_t generation = 0;
        uint32_t recordSize = 0;
        uint32_t length = 0;
        uint32_t produced = 0;
        uint32_t overwritten = 0;
        bool inUse = false;
        bool pending = false;
        uint8_t record[MaxRecordSize] = {};
    };

    static uint32_t nextGeneration(uint32_t generation)
    {
        generation = (generation + 1) & AutoPubPublisherHandle::GENERATION_MASK;
        return generation == 0 ? 1 : generation;
    }

    AutoPubProduceResult countStale()
    {
        _staleDrops.fetch_add(1, std::memory_order_relaxed);
        return AutoPubProduceResult::Stale;
    }

    Lock _lock;
    Entry _entries[Capacity];
    uint8_t _fillScratch[MaxRecordSize] = {};
    uint8_t _drainScratch[MaxRecordSize] = {};
    std::atomic<uint32_t> _staleDrops{0};
    std::atomic<uint32_t> _busyDrops{0};
    std::atomic<uint32_t> _emptyFills{0};
    std::atomic<uint32_t> _drainBusySkips{0};
    std::atomic<uint8_t> _inUseCount{0};
};

}
