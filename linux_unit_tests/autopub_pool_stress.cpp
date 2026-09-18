/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubPublisherPool concurrency stress test
//
// Models the firmware threading: two "bus" threads call produce() with
// handles that may be stale, while the "loop" thread drains and repeatedly
// detaches/re-attaches devices, deleting each device's state right after
// release() exactly as RaftROS::autoPubDetachDevice does. Build under
// ThreadSanitizer (data races) and AddressSanitizer (use-after-free):
//
//   make autopub-pool-stress SAN=thread
//   make autopub-pool-stress SAN=address
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>
#include "AutoPub/AutoPubPublisherPool.h"
#include "autopub_pool_test_lock.h"

using namespace RaftRuntime::AutoPub;

namespace
{

constexpr uint8_t CAPACITY = 4;
constexpr uint32_t RECORD_SIZE = 16;
constexpr uint32_t DEVICE_MAGIC = 0x5AFEC0DE;
using Pool = AutoPubPublisherPool<AutoPubTestLock, CAPACITY, RECORD_SIZE>;

// Stand-in for DynamicWriterCtx: heap state the producer mutates under the lock.
struct Device
{
    uint32_t magic = DEVICE_MAGIC;
    uint8_t id = 0;
    uint32_t fills = 0;
    uint8_t decodeBuf[RECORD_SIZE] = {};
};

}

int main()
{
    Pool pool;
    // Bus-side registrations (like the bus's copy of pCallbackInfo): the
    // encoded handle plus the id of the device whose raw data the bus will
    // deliver with it. Producers may read an old registration after a detach;
    // the pool must then never let that data reach the slot's new owner.
    std::atomic<uint64_t> busRegistrations[CAPACITY];
    Device* devices[CAPACITY] = {};
    AutoPubPublisherHandle handles[CAPACITY];
    uint8_t nextId = 1;

    auto attach = [&](uint8_t index) {
        Device* device = new Device();
        device->id = nextId++;
        handles[index] = pool.acquire(device, RECORD_SIZE);
        devices[index] = device;
        const uint64_t encoded = reinterpret_cast<uintptr_t>(handles[index].toCallbackInfo());
        busRegistrations[index].store((encoded << 8) | device->id);
    };
    auto detach = [&](uint8_t index) {
        pool.release(handles[index]);
        delete devices[index];  // safe only because release() fenced producers
        devices[index] = nullptr;
    };
    for (uint8_t i = 0; i < CAPACITY; ++i)
        attach(i);

    std::atomic<bool> stop{false};
    std::atomic<uint32_t> producerErrors{0};
    std::atomic<uint64_t> stored{0};
    auto producer = [&](unsigned seed) {
        uint32_t x = seed;
        while (!stop.load(std::memory_order_relaxed))
        {
            x = x * 1664525u + 1013904223u;
            const uint64_t registration = busRegistrations[x % CAPACITY].load();
            const uint8_t rawDataOwner = static_cast<uint8_t>(registration & 0xFF);
            const auto handle = AutoPubPublisherHandle::fromCallbackInfo(
                reinterpret_cast<const void*>(static_cast<uintptr_t>(registration >> 8)));
            const auto result = pool.produce(handle,
                [&](void* pUser, uint8_t* scratch, uint32_t recordSize) -> uint32_t {
                    Device* device = static_cast<Device*>(pUser);
                    // Reaching any device other than the raw data's owner means a
                    // stale registration fed another device's mailbox.
                    if (device->magic != DEVICE_MAGIC || recordSize != RECORD_SIZE ||
                        device->id != rawDataOwner)
                        producerErrors.fetch_add(1);
                    ++device->fills;
                    // "Decode" the raw data into the device buffer, then copy the latest record
                    std::memset(device->decodeBuf, rawDataOwner, sizeof(device->decodeBuf));
                    std::memcpy(scratch, device->decodeBuf, recordSize);
                    return recordSize;
                },
                1);
            if (result == AutoPubProduceResult::Stored || result == AutoPubProduceResult::Overwritten)
                stored.fetch_add(1, std::memory_order_relaxed);
            // Bus tasks poll periodically rather than spinning on the lock
            std::this_thread::yield();
        }
    };
    std::thread busA(producer, 1u);
    std::thread busB(producer, 2u);

    uint32_t consumerErrors = 0;
    uint64_t drained = 0;
    uint32_t cycles = 0;
    // Run for at least MIN_CYCLES detach/attach cycles and 1 s, capped at 30 s
    // (sanitizers slow the loop by very different factors).
    constexpr uint32_t MIN_CYCLES = 2000;
    const auto start = std::chrono::steady_clock::now();
    auto elapsed = [&] { return std::chrono::steady_clock::now() - start; };
    while ((cycles < MIN_CYCLES || elapsed() < std::chrono::seconds(1)) &&
           elapsed() < std::chrono::seconds(30))
    {
        drained += pool.drain([&](const AutoPubDrainedSample& sample) {
            const Device* device = static_cast<const Device*>(sample.pUser);
            // The drained user must be the live owner of that slot, and the
            // record must be one untorn fill by that same owner.
            if (sample.handle.slot >= CAPACITY || device != devices[sample.handle.slot] ||
                device->magic != DEVICE_MAGIC || sample.length != RECORD_SIZE)
            {
                ++consumerErrors;
                return;
            }
            for (uint32_t i = 0; i < sample.length; ++i)
                if (sample.record[i] != device->id)
                    ++consumerErrors;
        });
        const uint8_t index = static_cast<uint8_t>(cycles % CAPACITY);
        detach(index);
        attach(index);
        ++cycles;
    }
    stop.store(true);
    busA.join();
    busB.join();
    for (uint8_t i = 0; i < CAPACITY; ++i)
        detach(i);

    const auto counters = pool.counters();
    std::printf("autopub pool stress: cycles=%u stored=%llu drained=%llu stale=%u busy=%u "
                "producerErrors=%u consumerErrors=%u\n",
                cycles, (unsigned long long)stored.load(), (unsigned long long)drained,
                counters.staleDrops, counters.busyDrops, producerErrors.load(), consumerErrors);
    const bool exercised = cycles >= MIN_CYCLES && drained > 100 && counters.staleDrops > 0;
    if (producerErrors.load() || consumerErrors || !exercised || pool.inUseCount() != 0)
    {
        std::printf("--- autopub pool stress FAILED%s ---\n", exercised ? "" : " (insufficient interleaving)");
        return 1;
    }
    std::printf("--- autopub pool stress passed ---\n");
    return 0;
}
