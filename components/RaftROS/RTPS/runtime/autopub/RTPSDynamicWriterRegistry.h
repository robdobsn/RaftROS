/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSDynamicWriterRegistry — fixed-capacity table of per-device ROS 2 DataWriter endpoints
//
// Phase 4, Slice 4.1 (auto-publishing).  This is the shared-runtime data structure that
// tracks every currently-active "dynamic" DataWriter backing an I²C device (one row per
// publisher that was created in response to a Raft device coming online, per the
// RaftROS auto-publishing design doc §7).
//
// Shared-runtime layer: this header is pure data + pure decision logic (no sockets, no
// dynamic allocation, no Raft / ESP-IDF APIs, no std::string ownership).  Wrappers are
// expected to:
//   * call `allocate()` on a device-online event,
//   * call `release()` on a device-offline / detach event,
//   * iterate entries to emit SEDP publication DATA(w) announces / dispose DATAs,
//   * look up by writer entity ID when routing incoming ACKNACK submessages,
//   * maintain the per-slot user-data sequence number (`seqNum` / `sedpSeqNum`).
//
// Entity-ID allocation policy (`RTPSDynamicWriterRegistry_allocateEntityId`):
//   slot N -> { 0x00, 0x01, (uint8_t)(DYNAMIC_WRITER_ENTITY_KEY_BASE + N), 0x03 }
// where 0x03 is the DDS "user-defined writer w/ key" entity kind (matches the existing
// ENTITYID_CHATTER_WRITER = {0x00, 0x01, 0x01, 0x03}).  Entity key byte is 0x10..0x1F
// for the 16 default dynamic slots, leaving 0x00..0x0F reserved for static/builtin
// writers (chatter is 0x01).  Capacity is capped at 16 so no dynamic allocation is
// ever required on ESP.  This can be raised by changing the constant; the entityKey
// byte must not exceed 0xFF.
//
// Slot re-use: when a device goes offline, its slot is released and its entityId
// returns to the pool.  A newly-arriving device may be allocated the same slot
// (and therefore the same writer entityId).  The caller is expected to emit a
// dispose-DATA for the old publication *before* reusing the slot (per design doc §3.2);
// subscribers match by topic+type, not by writer GUID, so reuse is safe once the
// dispose has been observed.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

namespace RaftRuntime::RTPS::Runtime::AutoPub
{

// Maximum concurrent dynamic DataWriter endpoints.  Hard cap to bound heap usage on
// ESP32-S3 (design doc §8: ~1 kB per writer * 16 = ~24 kB total).  Raise with care.
static constexpr uint8_t DYNAMIC_WRITER_REGISTRY_CAPACITY = 16;

// Base entity-key byte (byte index 2 of the 4-byte entityId).  Dynamic slots 0..15
// map to entity-key bytes 0x10..0x1F, leaving 0x00..0x0F for static/builtin writers.
static constexpr uint8_t DYNAMIC_WRITER_ENTITY_KEY_BASE = 0x10;

// SEDP publications writer sequence-number allocation for dynamic slots.
//
// The SEDP publications writer is a single RTPS endpoint; every publication
// DATA(w) sample it emits must have a unique monotonically-increasing seq.
// Seq 1 is reserved for ros_discovery_info, seq 2 for /chatter.  Dynamic
// writers get one seq for the announce and one for the dispose — reusing
// the same seq for re-announces (FastDDS deduplicates by (writerGUID, seq)
// so a re-announce with the same seq is treated as a retransmit of the
// already-received sample rather than a fresh one, which is the behaviour
// we want).  Layout:
//   slot N announce  -> AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + N
//   slot N dispose   -> AUTOPUB_SEDP_DISPOSE_BASE_SEQ  + N
static constexpr uint64_t AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ = 3;
static constexpr uint64_t AUTOPUB_SEDP_DISPOSE_BASE_SEQ  =
    AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + DYNAMIC_WRITER_REGISTRY_CAPACITY;

// Opaque per-device key identifying which physical device owns a registry slot.
// Carries (busNum, address) rather than RaftDeviceID directly so this header has
// zero dependency on RaftCore — the wrapper is responsible for the translation.
// `subIndex` disambiguates multi-writer composite devices (Slice 4.10): a device
// that publishes both sensor_msgs/Temperature and sensor_msgs/RelativeHumidity
// occupies two slots with subIndex=0 and subIndex=1.
struct RTPSDynamicWriterKey
{
    uint8_t  busNum   = 0;
    uint32_t address  = 0;   // BusElemAddrType equivalent (wide enough for any bus)
    uint8_t  subIndex = 0;   // 0 = primary writer, 1+ = composite secondaries

    bool equals(const RTPSDynamicWriterKey& other) const
    {
        return busNum == other.busNum
            && address == other.address
            && subIndex == other.subIndex;
    }

    // A zero (busNum=0, address=0) key is reserved as "invalid / unset".
    bool isValid() const { return !(busNum == 0 && address == 0); }
};

// One row in the registry.  POD-like; can be zero-initialised and trivially copied.
struct RTPSDynamicWriterEntry
{
    bool     inUse = false;
    uint8_t  entityId[4] = {0, 0, 0, 0};    // writer entity ID (DDS user writer w/ key)
    RTPSDynamicWriterKey key = {};

    // Topic + type — non-owning pointers.  The caller is responsible for keeping
    // the strings alive for the lifetime of the slot (typically owned by the
    // per-device context struct in the RaftROS wrapper).
    const char* topic = nullptr;
    const char* type  = nullptr;

    // Sequence counters advanced by the publishing hot path (user-data) and the
    // SEDP publication announcer.  Owned by the caller; registry only stores.
    uint64_t seqNum = 0;
    uint64_t sedpSeqNum = 0;

    // Slice 4.11 — QoS profile id (mirrors `RTPSAutoPubQoSProfileId`).
    // Stored as a raw byte so this header stays free of the profile enum.
    // Default 3 == FallbackString (RELIABLE / VOLATILE / depth=10).
    uint8_t qosProfileId = 3;
};

// Deterministic writer-entity allocator.  Returns true if `slot` is within capacity
// and writes 4 bytes to `outEntityId`.  See entity-ID allocation policy comment above.
inline bool RTPSDynamicWriterRegistry_allocateEntityId(uint8_t slot, uint8_t outEntityId[4])
{
    if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY)
        return false;
    outEntityId[0] = 0x00;
    outEntityId[1] = 0x01;
    outEntityId[2] = static_cast<uint8_t>(DYNAMIC_WRITER_ENTITY_KEY_BASE + slot);
    outEntityId[3] = 0x03;
    return true;
}

class RTPSDynamicWriterRegistry
{
public:
    // Allocate the lowest-indexed free slot for `key`.  Returns the slot index (>= 0)
    // on success, or -1 if the registry is full, the key is invalid, a slot already
    // exists for this key, or `topic`/`type` are null/empty.  On success writes the
    // 4-byte writer entityId to `outEntityId` (if non-null).
    int allocate(RTPSDynamicWriterKey key, const char* topic, const char* type,
                 uint8_t outEntityId[4] = nullptr,
                 uint8_t qosProfileId = 3 /*FallbackString*/)
    {
        if (!key.isValid())
            return -1;
        if (!topic || !type || !*topic || !*type)
            return -1;
        if (find(key) >= 0)
            return -1;

        for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
        {
            if (_entries[slot].inUse)
                continue;
            uint8_t eid[4] = {0, 0, 0, 0};
            if (!RTPSDynamicWriterRegistry_allocateEntityId(slot, eid))
                return -1;
            _entries[slot].inUse      = true;
            _entries[slot].entityId[0] = eid[0];
            _entries[slot].entityId[1] = eid[1];
            _entries[slot].entityId[2] = eid[2];
            _entries[slot].entityId[3] = eid[3];
            _entries[slot].key        = key;
            _entries[slot].topic      = topic;
            _entries[slot].type       = type;
            _entries[slot].seqNum     = 0;
            // Fixed unique SEDP seq per slot — see comment on
            // AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ above.  Stable across the
            // slot's lifetime; re-announces reuse the same seq.
            _entries[slot].sedpSeqNum = AUTOPUB_SEDP_ANNOUNCE_BASE_SEQ + slot;
            _entries[slot].qosProfileId = qosProfileId;
            if (outEntityId)
                std::memcpy(outEntityId, eid, 4);
            return slot;
        }
        return -1;
    }

    // Release the slot whose key matches.  Returns true if a slot was freed.
    // Clears the slot's fields so stale pointers cannot be observed post-release.
    bool release(RTPSDynamicWriterKey key)
    {
        const int slot = find(key);
        if (slot < 0)
            return false;
        releaseSlot(static_cast<uint8_t>(slot));
        return true;
    }

    // Release by slot index.  Idempotent — returns false for unknown / already-free slots.
    bool releaseSlot(uint8_t slot)
    {
        if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY)
            return false;
        if (!_entries[slot].inUse)
            return false;
        _entries[slot] = RTPSDynamicWriterEntry{};
        return true;
    }

    // Linear search by key.  Returns slot index (>= 0) or -1 if not present.
    int find(RTPSDynamicWriterKey key) const
    {
        if (!key.isValid())
            return -1;
        for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
        {
            if (!_entries[slot].inUse)
                continue;
            if (_entries[slot].key.equals(key))
                return slot;
        }
        return -1;
    }

    // Linear search by writer entity ID (4-byte compare).  Returns slot index or -1.
    // Used by incoming ACKNACK / SEDP dispose ACK paths to route back to the writer state.
    int findByEntityId(const uint8_t entityId[4]) const
    {
        if (!entityId)
            return -1;
        for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
        {
            if (!_entries[slot].inUse)
                continue;
            if (std::memcmp(_entries[slot].entityId, entityId, 4) == 0)
                return slot;
        }
        return -1;
    }

    // Read-only accessor.  Returns nullptr for out-of-range / free slots.
    const RTPSDynamicWriterEntry* get(uint8_t slot) const
    {
        if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY)
            return nullptr;
        if (!_entries[slot].inUse)
            return nullptr;
        return &_entries[slot];
    }

    // Mutable accessor (used by the publishing hot path to bump `seqNum`).  Returns
    // nullptr for out-of-range / free slots.
    RTPSDynamicWriterEntry* getMutable(uint8_t slot)
    {
        if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY)
            return nullptr;
        if (!_entries[slot].inUse)
            return nullptr;
        return &_entries[slot];
    }

    // Capacity constants (copied to instance accessors for code that wants a value
    // through a virtual/generic handle).
    uint8_t capacity() const { return DYNAMIC_WRITER_REGISTRY_CAPACITY; }

    // Number of slots currently in use.  O(capacity).
    uint8_t inUseCount() const
    {
        uint8_t n = 0;
        for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
            if (_entries[slot].inUse)
                n++;
        return n;
    }

    // Drop every entry.  Primarily for teardown / unit tests.
    void clear()
    {
        for (uint8_t slot = 0; slot < DYNAMIC_WRITER_REGISTRY_CAPACITY; slot++)
            _entries[slot] = RTPSDynamicWriterEntry{};
    }

private:
    RTPSDynamicWriterEntry _entries[DYNAMIC_WRITER_REGISTRY_CAPACITY] = {};
};

} // namespace RaftRuntime::RTPS::Runtime::AutoPub
