/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSSubscriptionRegistry — fixed-capacity table of user-topic String subscriptions
//
// A tiny POD registry that records, for each registered std_msgs/String subscription,
// the 4-byte RTPS entity ID our reader exposes, plus the ROS 2 DDS topic + type names
// we advertise over SEDP.
//
// Shared-runtime layer: this header is pure data + pure decision logic (no sockets, no
// dynamic allocation, no Raft/ESP-IDF APIs).  Wrappers are expected to:
//   * populate the registry via `add(topic, type)`,
//   * iterate entries to emit SEDP subscription announces / heartbeats,
//   * iterate `readerEntityIds()` to fill the `ros_discovery_info` reader_gid_seq,
//   * look up by writer entity ID when dispatching DATA.
//
// Dispatch note: because DATA submessages carry only the remote writer's entity ID
// (not the topic name), per-topic handler routing requires correlating remote
// SEDP publications (remote writerGUID -> topicName) with our registry entries.
// That correlation is intentionally OUT OF SCOPE for this header; a single shared
// handler in the wrapper covers the current use-cases.  See follow-up plan.
//
// Entity ID allocation policy (`allocateNextEntityId`):
//   slot 0 -> ENTITYID_CHATTER_READER = {0x00, 0x01, 0x02, 0x04}
//   slot N -> {0x00, 0x01, (uint8_t)(0x02 + N), 0x04}
// The low nibble `0x04` is the DDS "user-defined reader w/ key" entity kind.
// Capacity is capped at 8 slots so no dynamic allocation is ever required on ESP.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

namespace RaftRuntime::RTPS::Runtime::Dispatch
{

static constexpr uint8_t RTPS_SUBSCRIPTION_REGISTRY_CAPACITY = 8;

struct RTPSSubscriptionEntry
{
    uint8_t entityId[4] = {0, 0, 0, 0};
    const char* topic = nullptr;      // non-owning; caller retains storage
    const char* type  = nullptr;      // non-owning; caller retains storage
};

// Deterministic reader-entity allocator.
// Returns true if `slot` is within capacity and writes 4 bytes to `outEntityId`.
inline bool RTPSSubscriptionRegistry_allocateNextEntityId(uint8_t slot, uint8_t outEntityId[4])
{
    if (slot >= RTPS_SUBSCRIPTION_REGISTRY_CAPACITY)
        return false;
    outEntityId[0] = 0x00;
    outEntityId[1] = 0x01;
    outEntityId[2] = static_cast<uint8_t>(0x02 + slot);
    outEntityId[3] = 0x04;
    return true;
}

struct RTPSSubscriptionRegistry
{
    RTPSSubscriptionEntry entries[RTPS_SUBSCRIPTION_REGISTRY_CAPACITY];
    uint8_t count = 0;

    // Append a new subscription entry.  Returns the index assigned (== old count) or -1 if full.
    // Overwrites `outEntityId` (if non-null) with the allocated 4-byte entity ID.
    int add(const char* topic, const char* type, uint8_t outEntityId[4] = nullptr)
    {
        if (count >= RTPS_SUBSCRIPTION_REGISTRY_CAPACITY)
            return -1;
        if (!topic || !type || !*topic || !*type)
            return -1;
        const uint8_t slot = count;
        uint8_t eid[4] = {0, 0, 0, 0};
        if (!RTPSSubscriptionRegistry_allocateNextEntityId(slot, eid))
            return -1;
        entries[slot].entityId[0] = eid[0];
        entries[slot].entityId[1] = eid[1];
        entries[slot].entityId[2] = eid[2];
        entries[slot].entityId[3] = eid[3];
        entries[slot].topic = topic;
        entries[slot].type = type;
        if (outEntityId)
            memcpy(outEntityId, eid, 4);
        count = static_cast<uint8_t>(slot + 1);
        return slot;
    }

    // Update the topic/type for an existing slot.  Returns true on success.
    bool setTopic(uint8_t slot, const char* topic, const char* type)
    {
        if (slot >= count)
            return false;
        if (!topic || !type || !*topic || !*type)
            return false;
        entries[slot].topic = topic;
        entries[slot].type = type;
        return true;
    }

    // Lookup by writer entity ID (== our reader entity ID we advertised).
    // Returns pointer into `entries` or nullptr.
    const RTPSSubscriptionEntry* findByEntityId(const uint8_t entityId[4]) const
    {
        if (!entityId)
            return nullptr;
        for (uint8_t i = 0; i < count; i++)
        {
            if (memcmp(entries[i].entityId, entityId, 4) == 0)
                return &entries[i];
        }
        return nullptr;
    }

    // Fill `outReaderIds` with pointers to each entry's entity ID bytes.
    // Returns the number written (<= count).  Pass `maxOut` to cap safely.
    uint32_t readerEntityIds(const uint8_t** outReaderIds, uint32_t maxOut) const
    {
        uint32_t n = 0;
        for (uint8_t i = 0; i < count && n < maxOut; i++)
            outReaderIds[n++] = entries[i].entityId;
        return n;
    }
};

} // namespace RaftRuntime::RTPS::Runtime::Dispatch
