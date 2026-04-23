/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubLifecycle — attach/detach coordinator for Phase 4 dynamic DataWriters
//
// Phase 4, Slice 4.2.  Composes an `RTPSDynamicWriterRegistry` with parallel **owned**
// topic / type string storage.  The registry alone stores `const char*` non-owning
// pointers (§5.1 of the design doc); but the wrapper builds topic/type strings
// dynamically per device (e.g. `rt/raft_esp32/imu_1_6a`) and needs somewhere to
// keep them alive for the slot's lifetime.  This helper provides that storage and
// presents a minimal attach / detach API mirroring the `DeviceManager`
// `registerForDeviceStatusChange` event flow.
//
// Shared-runtime layer: header-only, zero heap, no sockets, no Raft / ESP-IDF deps.
// Callable from any wrapper.
//
// Usage pattern (wrapper):
//   on device-online(busNum, addr, topic, type):
//       int slot = lifecycle.attach({busNum, addr}, topic, type, entityIdOut);
//       if (slot < 0) { /* registry full or dup */ return; }
//       // ...caller also builds its own RaftCore-flavoured device context
//       //    (field descs, decode state, poll-data buffer) keyed by slot
//   on device-offline(busNum, addr):
//       lifecycle.detach({busNum, addr});
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include "RTPSDynamicWriterRegistry.h"

namespace RaftRuntime::RTPS::Runtime::AutoPub
{

// Per-slot owned string buffers.  Sized to comfortably fit a ROS 2 topic like
//   `rt/raft_<hostname_up_to_32>/<slug_up_to_24>_<bus>_<addr>`
// and a type name like `sensor_msgs::msg::dds_::RelativeHumidity_`.
static constexpr uint8_t AUTOPUB_TOPIC_BUF_LEN = 96;
static constexpr uint8_t AUTOPUB_TYPE_BUF_LEN  = 96;

class RTPSAutoPubLifecycle
{
public:
    // Attach a new device slot.  Copies `topic` and `type` into owned storage and
    // registers the slot in the underlying `RTPSDynamicWriterRegistry`, linking the
    // registry's non-owning char pointers to this helper's storage.
    //
    // Returns the slot index (>= 0) on success, or -1 if:
    //   * the registry is full
    //   * the key is invalid (busNum == 0 && address == 0)
    //   * `topic` or `type` is null / empty
    //   * `topic` or `type` would not fit in the per-slot buffers
    //   * a slot for this key already exists (duplicate attach)
    //
    // On success writes the 4-byte writer entityId to `outEntityId` (if non-null).
    int attach(RTPSDynamicWriterKey key, const char* topic, const char* type,
               uint8_t outEntityId[4] = nullptr,
               uint8_t qosProfileId = 3 /*FallbackString*/)
    {
        if (!key.isValid())
            return -1;
        if (!topic || !type || !*topic || !*type)
            return -1;
        const size_t topicLen = std::strlen(topic);
        const size_t typeLen  = std::strlen(type);
        if (topicLen + 1 > AUTOPUB_TOPIC_BUF_LEN)
            return -1;
        if (typeLen + 1 > AUTOPUB_TYPE_BUF_LEN)
            return -1;

        // Allocate a registry slot **without** a topic/type yet so we know which
        // storage slot to populate; then link the registry entry to our owned
        // storage.  Because `allocate()` rejects null/empty names, we first copy
        // into our buffers, then hand those stable pointers to the registry.
        //
        // Pre-reserve: find lowest free slot ourselves so we can copy first.
        int targetSlot = -1;
        for (uint8_t s = 0; s < DYNAMIC_WRITER_REGISTRY_CAPACITY; s++)
        {
            const auto* e = _registry.get(s);
            if (e == nullptr)
            {
                targetSlot = s;
                break;
            }
        }
        if (targetSlot < 0)
            return -1;
        if (_registry.find(key) >= 0)
            return -1;  // duplicate

        // Copy strings into owned storage.
        std::memcpy(_topicBufs[targetSlot], topic, topicLen);
        _topicBufs[targetSlot][topicLen] = '\0';
        std::memcpy(_typeBufs[targetSlot], type, typeLen);
        _typeBufs[targetSlot][typeLen] = '\0';

        // Register.  Because the registry allocates in lowest-free-first order
        // (same scan we used above), it will pick the same `targetSlot`.
        const int slot = _registry.allocate(key, _topicBufs[targetSlot],
                                            _typeBufs[targetSlot], outEntityId,
                                            qosProfileId);
        if (slot < 0 || slot != targetSlot)
        {
            // Extremely defensive — keeps storage clean if the registry picks
            // a different slot (should be impossible given the pre-scan).
            if (slot >= 0)
                _registry.releaseSlot(static_cast<uint8_t>(slot));
            _topicBufs[targetSlot][0] = '\0';
            _typeBufs[targetSlot][0]  = '\0';
            return -1;
        }
        return slot;
    }

    // Detach the slot for `key`.  Zeroes the owned string storage and releases
    // the registry slot.  Returns true if a slot was freed.
    bool detach(RTPSDynamicWriterKey key)
    {
        const int slot = _registry.find(key);
        if (slot < 0)
            return false;
        return detachSlot(static_cast<uint8_t>(slot));
    }

    // Detach by slot index.  Idempotent — returns false for out-of-range or
    // already-free slots.
    bool detachSlot(uint8_t slot)
    {
        if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY)
            return false;
        if (_registry.get(slot) == nullptr)
            return false;
        _topicBufs[slot][0] = '\0';
        _typeBufs[slot][0]  = '\0';
        return _registry.releaseSlot(slot);
    }

    // Look up the slot index for a given device key; -1 if not attached.
    int find(RTPSDynamicWriterKey key) const { return _registry.find(key); }

    // Look up by writer entity ID (for ACKNACK / SEDP dispose ACK routing).
    int findByEntityId(const uint8_t entityId[4]) const
    {
        return _registry.findByEntityId(entityId);
    }

    // Accessors through to the registry.
    const RTPSDynamicWriterEntry* get(uint8_t slot) const { return _registry.get(slot); }
    RTPSDynamicWriterEntry*       getMutable(uint8_t slot) { return _registry.getMutable(slot); }
    uint8_t capacity() const   { return _registry.capacity(); }
    uint8_t inUseCount() const { return _registry.inUseCount(); }

    // Direct access to the owned storage (read-only).  Useful for logging /
    // SEDP-announce building paths that already have a slot index.
    const char* topicForSlot(uint8_t slot) const
    {
        if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY) return nullptr;
        return _topicBufs[slot];
    }
    const char* typeForSlot(uint8_t slot) const
    {
        if (slot >= DYNAMIC_WRITER_REGISTRY_CAPACITY) return nullptr;
        return _typeBufs[slot];
    }

    // Accessor for tests and composition — the underlying registry is exposed
    // so wrappers can plug it into existing SEDP / reliability runtime helpers
    // without re-implementing find/findByEntityId.
    const RTPSDynamicWriterRegistry& registry() const { return _registry; }
    RTPSDynamicWriterRegistry&       registryMutable() { return _registry; }

    // Tear everything down — primarily for unit tests.
    void clear()
    {
        _registry.clear();
        for (uint8_t s = 0; s < DYNAMIC_WRITER_REGISTRY_CAPACITY; s++)
        {
            _topicBufs[s][0] = '\0';
            _typeBufs[s][0]  = '\0';
        }
    }

private:
    RTPSDynamicWriterRegistry _registry;
    char _topicBufs[DYNAMIC_WRITER_REGISTRY_CAPACITY][AUTOPUB_TOPIC_BUF_LEN] = {};
    char _typeBufs [DYNAMIC_WRITER_REGISTRY_CAPACITY][AUTOPUB_TYPE_BUF_LEN]  = {};
};

} // namespace RaftRuntime::RTPS::Runtime::AutoPub
