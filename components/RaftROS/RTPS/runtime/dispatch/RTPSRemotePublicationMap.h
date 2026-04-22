/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSRemotePublicationMap — per-(remote writer GUID) -> subscription slot mapping
//
// Populated when an SEDP BuiltinPublications DATA submessage is parsed and the
// advertised topic name matches one of our subscription registry entries.
// Looked up on every user-data DATA submessage to route the payload to the correct
// per-topic handler (instead of the single legacy `_stringMessageHandler`).
//
// Fixed capacity (no heap).  Linear scan (N<=16 in practice) is faster than a hash map
// on ESP32-S3 for these sizes.  On overflow, the oldest entry is overwritten.
//
// Shared-runtime layer: pure POD + pure decision logic.  No sockets, no Raft APIs.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

namespace RaftRuntime::RTPS::Runtime::Dispatch
{

static constexpr uint8_t RTPS_REMOTE_PUBLICATION_MAP_CAPACITY = 16;

struct RTPSRemotePublicationEntry
{
    uint8_t writerGuid[16] = {0};   // guidPrefix(12) + writerEntityId(4)
    int8_t  subscriptionSlot = -1;  // index into RTPSSubscriptionRegistry (-1 = unused)
};

struct RTPSRemotePublicationMap
{
    RTPSRemotePublicationEntry entries[RTPS_REMOTE_PUBLICATION_MAP_CAPACITY];
    uint8_t count = 0;        // number of entries currently in use
    uint8_t writeCursor = 0;  // round-robin overwrite position once full

    // Returns the subscription slot for `writerGuid` or -1 if not mapped.
    int8_t findSlot(const uint8_t writerGuid[16]) const
    {
        if (!writerGuid)
            return -1;
        for (uint8_t i = 0; i < count; i++)
        {
            if (memcmp(entries[i].writerGuid, writerGuid, 16) == 0)
                return entries[i].subscriptionSlot;
        }
        return -1;
    }

    // Returns the subscription slot for the (guidPrefix, writerEntityId) pair.
    int8_t findSlot(const uint8_t guidPrefix[12], const uint8_t writerEntityId[4]) const
    {
        uint8_t key[16];
        if (!guidPrefix || !writerEntityId)
            return -1;
        memcpy(key, guidPrefix, 12);
        memcpy(key + 12, writerEntityId, 4);
        return findSlot(key);
    }

    // Upsert a (writerGuid -> slot) mapping.  If the writerGuid is already present,
    // updates its slot.  Otherwise appends a new entry; once the table is full,
    // overwrites the oldest via `writeCursor` (round-robin).  Returns true if a new
    // entry was added (as opposed to updating an existing row).
    bool upsert(const uint8_t writerGuid[16], int8_t slot)
    {
        if (!writerGuid || slot < 0)
            return false;
        for (uint8_t i = 0; i < count; i++)
        {
            if (memcmp(entries[i].writerGuid, writerGuid, 16) == 0)
            {
                entries[i].subscriptionSlot = slot;
                return false;
            }
        }
        if (count < RTPS_REMOTE_PUBLICATION_MAP_CAPACITY)
        {
            memcpy(entries[count].writerGuid, writerGuid, 16);
            entries[count].subscriptionSlot = slot;
            count++;
        }
        else
        {
            if (writeCursor >= RTPS_REMOTE_PUBLICATION_MAP_CAPACITY)
                writeCursor = 0;
            memcpy(entries[writeCursor].writerGuid, writerGuid, 16);
            entries[writeCursor].subscriptionSlot = slot;
            writeCursor++;
        }
        return true;
    }
};

} // namespace RaftRuntime::RTPS::Runtime::Dispatch
