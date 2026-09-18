/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubSampleEmitter — fan one serialised sample out to RTPS peers
//
// Adapts an AutoPubSampleRunner payload to a best-effort (VOLATILE) RTPS
// writer: one sequence number per non-empty sample, sent synchronously to
// every discovered peer. No history cache or retransmission.
//
// `entry` is a raw registry pointer, NOT a generational handle: the caller
// must ensure the slot was not released and reused since it was looked up.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RTPSDynamicWriterRegistry.h"
#include "AutoPub/AutoPubSampleRunner.h"

namespace RaftRuntime::RTPS::Runtime::AutoPub
{

/// @brief Aggregate outcome of one sample across all peers.
struct RTPSAutoPubSampleEmission
{
    uint64_t sequence = 0;
    uint32_t peersSent = 0;
    RaftRuntime::AutoPub::AutoPubPublishResult result =
        RaftRuntime::AutoPub::AutoPubPublishResult::NotAttempted;
};

/// @brief Emit one sample on `entry`, calling
/// `sendToPeer(peer, entry, payload, length, sequence)` for each peer.
/// Null/released entry: InvalidHandle, nothing sent. Empty payload:
/// NotAttempted, sequence unchanged. Otherwise the sequence increments even
/// with no peers or failed sends; the result is Accepted if any peer accepted,
/// else the last peer failure, or Disconnected when there are no peers.
template<typename Peers, typename SendToPeer>
RTPSAutoPubSampleEmission RTPSAutoPubSampleEmitter_emit(
    RTPSDynamicWriterEntry* entry, const uint8_t* payload, uint32_t length,
    const Peers& peers, SendToPeer&& sendToPeer)
{
    using RaftRuntime::AutoPub::AutoPubPublishResult;
    RTPSAutoPubSampleEmission emission;
    if (!entry || !entry->inUse)
    {
        emission.result = AutoPubPublishResult::InvalidHandle;
        return emission;
    }
    if (!payload || length == 0)
        return emission;

    emission.sequence = ++entry->seqNum;
    emission.result = AutoPubPublishResult::Disconnected;
    for (const auto& peer : peers)
    {
        const auto result = sendToPeer(peer, *entry, payload, length, emission.sequence);
        if (result == AutoPubPublishResult::Accepted)
        {
            ++emission.peersSent;
            emission.result = AutoPubPublishResult::Accepted;
        }
        else if (emission.peersSent == 0)
        {
            emission.result = result;
        }
    }
    return emission;
}

}