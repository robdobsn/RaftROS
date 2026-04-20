#pragma once

#include <stdint.h>

// RTPS HEARTBEAT flags: respond with ACKNACK only when Final flag is not set.
bool RTPSReliability_shouldRespondToHeartbeat(uint8_t heartbeatFlags);

// ACKNACK bitmapBaseLow semantics used by this codebase:
// if base <= seqNum, that sequence may need retransmission.
bool RTPSReliability_acknackRequestsSeq(uint32_t bitmapBaseLow, uint64_t seqNum);
