#include "RTPSReliabilityPolicy.h"

#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

bool RTPSReliability_shouldRespondToHeartbeat(uint8_t heartbeatFlags)
{
    return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::shouldRespondToHeartbeat(
        heartbeatFlags);
}

bool RTPSReliability_acknackRequestsSeq(uint32_t bitmapBaseLow, uint64_t seqNum)
{
    return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::acknackRequestsSeq(
        bitmapBaseLow, seqNum);
}
