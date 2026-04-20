#include "RTPSReliabilityPolicy.h"

bool RTPSReliability_shouldRespondToHeartbeat(uint8_t heartbeatFlags)
{
    return (heartbeatFlags & 0x02) == 0;
}

bool RTPSReliability_acknackRequestsSeq(uint32_t bitmapBaseLow, uint64_t seqNum)
{
    return bitmapBaseLow <= seqNum;
}
