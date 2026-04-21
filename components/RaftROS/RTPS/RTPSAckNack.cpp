#include "RTPSAckNack.h"

#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

bool RTPSAckNack_parse(const uint8_t* pContent, uint32_t contentLen, RTPSAckNackFields& outFields)
{
    return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::parseAckNack(
        pContent, contentLen, outFields);
}

RTPSAckNackWriterKind RTPSAckNack_classifyWriter(const uint8_t* writerEID)
{
    return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::classifyWriter(writerEID);
}

const char* RTPSAckNack_writerKindToStr(RTPSAckNackWriterKind writerKind)
{
    return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::writerKindToStr(writerKind);
}
