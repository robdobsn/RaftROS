#pragma once

#include <stdint.h>

enum class RTPSAckNackWriterKind
{
    Unknown = 0,
    SedpPublications,
    SedpSubscriptions,
    RosDiscoveryInfo,
    Chatter,
};

struct RTPSAckNackFields
{
    const uint8_t* readerEID = nullptr;
    const uint8_t* writerEID = nullptr;
    uint32_t bitmapBaseLow = 0;
    uint32_t numBits = 0;
};

// Parse the ACKNACK payload fields used by current RaftROS handlers.
bool RTPSAckNack_parse(const uint8_t* pContent, uint32_t contentLen, RTPSAckNackFields& outFields);

// Classify writerEID into a known RaftROS writer type.
RTPSAckNackWriterKind RTPSAckNack_classifyWriter(const uint8_t* writerEID);

// Human-readable name for logs.
const char* RTPSAckNack_writerKindToStr(RTPSAckNackWriterKind writerKind);
