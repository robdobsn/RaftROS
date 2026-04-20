#include "RTPSAckNack.h"
#include "RTPSTypes.h"
#include "RTPSMessage.h"

#include <string.h>

bool RTPSAckNack_parse(const uint8_t* pContent, uint32_t contentLen, RTPSAckNackFields& outFields)
{
    if (!pContent || contentLen < 24)
        return false;

    // ACKNACK layout:
    // readerEID(4) + writerEID(4) + bitmapBase(8) + numBits(4) + [bitmap] + count(4)
    outFields.readerEID = pContent;
    outFields.writerEID = pContent + 4;
    outFields.bitmapBaseLow = RTPSMessage::readLE32(pContent + 12);
    outFields.numBits = RTPSMessage::readLE32(pContent + 16);
    return true;
}

RTPSAckNackWriterKind RTPSAckNack_classifyWriter(const uint8_t* writerEID)
{
    if (!writerEID)
        return RTPSAckNackWriterKind::Unknown;

    if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::SedpPublications;
    if (memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::SedpSubscriptions;
    if (memcmp(writerEID, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::RosDiscoveryInfo;
    if (memcmp(writerEID, ENTITYID_CHATTER_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::Chatter;
    return RTPSAckNackWriterKind::Unknown;
}

const char* RTPSAckNack_writerKindToStr(RTPSAckNackWriterKind writerKind)
{
    switch (writerKind)
    {
    case RTPSAckNackWriterKind::SedpPublications:
        return "sedp_publications";
    case RTPSAckNackWriterKind::SedpSubscriptions:
        return "sedp_subscriptions";
    case RTPSAckNackWriterKind::RosDiscoveryInfo:
        return "ros_discovery_info";
    case RTPSAckNackWriterKind::Chatter:
        return "chatter";
    default:
        return "unknown";
    }
}
