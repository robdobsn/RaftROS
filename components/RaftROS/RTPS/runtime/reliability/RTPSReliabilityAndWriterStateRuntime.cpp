#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

#include <cstring>

#include "RTPSTypes.h"
#include "runtime/wire/RTPSMessage.h"

namespace RaftROS::RTPS::Runtime::ReliabilityAndWriterState
{

RTPSAckActionSedpSequenceContext makeAckSedpSequenceContext(
    uint64_t rosDiscoveryPublicationSeqNum,
    uint64_t rosDiscoverySubscriptionSeqNum,
    uint64_t chatterPublicationSeqNum,
    bool deriveChatterPublicationFromRosDiscoveryPublication)
{
    RTPSAckActionSedpSequenceContext context;
    context.rosDiscoveryPublicationSeqNum = rosDiscoveryPublicationSeqNum;
    context.rosDiscoverySubscriptionSeqNum = rosDiscoverySubscriptionSeqNum;
    context.chatterPublicationSeqNum = chatterPublicationSeqNum;
    context.deriveChatterPublicationFromRosDiscoveryPublication =
        deriveChatterPublicationFromRosDiscoveryPublication;
    return context;
}

RTPSAckActionUserDataSequenceContext makeAckUserDataSequenceContext(
    uint64_t rosDiscoveryInfoSeqNum,
    uint64_t chatterDataSeqNum,
    bool chatterFirstSNMatchesSequence)
{
    RTPSAckActionUserDataSequenceContext context;
    context.rosDiscoveryInfoSeqNum = rosDiscoveryInfoSeqNum;
    context.chatterDataSeqNum = chatterDataSeqNum;
    context.chatterFirstSNMatchesSequence = chatterFirstSNMatchesSequence;
    return context;
}

RTPSAckNackDecisionOptions makeAckNackDecisionOptionsForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor)
{
    RTPSAckNackDecisionOptions options;
    switch (runtimeFlavor)
    {
        case RTPSAckNackRuntimeFlavor::LinuxStandalone:
            options.publicationsIncludesChatterAnnouncement = true;
            options.requirePublicationSeq2GateForRetransmit = true;
            break;

        case RTPSAckNackRuntimeFlavor::EspStyle:
        default:
            options.publicationsIncludesChatterAnnouncement = false;
            options.requirePublicationSeq2GateForRetransmit = false;
            break;
    }
    return options;
}

RTPSAckNackMutationPolicy makeAckNackMutationPolicyForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor)
{
    RTPSAckNackMutationPolicy mutationPolicy;
    switch (runtimeFlavor)
    {
        case RTPSAckNackRuntimeFlavor::LinuxStandalone:
            mutationPolicy.incrementHeartbeatOnSedpRetransmit = true;
            mutationPolicy.incrementHeartbeatOnUserDataRetransmit = true;
            break;

        case RTPSAckNackRuntimeFlavor::EspStyle:
        default:
            mutationPolicy.incrementHeartbeatOnSedpRetransmit = false;
            mutationPolicy.incrementHeartbeatOnUserDataRetransmit = true;
            break;
    }
    return mutationPolicy;
}

RTPSAckActionSedpSequenceContext makeAckSedpSequenceContextForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor,
    uint64_t rosDiscoveryPublicationSeqNum,
    uint64_t rosDiscoverySubscriptionSeqNum,
    uint64_t chatterPublicationSeqNum)
{
    const bool deriveChatterPublicationFromRosDiscoveryPublication =
        runtimeFlavor == RTPSAckNackRuntimeFlavor::LinuxStandalone;
    return makeAckSedpSequenceContext(
        rosDiscoveryPublicationSeqNum,
        rosDiscoverySubscriptionSeqNum,
        chatterPublicationSeqNum,
        deriveChatterPublicationFromRosDiscoveryPublication);
}

RTPSAckActionUserDataSequenceContext makeAckUserDataSequenceContextForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor,
    uint64_t rosDiscoveryInfoSeqNum,
    uint64_t chatterDataSeqNum)
{
    const bool chatterFirstSNMatchesSequence =
        runtimeFlavor == RTPSAckNackRuntimeFlavor::LinuxStandalone;
    return makeAckUserDataSequenceContext(
        rosDiscoveryInfoSeqNum,
        chatterDataSeqNum,
        chatterFirstSNMatchesSequence);
}

bool shouldRespondToHeartbeat(uint8_t heartbeatFlags)
{
    return (heartbeatFlags & 0x02) == 0;
}

bool acknackRequestsSeq(uint32_t bitmapBaseLow, uint64_t seqNum)
{
    return bitmapBaseLow <= seqNum;
}

bool parseAckNack(const uint8_t* pContent,
                  uint32_t contentLen,
                  RTPSAckNackFields& outFields)
{
    if (!pContent || contentLen < 24)
        return false;

    outFields.readerEID = pContent;
    outFields.writerEID = pContent + 4;
    outFields.bitmapBaseLow = RTPSMessage::readLE32(pContent + 12);
    outFields.numBits = RTPSMessage::readLE32(pContent + 16);
    return true;
}

RTPSAckNackWriterKind classifyWriter(const uint8_t* writerEID)
{
    if (!writerEID)
        return RTPSAckNackWriterKind::Unknown;

    if (std::memcmp(writerEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::SedpPublications;
    if (std::memcmp(writerEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::SedpSubscriptions;
    if (std::memcmp(writerEID, ENTITYID_ROS_DISC_INFO_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::RosDiscoveryInfo;
    if (std::memcmp(writerEID, ENTITYID_CHATTER_WRITER, 4) == 0)
        return RTPSAckNackWriterKind::Chatter;
    return RTPSAckNackWriterKind::Unknown;
}

const char* writerKindToStr(RTPSAckNackWriterKind writerKind)
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

const uint8_t* localReaderForRemoteWriter(const uint8_t* remoteWriterEID,
                                          const uint8_t* fallbackReaderEID)
{
    if (!remoteWriterEID)
        return fallbackReaderEID ? fallbackReaderEID : ENTITYID_UNKNOWN;

    if (std::memcmp(remoteWriterEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
        return ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER;
    if (std::memcmp(remoteWriterEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
        return ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER;
    if (std::memcmp(remoteWriterEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
        return ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER;
    if (std::memcmp(remoteWriterEID, ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER, 4) == 0)
        return ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER;

    return fallbackReaderEID ? fallbackReaderEID : ENTITYID_UNKNOWN;
}

void evaluateAckNackActions(const RTPSAckNackFields& fields,
                            RTPSAckNackWriterKind writerKind,
                            const RTPSAckNackDecisionOptions& options,
                            uint64_t chatterSeq,
                            RTPSAckNackDecisionEmitActionFn emitAction,
                            void* emitCtx)
{
    if (!emitAction)
        return;

    switch (writerKind)
    {
        case RTPSAckNackWriterKind::SedpPublications:
        {
            const bool gatePass = !options.requirePublicationSeq2GateForRetransmit ||
                                  acknackRequestsSeq(fields.bitmapBaseLow, 2);
            if (!gatePass)
                return;

            if (acknackRequestsSeq(fields.bitmapBaseLow, 1))
            {
                emitAction(emitCtx,
                           RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication);
            }

            if (options.publicationsIncludesChatterAnnouncement &&
                acknackRequestsSeq(fields.bitmapBaseLow, 2))
            {
                emitAction(emitCtx,
                           RTPSAckNackDecisionAction::RetransmitSedpChatterPublication);
            }
            return;
        }

        case RTPSAckNackWriterKind::SedpSubscriptions:
            if (acknackRequestsSeq(fields.bitmapBaseLow, 1))
            {
                emitAction(emitCtx,
                           RTPSAckNackDecisionAction::RetransmitSedpRosDiscoverySubscription);
            }
            return;

        case RTPSAckNackWriterKind::RosDiscoveryInfo:
            if (acknackRequestsSeq(fields.bitmapBaseLow, 1))
                emitAction(emitCtx, RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo);
            return;

        case RTPSAckNackWriterKind::Chatter:
            if ((chatterSeq > 0) && acknackRequestsSeq(fields.bitmapBaseLow, chatterSeq))
                emitAction(emitCtx, RTPSAckNackDecisionAction::RetransmitChatterData);
            return;

        default:
            return;
    }
}

void applyAckActionHeartbeatMutation(RTPSAckNackDecisionAction action,
                                     const RTPSAckNackMutationPolicy& mutationPolicy,
                                     uint32_t& heartbeatCount)
{
    switch (action)
    {
        case RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication:
        case RTPSAckNackDecisionAction::RetransmitSedpChatterPublication:
        case RTPSAckNackDecisionAction::RetransmitSedpRosDiscoverySubscription:
            if (mutationPolicy.incrementHeartbeatOnSedpRetransmit)
                heartbeatCount++;
            return;

        case RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo:
        case RTPSAckNackDecisionAction::RetransmitChatterData:
            if (mutationPolicy.incrementHeartbeatOnUserDataRetransmit)
                heartbeatCount++;
            return;

        default:
            return;
    }
}

bool getAckActionSedpPublicationProfile(RTPSAckNackDecisionAction action,
                                        RTPSAckActionSedpEndpointProfile& outProfile)
{
    switch (action)
    {
        case RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication:
            outProfile.entityId = ENTITYID_ROS_DISC_INFO_WRITER;
            outProfile.topicName = ROS_DISCOVERY_INFO_TOPIC;
            outProfile.typeName = ROS_DISCOVERY_INFO_TYPE;
            outProfile.reliabilityKind = RELIABILITY_RELIABLE;
            outProfile.durabilityKind = DURABILITY_TRANSIENT_LOCAL;
            return true;

        case RTPSAckNackDecisionAction::RetransmitSedpChatterPublication:
            outProfile.entityId = ENTITYID_CHATTER_WRITER;
            outProfile.topicName = CHATTER_DDS_TOPIC;
            outProfile.typeName = CHATTER_DDS_TYPE;
            outProfile.reliabilityKind = RELIABILITY_RELIABLE;
            outProfile.durabilityKind = DURABILITY_VOLATILE;
            return true;

        default:
            return false;
    }
}

bool getAckActionSedpSubscriptionProfile(RTPSAckNackDecisionAction action,
                                         RTPSAckActionSedpEndpointProfile& outProfile)
{
    switch (action)
    {
        case RTPSAckNackDecisionAction::RetransmitSedpRosDiscoverySubscription:
            outProfile.entityId = ENTITYID_ROS_DISC_INFO_READER;
            outProfile.topicName = ROS_DISCOVERY_INFO_TOPIC;
            outProfile.typeName = ROS_DISCOVERY_INFO_TYPE;
            outProfile.reliabilityKind = RELIABILITY_RELIABLE;
            outProfile.durabilityKind = DURABILITY_TRANSIENT_LOCAL;
            return true;

        default:
            return false;
    }
}

uint64_t getAckActionSedpSequenceNumber(
    RTPSAckNackDecisionAction action,
    const RTPSAckActionSedpSequenceContext& sequenceContext,
    bool& outHasSequence)
{
    outHasSequence = true;
    switch (action)
    {
        case RTPSAckNackDecisionAction::RetransmitSedpRosDiscoveryPublication:
            return sequenceContext.rosDiscoveryPublicationSeqNum;

        case RTPSAckNackDecisionAction::RetransmitSedpRosDiscoverySubscription:
            return sequenceContext.rosDiscoverySubscriptionSeqNum;

        case RTPSAckNackDecisionAction::RetransmitSedpChatterPublication:
            if (sequenceContext.deriveChatterPublicationFromRosDiscoveryPublication)
                return sequenceContext.rosDiscoveryPublicationSeqNum + 1;
            return sequenceContext.chatterPublicationSeqNum;

        default:
            outHasSequence = false;
            return 0;
    }
}

bool getAckActionSedpPlan(
    RTPSAckNackDecisionAction action,
    const RTPSAckActionSedpSequenceContext& sequenceContext,
    RTPSAckActionSedpPlan& outPlan)
{
    bool hasSequence = false;
    const uint64_t sequenceNumber = getAckActionSedpSequenceNumber(
        action,
        sequenceContext,
        hasSequence);
    if (!hasSequence)
        return false;

    RTPSAckActionSedpEndpointProfile profile;
    if (getAckActionSedpPublicationProfile(action, profile) ||
        getAckActionSedpSubscriptionProfile(action, profile))
    {
        outPlan.endpoint = profile;
        outPlan.sequenceNumber = sequenceNumber;
        return true;
    }

    return false;
}

bool getAckActionUserDataPlan(
    RTPSAckNackDecisionAction action,
    const RTPSAckActionUserDataSequenceContext& sequenceContext,
    RTPSAckActionUserDataPlan& outPlan)
{
    switch (action)
    {
        case RTPSAckNackDecisionAction::RetransmitRosDiscoveryInfo:
            outPlan.writerEntityId = ENTITYID_ROS_DISC_INFO_WRITER;
            outPlan.sequenceNumber = sequenceContext.rosDiscoveryInfoSeqNum;
            outPlan.hasFirstSNOverride = false;
            outPlan.firstSN = 1;
            return true;

        case RTPSAckNackDecisionAction::RetransmitChatterData:
            outPlan.writerEntityId = ENTITYID_CHATTER_WRITER;
            outPlan.sequenceNumber = sequenceContext.chatterDataSeqNum;
            outPlan.hasFirstSNOverride = sequenceContext.chatterFirstSNMatchesSequence;
            outPlan.firstSN = sequenceContext.chatterDataSeqNum;
            return true;

        default:
            return false;
    }
}

}
