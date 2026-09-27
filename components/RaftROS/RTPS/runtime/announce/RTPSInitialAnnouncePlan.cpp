#include "runtime/announce/RTPSInitialAnnouncePlan.h"
#include "RTPSTypes.h"

static void appendStep(RTPSInitialAnnounceSequence& seq,
                       RTPSInitialAnnounceAction action,
                       bool incrementHeartbeatBeforeSend,
                       RTPSSeqCounterHint seqCounterHint)
{
    if (seq.numSteps >= (sizeof(seq.steps) / sizeof(seq.steps[0])))
        return;
    seq.steps[seq.numSteps].action = action;
    seq.steps[seq.numSteps].incrementHeartbeatBeforeSend = incrementHeartbeatBeforeSend;
    seq.steps[seq.numSteps].seqCounterHint = seqCounterHint;
    seq.numSteps++;
}

RTPSInitialAnnouncePlan RTPSInitialAnnouncePlan_default()
{
    RTPSInitialAnnouncePlan plan;
    plan.sendInitialRosDiscoveryUserData = false;
    return plan;
}

RTPSInitialAnnounceSequence RTPSInitialAnnouncePlan_buildSequence(
    const RTPSInitialAnnouncePlan& plan,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor)
{
    RTPSInitialAnnounceSequence seq;

    if (plan.sendUnicastSpdpReply)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::SpdpUnicastReply,
                   false,
                   RTPSSeqCounterHint::Spdp);
        if (plan.sendSpdpDiscoveryPortCopy)
        {
            appendStep(seq,
                       RTPSInitialAnnounceAction::SpdpDiscoveryPortCopy,
                       false,
                       RTPSSeqCounterHint::Spdp);
        }
    }

    const bool heartbeatForSedp = runtimeFlavor == RTPSInitialAnnounceRuntimeFlavor::LinuxStyle;
    if (plan.sendSedpRosDiscoveryWriter)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::SedpRosDiscoveryWriter,
                   heartbeatForSedp,
                   RTPSSeqCounterHint::SedpRosWriter);
    }
    if (plan.sendSedpRosDiscoveryReader)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::SedpRosDiscoveryReader,
                   heartbeatForSedp,
                   RTPSSeqCounterHint::SedpRosReader);
    }
    if (plan.sendSedpChatterWriter)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::SedpChatterWriter,
                   heartbeatForSedp,
                   RTPSSeqCounterHint::SedpChatterWriter);
    }
    if (plan.sendSedpChatterReader)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::SedpChatterReader,
                   heartbeatForSedp,
                   RTPSSeqCounterHint::SedpChatterReader);
    }
    if (plan.sendParticipantMessageData)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::ParticipantMessageData,
                   true,
                   RTPSSeqCounterHint::Liveliness);
    }
    if (plan.sendInitialRosDiscoveryUserData)
    {
        appendStep(seq,
                   RTPSInitialAnnounceAction::RosDiscoveryUserData,
                   true,
                   RTPSSeqCounterHint::RosDiscoveryUserData);
    }

    return seq;
}

RTPSInitialAnnounceSendTarget RTPSInitialAnnouncePlan_getSendTarget(
    RTPSInitialAnnounceAction action)
{
    switch (action)
    {
        case RTPSInitialAnnounceAction::SpdpUnicastReply:
            return { RTPSInitialAnnounceSocket::Spdp,
                     RTPSInitialAnnounceAddressing::SenderAddr };
        case RTPSInitialAnnounceAction::SpdpDiscoveryPortCopy:
            return { RTPSInitialAnnounceSocket::Spdp,
                     RTPSInitialAnnounceAddressing::SenderAddrWithSpdpPort };
        case RTPSInitialAnnounceAction::SedpRosDiscoveryWriter:
        case RTPSInitialAnnounceAction::SedpRosDiscoveryReader:
        case RTPSInitialAnnounceAction::SedpChatterWriter:
        case RTPSInitialAnnounceAction::SedpChatterReader:
        case RTPSInitialAnnounceAction::ParticipantMessageData:
            return { RTPSInitialAnnounceSocket::Metatraffic,
                     RTPSInitialAnnounceAddressing::RemoteMetatrafficUnicast };
        case RTPSInitialAnnounceAction::RosDiscoveryUserData:
            return { RTPSInitialAnnounceSocket::UserData,
                     RTPSInitialAnnounceAddressing::RemoteUserDataUnicast };
        default:
            return {};
    }
}

RTPSInitialAnnounceBuildSpec RTPSInitialAnnouncePlan_getBuildSpec(
    RTPSInitialAnnounceAction action)
{
    switch (action)
    {
        case RTPSInitialAnnounceAction::SpdpUnicastReply:
            return { RTPSInitialAnnounceBuildKind::SpdpAnnouncement,
                     RTPSInitialAnnounceSedpEndpointProfile::None };
        case RTPSInitialAnnounceAction::SpdpDiscoveryPortCopy:
            // Built afresh, not ReusePrevious.  The sequence is drained one step
            // per loop pass, and between passes the shared send buffer is used
            // by heartbeats, auto-publish announces and ACKNACK replies - so
            // "reuse the previous step's bytes" sent the previous step's
            // *length* over whatever was in the buffer by then: a 256-byte
            // truncated SEDP announcement to the peer's discovery port, once per
            // participant that stayed around long enough to ACKNACK in between.
            return { RTPSInitialAnnounceBuildKind::SpdpAnnouncement,
                     RTPSInitialAnnounceSedpEndpointProfile::None };
        case RTPSInitialAnnounceAction::SedpRosDiscoveryWriter:
            return { RTPSInitialAnnounceBuildKind::SedpPublication,
                     RTPSInitialAnnounceSedpEndpointProfile::RosDiscoveryInfoWriter };
        case RTPSInitialAnnounceAction::SedpRosDiscoveryReader:
            return { RTPSInitialAnnounceBuildKind::SedpSubscription,
                     RTPSInitialAnnounceSedpEndpointProfile::RosDiscoveryInfoReader };
        case RTPSInitialAnnounceAction::SedpChatterWriter:
            return { RTPSInitialAnnounceBuildKind::SedpPublication,
                     RTPSInitialAnnounceSedpEndpointProfile::ChatterWriter };
        case RTPSInitialAnnounceAction::SedpChatterReader:
            return { RTPSInitialAnnounceBuildKind::SedpSubscription,
                     RTPSInitialAnnounceSedpEndpointProfile::ChatterReader };
        case RTPSInitialAnnounceAction::ParticipantMessageData:
            return { RTPSInitialAnnounceBuildKind::ParticipantMessageData,
                     RTPSInitialAnnounceSedpEndpointProfile::None };
        case RTPSInitialAnnounceAction::RosDiscoveryUserData:
            return { RTPSInitialAnnounceBuildKind::RosDiscoveryUserData,
                     RTPSInitialAnnounceSedpEndpointProfile::None };
        default:
            return {};
    }
}

RTPSInitialAnnounceSedpEndpointSpec RTPSInitialAnnouncePlan_getSedpEndpointSpec(
    RTPSInitialAnnounceSedpEndpointProfile profile)
{
    switch (profile)
    {
        case RTPSInitialAnnounceSedpEndpointProfile::RosDiscoveryInfoWriter:
            return { ENTITYID_ROS_DISC_INFO_WRITER,
                     ROS_DISCOVERY_INFO_TOPIC,
                     ROS_DISCOVERY_INFO_TYPE,
                     RELIABILITY_RELIABLE,
                     DURABILITY_TRANSIENT_LOCAL };
        case RTPSInitialAnnounceSedpEndpointProfile::RosDiscoveryInfoReader:
            return { ENTITYID_ROS_DISC_INFO_READER,
                     ROS_DISCOVERY_INFO_TOPIC,
                     ROS_DISCOVERY_INFO_TYPE,
                     RELIABILITY_RELIABLE,
                     DURABILITY_TRANSIENT_LOCAL };
        case RTPSInitialAnnounceSedpEndpointProfile::ChatterWriter:
            return { ENTITYID_CHATTER_WRITER,
                     CHATTER_DDS_TOPIC,
                     CHATTER_DDS_TYPE,
                     RELIABILITY_RELIABLE,
                     DURABILITY_VOLATILE };
        case RTPSInitialAnnounceSedpEndpointProfile::ChatterReader:
            return { ENTITYID_CHATTER_READER,
                     CHATTER_IN_DDS_TOPIC,
                     CHATTER_IN_DDS_TYPE,
                     RELIABILITY_RELIABLE,
                     DURABILITY_VOLATILE };
        default:
            return {};
    }
}

RTPSInitialAnnounceLogSpec RTPSInitialAnnouncePlan_getLogSpec(
    RTPSInitialAnnounceAction action)
{
    switch (action)
    {
        case RTPSInitialAnnounceAction::SpdpUnicastReply:
            return { "unicast SPDP", true, true };
        case RTPSInitialAnnounceAction::SedpRosDiscoveryWriter:
            return { "SEDP pub", false, true };
        case RTPSInitialAnnounceAction::SedpRosDiscoveryReader:
            return { "SEDP sub", false, true };
        case RTPSInitialAnnounceAction::SedpChatterWriter:
            return { "SEDP chatter pub", false, true };
        case RTPSInitialAnnounceAction::SedpChatterReader:
            return { "SEDP chatter sub", false, true };
        case RTPSInitialAnnounceAction::ParticipantMessageData:
            return { "liveliness", false, true };
        default:
            return {};
    }
}

uint64_t RTPSInitialAnnouncePlan_applySequencePolicy(
    const RTPSInitialAnnounceStep& step,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor,
    RTPSInitialAnnounceCounterState& counters)
{
    switch (step.seqCounterHint)
    {
        case RTPSSeqCounterHint::Spdp:
            counters.spdpSeqNum++;
            return counters.spdpSeqNum;
        case RTPSSeqCounterHint::SedpRosWriter:
            return counters.sedpRosWriterSeqNum;
        case RTPSSeqCounterHint::SedpRosReader:
            return counters.sedpRosReaderSeqNum;
        case RTPSSeqCounterHint::SedpChatterWriter:
            if (runtimeFlavor == RTPSInitialAnnounceRuntimeFlavor::LinuxStyle)
                return counters.sedpRosWriterSeqNum + 1;
            return counters.sedpChatterWriterSeqNum;
        case RTPSSeqCounterHint::SedpChatterReader:
            if (runtimeFlavor == RTPSInitialAnnounceRuntimeFlavor::LinuxStyle)
                return counters.sedpRosReaderSeqNum + 1;
            return counters.sedpChatterReaderSeqNum;
        case RTPSSeqCounterHint::Liveliness:
            return counters.livelinessSeqNum;
        case RTPSSeqCounterHint::RosDiscoveryUserData:
            return counters.rosDiscoveryUserDataSeqNum;
        default:
            return 0;
    }
}

RTPSInitialAnnounceDebugSpec RTPSInitialAnnouncePlan_getDebugSpec(
    RTPSInitialAnnounceAction action,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor)
{
    RTPSInitialAnnounceDebugSpec spec;
    if ((runtimeFlavor == RTPSInitialAnnounceRuntimeFlavor::LinuxStyle) &&
        (action == RTPSInitialAnnounceAction::SedpRosDiscoveryWriter))
    {
        spec.hexDumpPayloadToStderr = true;
    }
    return spec;
}

RTPSInitialAnnounceSendResultSpec RTPSInitialAnnouncePlan_classifySendResult(
    int sentBytes,
    uint32_t expectedBytes)
{
    RTPSInitialAnnounceSendResultSpec spec;
    if (expectedBytes == 0)
    {
        spec.resultKind = RTPSInitialAnnounceSendResultKind::NotSent;
        spec.shouldLog = false;
        spec.isFailure = false;
        return spec;
    }

    spec.shouldLog = true;
    if (sentBytes < 0)
    {
        spec.resultKind = RTPSInitialAnnounceSendResultKind::Failed;
        spec.isFailure = true;
        return spec;
    }

    if ((uint32_t)sentBytes < expectedBytes)
    {
        spec.resultKind = RTPSInitialAnnounceSendResultKind::ShortSend;
        spec.isFailure = true;
        return spec;
    }

    spec.resultKind = RTPSInitialAnnounceSendResultKind::Success;
    spec.isFailure = false;
    return spec;
}

RTPSInitialAnnounceStepEvaluation RTPSInitialAnnouncePlan_evaluatePreBuild(
    const RTPSInitialAnnounceStep& step,
    uint32_t previousPayloadLen)
{
    RTPSInitialAnnounceStepEvaluation eval;
    if ((step.action == RTPSInitialAnnounceAction::SpdpDiscoveryPortCopy) &&
        (previousPayloadLen == 0))
    {
        eval.shouldSkip = true;
        eval.reason = RTPSInitialAnnounceSkipReason::MissingPreviousPayload;
    }
    return eval;
}

RTPSInitialAnnounceStepEvaluation RTPSInitialAnnouncePlan_evaluatePostBuild(
    uint32_t payloadLen)
{
    RTPSInitialAnnounceStepEvaluation eval;
    if (payloadLen == 0)
    {
        eval.shouldSkip = true;
        eval.reason = RTPSInitialAnnounceSkipReason::EmptyPayload;
    }
    return eval;
}

uint32_t RTPSInitialAnnouncePlan_applyHeartbeatPolicy(
    const RTPSInitialAnnounceStep& step,
    uint32_t heartbeatCount)
{
    if (step.incrementHeartbeatBeforeSend)
        return heartbeatCount + 1;
    return heartbeatCount;
}