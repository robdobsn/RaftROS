#include "runtime/announce/RTPSWriterHeartbeatRunner.h"

static void appendStep(RTPSWriterHeartbeatSequence& seq,
                       RTPSWriterHeartbeatAction action,
                       bool incrementHeartbeatBeforeBuild,
                       bool incrementLivelinessSeqBeforeBuild)
{
    if (seq.numSteps >= (sizeof(seq.steps) / sizeof(seq.steps[0])))
        return;
    seq.steps[seq.numSteps].action = action;
    seq.steps[seq.numSteps].incrementHeartbeatBeforeBuild = incrementHeartbeatBeforeBuild;
    seq.steps[seq.numSteps].incrementLivelinessSeqBeforeBuild = incrementLivelinessSeqBeforeBuild;
    seq.numSteps++;
}

RTPSWriterHeartbeatSequence RTPSWriterHeartbeatRunner_buildSequence(
    RTPSWriterHeartbeatRuntimeFlavor runtimeFlavor)
{
    RTPSWriterHeartbeatSequence seq;
    const bool linuxLikeHeartbeatOnSedp = (runtimeFlavor == RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle);

    appendStep(seq, RTPSWriterHeartbeatAction::SedpRosDiscoveryPublication,
               linuxLikeHeartbeatOnSedp, false);
    appendStep(seq, RTPSWriterHeartbeatAction::SedpRosDiscoverySubscription,
               linuxLikeHeartbeatOnSedp, false);
    appendStep(seq, RTPSWriterHeartbeatAction::SedpChatterPublication,
               linuxLikeHeartbeatOnSedp, false);
    appendStep(seq, RTPSWriterHeartbeatAction::SedpChatterSubscription,
               linuxLikeHeartbeatOnSedp, false);
    appendStep(seq, RTPSWriterHeartbeatAction::ParticipantMessageData,
               true, true);
    appendStep(seq, RTPSWriterHeartbeatAction::RosDiscoveryInfoData,
               true, false);

    return seq;
}

RTPSWriterHeartbeatSendTarget RTPSWriterHeartbeatRunner_sendTargetForAction(
    RTPSWriterHeartbeatAction action)
{
    if (action == RTPSWriterHeartbeatAction::RosDiscoveryInfoData)
        return RTPSWriterHeartbeatSendTarget::UserData;
    return RTPSWriterHeartbeatSendTarget::Metatraffic;
}

uint64_t RTPSWriterHeartbeatRunner_sequenceForAction(
    RTPSWriterHeartbeatAction action,
    RTPSWriterHeartbeatRuntimeFlavor runtimeFlavor,
    const RTPSWriterHeartbeatCounterState& counters)
{
    switch (action)
    {
        case RTPSWriterHeartbeatAction::SedpRosDiscoveryPublication:
            return counters.sedpSeqNum;
        case RTPSWriterHeartbeatAction::SedpRosDiscoverySubscription:
            return counters.sedpSubSeqNum;
        case RTPSWriterHeartbeatAction::SedpChatterPublication:
            if (runtimeFlavor == RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle)
                return counters.sedpSeqNum + 1;
            return counters.chatterSedpSeqNum;
        case RTPSWriterHeartbeatAction::SedpChatterSubscription:
            if (runtimeFlavor == RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle)
                return counters.sedpSubSeqNum + 1;
            return counters.chatterSedpSubSeqNum;
        case RTPSWriterHeartbeatAction::ParticipantMessageData:
            return counters.livelinessSeqNum;
        case RTPSWriterHeartbeatAction::RosDiscoveryInfoData:
            return counters.rosDiscSeqNum;
        default:
            return 0;
    }
}

void RTPSWriterHeartbeatRunner_run(
    RTPSWriterHeartbeatRuntimeFlavor runtimeFlavor,
    RTPSWriterHeartbeatCounterState& counters,
    const RTPSWriterHeartbeatRunnerCallbacks& callbacks,
    void* userCtx)
{
    if (!callbacks.buildPayload || !callbacks.sendPayload)
        return;

    const RTPSWriterHeartbeatSequence seq = RTPSWriterHeartbeatRunner_buildSequence(runtimeFlavor);
    for (uint8_t i = 0; i < seq.numSteps; i++)
    {
        const RTPSWriterHeartbeatStep& step = seq.steps[i];
        if (step.incrementHeartbeatBeforeBuild)
            counters.heartbeatCount++;
        if (step.incrementLivelinessSeqBeforeBuild)
            counters.livelinessSeqNum++;

        const uint64_t seqNum = RTPSWriterHeartbeatRunner_sequenceForAction(
            step.action, runtimeFlavor, counters);

        const uint32_t payloadLen = callbacks.buildPayload(
            userCtx, step.action, seqNum, counters.heartbeatCount);
        if (payloadLen == 0)
            continue;

        if ((runtimeFlavor == RTPSWriterHeartbeatRuntimeFlavor::LinuxStyle) &&
            (step.action == RTPSWriterHeartbeatAction::RosDiscoveryInfoData) &&
            !counters.rosDiscDebugDumped && callbacks.debugPayload)
        {
            callbacks.debugPayload(userCtx, step.action, payloadLen);
            counters.rosDiscDebugDumped = true;
        }

        const RTPSWriterHeartbeatSendTarget sendTarget =
            RTPSWriterHeartbeatRunner_sendTargetForAction(step.action);
        const int sent = callbacks.sendPayload(userCtx, sendTarget, payloadLen);

        if (callbacks.logSend)
            callbacks.logSend(userCtx, step.action, sent, payloadLen, sendTarget);
    }
}