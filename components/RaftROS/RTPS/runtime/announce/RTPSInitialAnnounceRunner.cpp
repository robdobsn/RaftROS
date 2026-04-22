#include "runtime/announce/RTPSInitialAnnounceRunner.h"

bool RTPSInitialAnnounceRunner_runStep(
    const RTPSInitialAnnounceSequence& sequence,
    uint8_t stepIdx,
    RTPSInitialAnnounceRunnerContext& runCtx,
    const RTPSInitialAnnounceRunnerCallbacks& callbacks,
    void* userCtx)
{
    if (stepIdx >= sequence.numSteps)
        return false;
    if (!callbacks.buildPayload || !callbacks.sendPayload)
        return true;

    const RTPSInitialAnnounceStep& step = sequence.steps[stepIdx];

    const RTPSInitialAnnounceStepEvaluation preBuildEval =
        RTPSInitialAnnouncePlan_evaluatePreBuild(step, runCtx.previousPayloadLen);
    if (preBuildEval.shouldSkip)
        return true;

    runCtx.heartbeatCount = RTPSInitialAnnouncePlan_applyHeartbeatPolicy(step, runCtx.heartbeatCount);

    const uint64_t seqNum = RTPSInitialAnnouncePlan_applySequencePolicy(
        step, runCtx.runtimeFlavor, runCtx.seqCounters);

    const uint32_t payloadLen = callbacks.buildPayload(
        userCtx, step, seqNum, runCtx.heartbeatCount, runCtx.previousPayloadLen);

    const RTPSInitialAnnounceStepEvaluation postBuildEval =
        RTPSInitialAnnouncePlan_evaluatePostBuild(payloadLen);
    if (postBuildEval.shouldSkip)
        return true;

    runCtx.previousPayloadLen = payloadLen;

    const RTPSInitialAnnounceDebugSpec debugSpec =
        RTPSInitialAnnouncePlan_getDebugSpec(step.action, runCtx.runtimeFlavor);
    if (debugSpec.hexDumpPayloadToStderr && callbacks.debugPayload)
        callbacks.debugPayload(userCtx, step, payloadLen);

    const int sent = callbacks.sendPayload(userCtx, step.action, payloadLen);
    const RTPSInitialAnnounceSendResultSpec sendResult =
        RTPSInitialAnnouncePlan_classifySendResult(sent, payloadLen);
    if (!sendResult.shouldLog)
        return true;

    const RTPSInitialAnnounceLogSpec logSpec = RTPSInitialAnnouncePlan_getLogSpec(step.action);
    if (!logSpec.enabled || !logSpec.actionLabel)
        return true;

    if (callbacks.logResult)
        callbacks.logResult(userCtx, step, logSpec, sendResult, sent, payloadLen);
    return true;
}

void RTPSInitialAnnounceRunner_run(
    const RTPSInitialAnnounceSequence& sequence,
    RTPSInitialAnnounceRunnerContext& runCtx,
    const RTPSInitialAnnounceRunnerCallbacks& callbacks,
    void* userCtx)
{
    for (uint8_t stepIdx = 0; stepIdx < sequence.numSteps; stepIdx++)
        RTPSInitialAnnounceRunner_runStep(sequence, stepIdx, runCtx, callbacks, userCtx);
}