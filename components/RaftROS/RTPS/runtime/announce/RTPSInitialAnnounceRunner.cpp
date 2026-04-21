#include "runtime/announce/RTPSInitialAnnounceRunner.h"

void RTPSInitialAnnounceRunner_run(
    const RTPSInitialAnnounceSequence& sequence,
    RTPSInitialAnnounceRunnerContext& runCtx,
    const RTPSInitialAnnounceRunnerCallbacks& callbacks,
    void* userCtx)
{
    if (!callbacks.buildPayload || !callbacks.sendPayload)
        return;

    for (uint8_t stepIdx = 0; stepIdx < sequence.numSteps; stepIdx++)
    {
        const RTPSInitialAnnounceStep& step = sequence.steps[stepIdx];

        const RTPSInitialAnnounceStepEvaluation preBuildEval =
            RTPSInitialAnnouncePlan_evaluatePreBuild(step, runCtx.previousPayloadLen);
        if (preBuildEval.shouldSkip)
            continue;

        runCtx.heartbeatCount = RTPSInitialAnnouncePlan_applyHeartbeatPolicy(step, runCtx.heartbeatCount);

        const uint64_t seqNum = RTPSInitialAnnouncePlan_applySequencePolicy(
            step, runCtx.runtimeFlavor, runCtx.seqCounters);

        const uint32_t payloadLen = callbacks.buildPayload(
            userCtx, step, seqNum, runCtx.heartbeatCount, runCtx.previousPayloadLen);

        const RTPSInitialAnnounceStepEvaluation postBuildEval =
            RTPSInitialAnnouncePlan_evaluatePostBuild(payloadLen);
        if (postBuildEval.shouldSkip)
            continue;

        runCtx.previousPayloadLen = payloadLen;

        const RTPSInitialAnnounceDebugSpec debugSpec =
            RTPSInitialAnnouncePlan_getDebugSpec(step.action, runCtx.runtimeFlavor);
        if (debugSpec.hexDumpPayloadToStderr && callbacks.debugPayload)
            callbacks.debugPayload(userCtx, step, payloadLen);

        const int sent = callbacks.sendPayload(userCtx, step.action, payloadLen);
        const RTPSInitialAnnounceSendResultSpec sendResult =
            RTPSInitialAnnouncePlan_classifySendResult(sent, payloadLen);
        if (!sendResult.shouldLog)
            continue;

        const RTPSInitialAnnounceLogSpec logSpec = RTPSInitialAnnouncePlan_getLogSpec(step.action);
        if (!logSpec.enabled || !logSpec.actionLabel)
            continue;

        if (callbacks.logResult)
            callbacks.logResult(userCtx, step, logSpec, sendResult, sent, payloadLen);
    }
}