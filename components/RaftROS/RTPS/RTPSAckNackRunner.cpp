#include "RTPSAckNackRunner.h"

#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

namespace
{

struct RTPSAckNackRunnerEmitCtx
{
    const RTPSAckNackRunnerCallbacks* callbacks = nullptr;
    void* userCtx = nullptr;
    const RTPSAckNackFields* fields = nullptr;
    RTPSAckNackWriterKind writerKind = RTPSAckNackWriterKind::Unknown;
};

bool RTPSAckNackRunner_mapRuntimeAction(
    RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction runtimeAction,
    RTPSAckNackRunnerAction& runnerAction)
{
    using RuntimeAction = RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction;
    switch (runtimeAction)
    {
        case RuntimeAction::RetransmitSedpRosDiscoveryPublication:
            runnerAction = RTPSAckNackRunnerAction::RetransmitSedpRosDiscoveryPublication;
            return true;
        case RuntimeAction::RetransmitSedpChatterPublication:
            runnerAction = RTPSAckNackRunnerAction::RetransmitSedpChatterPublication;
            return true;
        case RuntimeAction::RetransmitSedpRosDiscoverySubscription:
            runnerAction = RTPSAckNackRunnerAction::RetransmitSedpRosDiscoverySubscription;
            return true;
        case RuntimeAction::RetransmitRosDiscoveryInfo:
            runnerAction = RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo;
            return true;
        case RuntimeAction::RetransmitChatterData:
            runnerAction = RTPSAckNackRunnerAction::RetransmitChatterData;
            return true;
        default:
            return false;
    }
}

void RTPSAckNackRunner_emitAction(
    void* emitCtx,
    RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction runtimeAction)
{
    auto* ctx = static_cast<RTPSAckNackRunnerEmitCtx*>(emitCtx);
    if (!ctx || !ctx->callbacks || !ctx->callbacks->executeAction || !ctx->fields)
        return;

    RTPSAckNackRunnerAction runnerAction;
    if (!RTPSAckNackRunner_mapRuntimeAction(runtimeAction, runnerAction))
        return;

    ctx->callbacks->executeAction(ctx->userCtx, runnerAction, *ctx->fields, ctx->writerKind);
}

}

void RTPSAckNackRunner_run(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    const RTPSAckNackRunnerOptions& options,
    const RTPSAckNackRunnerCallbacks& callbacks,
    void* userCtx)
{
    RTPSAckNackFields fields;
    if (!RTPSAckNack_parse(pContent, contentLen, fields))
        return;

    const RTPSAckNackWriterKind writerKind = RTPSAckNack_classifyWriter(fields.writerEID);
    if (callbacks.logParsed)
        callbacks.logParsed(userCtx, fields, writerKind);

    if (!callbacks.resolveRemote || !callbacks.resolveRemote(userCtx, srcGuidPrefix))
    {
        if (callbacks.unknownRemote)
            callbacks.unknownRemote(userCtx);
        return;
    }

    if (!callbacks.executeAction)
        return;

    RTPSAckNackRunnerEmitCtx emitCtx;
    emitCtx.callbacks = &callbacks;
    emitCtx.userCtx = userCtx;
    emitCtx.fields = &fields;
    emitCtx.writerKind = writerKind;

    RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionOptions runtimeOptions;
    runtimeOptions.publicationsIncludesChatterAnnouncement = options.publicationsIncludesChatterAnnouncement;
    runtimeOptions.requirePublicationSeq2GateForRetransmit =
        options.requirePublicationSeq2GateForRetransmit;

    const uint64_t chatterSeq = callbacks.getChatterSeq ? callbacks.getChatterSeq(userCtx) : 0;
    RaftROS::RTPS::Runtime::ReliabilityAndWriterState::evaluateAckNackActions(
        fields,
        writerKind,
        runtimeOptions,
        chatterSeq,
        RTPSAckNackRunner_emitAction,
        &emitCtx);
}
