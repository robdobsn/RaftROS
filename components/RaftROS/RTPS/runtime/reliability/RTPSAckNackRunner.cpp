#include "runtime/reliability/RTPSAckNackRunner.h"

namespace
{

struct RTPSAckNackRunnerEmitCtx
{
    const RTPSAckNackRunnerCallbacks* callbacks = nullptr;
    void* userCtx = nullptr;
    const RTPSAckNackFields* fields = nullptr;
    RTPSAckNackWriterKind writerKind = RTPSAckNackWriterKind::Unknown;
};

void RTPSAckNackRunner_emitAction(
    void* emitCtx,
    RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionAction runtimeAction)
{
    auto* ctx = static_cast<RTPSAckNackRunnerEmitCtx*>(emitCtx);
    if (!ctx || !ctx->callbacks || !ctx->callbacks->executeAction || !ctx->fields)
        return;

    ctx->callbacks->executeAction(ctx->userCtx, runtimeAction, *ctx->fields, ctx->writerKind);
}

}

void RTPSAckNackRunner_run(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    const RaftROS::RTPS::Runtime::ReliabilityAndWriterState::RTPSAckNackDecisionOptions& options,
    const RTPSAckNackRunnerCallbacks& callbacks,
    void* userCtx)
{
    RTPSAckNackFields fields;
    if (!RaftROS::RTPS::Runtime::ReliabilityAndWriterState::parseAckNack(
            pContent,
            contentLen,
            fields))
        return;

    const RTPSAckNackWriterKind writerKind =
        RaftROS::RTPS::Runtime::ReliabilityAndWriterState::classifyWriter(fields.writerEID);
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

    const uint64_t chatterSeq = callbacks.getChatterSeq ? callbacks.getChatterSeq(userCtx) : 0;
    RaftROS::RTPS::Runtime::ReliabilityAndWriterState::evaluateAckNackActions(
        fields,
        writerKind,
        options,
        chatterSeq,
        RTPSAckNackRunner_emitAction,
        &emitCtx);
}