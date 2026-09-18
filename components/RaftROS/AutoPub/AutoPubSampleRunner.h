/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubSampleRunner — transport-neutral dispatch of one decoded sample
//
// Selects the latest record of a decoded batch, serialises it into one or two
// caller-owned CDR buffers (primary then secondary, for composite devices) and
// hands each serialised payload to a synchronous publish callable.
//
// Ownership: everything is borrowed. The batch, field descriptors, output
// buffers and result array must stay valid, and must not be modified by the
// publish callable, for the whole run() call. The callable must send or copy
// the payload before returning; the buffer is reused by the next sample.
// There is no allocation, queue, worker, locking or transport identity here.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AutoPubCDRSerializer.h"
#include <cstring>

namespace RaftRuntime::AutoPub
{

/// @brief Backend outcome reported by a publish callable. Values describe
/// what happened to one payload; they do not imply queueing or QoS guarantees.
enum class AutoPubPublishResult : uint8_t
{
    NotAttempted,
    Accepted,
    QueueFull,
    Disconnected,
    InvalidHandle,
    Oversized,
    SendFailed,
};

/// @brief Borrowed view of decoder output. `capacity` is the true size of
/// `data` in bytes; the runner trusts it and checks recordCount * recordSize
/// against it. Each record must start with a uint32 `timeMs` (code-generator
/// convention) to set the Header stamp.
struct AutoPubDecodedBatch
{
    const uint8_t* data = nullptr;
    uint32_t capacity = 0;
    uint32_t recordSize = 0;
    uint32_t recordCount = 0;
    const AutoPubAttrFieldDesc* fields = nullptr;
    uint16_t fieldCount = 0;
    const char* frameId = "raft";
};

/// @brief One caller-owned serialisation target. Unknown kind, null data or
/// zero capacity disables the output (it is neither serialised nor published).
struct AutoPubSampleOutput
{
    AutoPubMsgKind kind = AutoPubMsgKind::Unknown;
    uint8_t* data = nullptr;
    uint32_t capacity = 0;
};

/// @brief Per-output outcome. publishResult stays NotAttempted unless the
/// output serialised to a non-empty payload.
struct AutoPubSampleResult
{
    uint32_t bytesWritten = 0;
    bool serialized = false;
    AutoPubPublishResult publishResult = AutoPubPublishResult::NotAttempted;
};

class AutoPubSampleRunner
{
public:
    static constexpr uint8_t MAX_OUTPUTS = 2;

    /// @brief Serialise the latest record into each enabled output, in order,
    /// calling `publish(outputIndex, payload, length, timestampMs)` (returning
    /// AutoPubPublishResult) synchronously after each successful serialisation.
    /// A failed output does not suppress later outputs.
    /// @return false if the arguments or batch are invalid (results are reset
    /// when `results` is usable); true means the batch was valid, not that
    /// every output serialised or was delivered — inspect `results`.
    template<typename Publish>
    static bool run(const AutoPubDecodedBatch& batch,
                    const AutoPubSampleOutput* outputs, uint8_t outputCount,
                    AutoPubSampleResult* results, Publish&& publish)
    {
        if (!results || outputCount == 0 || outputCount > MAX_OUTPUTS)
            return false;
        for (uint8_t outputIndex = 0; outputIndex < outputCount; ++outputIndex)
            results[outputIndex] = {};
        if (!outputs || !batch.data || batch.recordSize == 0 || batch.recordCount == 0 ||
            batch.recordCount > batch.capacity / batch.recordSize ||
            !batch.fields || batch.fieldCount == 0)
            return false;

        const uint8_t* latest = batch.data + (batch.recordCount - 1) * batch.recordSize;
        AutoPubCDRContext context;
        context.pFieldDescs = batch.fields;
        context.fieldCount = batch.fieldCount;
        context.pStruct = latest;
        context.structSize = batch.recordSize;
        context.frameId = batch.frameId;
        if (batch.recordSize >= sizeof(context.timestampMs))
            std::memcpy(&context.timestampMs, latest, sizeof(context.timestampMs));

        for (uint8_t outputIndex = 0; outputIndex < outputCount; ++outputIndex)
        {
            const auto& output = outputs[outputIndex];
            auto& result = results[outputIndex];
            if (output.kind == AutoPubMsgKind::Unknown || !output.data || output.capacity == 0)
                continue;
            result.serialized = AutoPubCDRSerializer_serialize(
                output.kind, context, output.data, output.capacity, result.bytesWritten);
            if (result.serialized && result.bytesWritten > 0)
                result.publishResult = publish(outputIndex, output.data,
                                               result.bytesWritten, context.timestampMs);
        }
        return true;
    }
};

}