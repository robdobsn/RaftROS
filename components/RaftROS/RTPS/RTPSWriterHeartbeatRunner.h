#pragma once

#include <stdint.h>

enum class RTPSWriterHeartbeatRuntimeFlavor : uint8_t
{
    EspStyle,
    LinuxStyle
};

enum class RTPSWriterHeartbeatAction : uint8_t
{
    SedpRosDiscoveryPublication,
    SedpRosDiscoverySubscription,
    SedpChatterPublication,
    ParticipantMessageData,
    RosDiscoveryInfoData,
};

enum class RTPSWriterHeartbeatSendTarget : uint8_t
{
    Metatraffic,
    UserData,
};

struct RTPSWriterHeartbeatStep
{
    RTPSWriterHeartbeatAction action = RTPSWriterHeartbeatAction::SedpRosDiscoveryPublication;
    bool incrementHeartbeatBeforeBuild = false;
    bool incrementLivelinessSeqBeforeBuild = false;
};

struct RTPSWriterHeartbeatSequence
{
    RTPSWriterHeartbeatStep steps[5];
    uint8_t numSteps = 0;
};

struct RTPSWriterHeartbeatCounterState
{
    uint64_t sedpSeqNum = 0;
    uint64_t sedpSubSeqNum = 0;
    uint64_t chatterSedpSeqNum = 0;
    uint64_t livelinessSeqNum = 0;
    uint64_t rosDiscSeqNum = 0;
    uint32_t heartbeatCount = 0;
    bool rosDiscDebugDumped = false;
};

typedef uint32_t (*RTPSWriterHeartbeatBuildPayloadFn)(
    void* userCtx,
    RTPSWriterHeartbeatAction action,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount);

typedef int (*RTPSWriterHeartbeatSendPayloadFn)(
    void* userCtx,
    RTPSWriterHeartbeatSendTarget sendTarget,
    uint32_t payloadLen);

typedef void (*RTPSWriterHeartbeatDebugPayloadFn)(
    void* userCtx,
    RTPSWriterHeartbeatAction action,
    uint32_t payloadLen);

typedef void (*RTPSWriterHeartbeatLogSendFn)(
    void* userCtx,
    RTPSWriterHeartbeatAction action,
    int sentBytes,
    uint32_t payloadLen,
    RTPSWriterHeartbeatSendTarget sendTarget);

struct RTPSWriterHeartbeatRunnerCallbacks
{
    RTPSWriterHeartbeatBuildPayloadFn buildPayload = nullptr;
    RTPSWriterHeartbeatSendPayloadFn sendPayload = nullptr;
    RTPSWriterHeartbeatDebugPayloadFn debugPayload = nullptr;
    RTPSWriterHeartbeatLogSendFn logSend = nullptr;
};

RTPSWriterHeartbeatSequence RTPSWriterHeartbeatRunner_buildSequence(
    RTPSWriterHeartbeatRuntimeFlavor runtimeFlavor);

RTPSWriterHeartbeatSendTarget RTPSWriterHeartbeatRunner_sendTargetForAction(
    RTPSWriterHeartbeatAction action);

uint64_t RTPSWriterHeartbeatRunner_sequenceForAction(
    RTPSWriterHeartbeatAction action,
    RTPSWriterHeartbeatRuntimeFlavor runtimeFlavor,
    const RTPSWriterHeartbeatCounterState& counters);

void RTPSWriterHeartbeatRunner_run(
    RTPSWriterHeartbeatRuntimeFlavor runtimeFlavor,
    RTPSWriterHeartbeatCounterState& counters,
    const RTPSWriterHeartbeatRunnerCallbacks& callbacks,
    void* userCtx);
