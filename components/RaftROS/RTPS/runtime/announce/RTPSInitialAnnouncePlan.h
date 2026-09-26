#pragma once

#include <stdint.h>

struct RTPSInitialAnnouncePlan
{
    bool sendUnicastSpdpReply = true;
    bool sendSpdpDiscoveryPortCopy = true;
    bool sendSedpRosDiscoveryWriter = true;
    bool sendSedpRosDiscoveryReader = true;
    bool sendSedpChatterWriter = true;
    bool sendSedpChatterReader = false;
    bool sendParticipantMessageData = true;
    bool sendInitialRosDiscoveryUserData = false;
};

enum class RTPSInitialAnnounceRuntimeFlavor : uint8_t
{
    EspStyle,
    LinuxStyle
};

enum class RTPSInitialAnnounceAction : uint8_t
{
    SpdpUnicastReply,
    SpdpDiscoveryPortCopy,
    SedpRosDiscoveryWriter,
    SedpRosDiscoveryReader,
    SedpChatterWriter,
    SedpChatterReader,
    ParticipantMessageData,
    RosDiscoveryUserData
};

enum class RTPSInitialAnnounceSocket : uint8_t
{
    None,
    Spdp,
    Metatraffic,
    UserData
};

enum class RTPSInitialAnnounceAddressing : uint8_t
{
    None,
    SenderAddr,
    SenderAddrWithSpdpPort,
    RemoteMetatrafficUnicast,
    RemoteUserDataUnicast
};

struct RTPSInitialAnnounceSendTarget
{
    RTPSInitialAnnounceSocket socket = RTPSInitialAnnounceSocket::None;
    RTPSInitialAnnounceAddressing addressing = RTPSInitialAnnounceAddressing::None;
};

enum class RTPSInitialAnnounceBuildKind : uint8_t
{
    None,
    ReusePrevious,
    SpdpAnnouncement,
    SedpPublication,
    SedpSubscription,
    ParticipantMessageData,
    RosDiscoveryUserData
};

enum class RTPSInitialAnnounceSedpEndpointProfile : uint8_t
{
    None,
    RosDiscoveryInfoWriter,
    RosDiscoveryInfoReader,
    ChatterWriter,
    ChatterReader
};

struct RTPSInitialAnnounceBuildSpec
{
    RTPSInitialAnnounceBuildKind buildKind = RTPSInitialAnnounceBuildKind::None;
    RTPSInitialAnnounceSedpEndpointProfile sedpEndpointProfile =
        RTPSInitialAnnounceSedpEndpointProfile::None;
};

struct RTPSInitialAnnounceSedpEndpointSpec
{
    const uint8_t* entityId = nullptr;
    const char* topicName = nullptr;
    const char* typeName = nullptr;
    uint32_t reliabilityKind = 0;
    uint32_t durabilityKind = 0;
};

struct RTPSInitialAnnounceLogSpec
{
    const char* actionLabel = nullptr;
    bool logToSenderAddress = false;
    bool enabled = false;
};

struct RTPSInitialAnnounceDebugSpec
{
    bool hexDumpPayloadToStderr = false;
};

enum class RTPSInitialAnnounceSendResultKind : uint8_t
{
    NotSent,
    Success,
    Failed,
    ShortSend
};

enum class RTPSInitialAnnounceSkipReason : uint8_t
{
    None,
    MissingPreviousPayload,
    EmptyPayload
};

struct RTPSInitialAnnounceSendResultSpec
{
    RTPSInitialAnnounceSendResultKind resultKind = RTPSInitialAnnounceSendResultKind::NotSent;
    bool shouldLog = false;
    bool isFailure = false;
};

struct RTPSInitialAnnounceStepEvaluation
{
    bool shouldSkip = false;
    RTPSInitialAnnounceSkipReason reason = RTPSInitialAnnounceSkipReason::None;
};

enum class RTPSSeqCounterHint : uint8_t
{
    None,
    Spdp,
    SedpRosWriter,
    SedpRosReader,
    SedpChatterWriter,
    SedpChatterReader,
    Liveliness,
    RosDiscoveryUserData
};

struct RTPSInitialAnnounceStep
{
    RTPSInitialAnnounceAction action = RTPSInitialAnnounceAction::SpdpUnicastReply;
    bool incrementHeartbeatBeforeSend = false;
    RTPSSeqCounterHint seqCounterHint = RTPSSeqCounterHint::None;
};

struct RTPSInitialAnnounceCounterState
{
    uint64_t spdpSeqNum = 0;
    uint64_t sedpRosWriterSeqNum = 0;
    uint64_t sedpRosReaderSeqNum = 0;
    uint64_t sedpChatterWriterSeqNum = 0;
    uint64_t sedpChatterReaderSeqNum = 0;
    uint64_t livelinessSeqNum = 1;   ///< RTPS sequence numbers start at 1
    uint64_t rosDiscoveryUserDataSeqNum = 0;
};

struct RTPSInitialAnnounceSequence
{
    RTPSInitialAnnounceStep steps[8];
    uint8_t numSteps = 0;
};

RTPSInitialAnnouncePlan RTPSInitialAnnouncePlan_default();

RTPSInitialAnnounceSequence RTPSInitialAnnouncePlan_buildSequence(
    const RTPSInitialAnnouncePlan& plan,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor);

RTPSInitialAnnounceSendTarget RTPSInitialAnnouncePlan_getSendTarget(
    RTPSInitialAnnounceAction action);

RTPSInitialAnnounceBuildSpec RTPSInitialAnnouncePlan_getBuildSpec(
    RTPSInitialAnnounceAction action);

RTPSInitialAnnounceSedpEndpointSpec RTPSInitialAnnouncePlan_getSedpEndpointSpec(
    RTPSInitialAnnounceSedpEndpointProfile profile);

RTPSInitialAnnounceLogSpec RTPSInitialAnnouncePlan_getLogSpec(
    RTPSInitialAnnounceAction action);

uint64_t RTPSInitialAnnouncePlan_applySequencePolicy(
    const RTPSInitialAnnounceStep& step,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor,
    RTPSInitialAnnounceCounterState& counters);

RTPSInitialAnnounceDebugSpec RTPSInitialAnnouncePlan_getDebugSpec(
    RTPSInitialAnnounceAction action,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor);

RTPSInitialAnnounceSendResultSpec RTPSInitialAnnouncePlan_classifySendResult(
    int sentBytes,
    uint32_t expectedBytes);

RTPSInitialAnnounceStepEvaluation RTPSInitialAnnouncePlan_evaluatePreBuild(
    const RTPSInitialAnnounceStep& step,
    uint32_t previousPayloadLen);

RTPSInitialAnnounceStepEvaluation RTPSInitialAnnouncePlan_evaluatePostBuild(
    uint32_t payloadLen);

uint32_t RTPSInitialAnnouncePlan_applyHeartbeatPolicy(
    const RTPSInitialAnnounceStep& step,
    uint32_t heartbeatCount);