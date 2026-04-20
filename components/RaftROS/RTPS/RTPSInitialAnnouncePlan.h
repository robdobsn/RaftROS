#pragma once

#include <stdint.h>

struct RTPSInitialAnnouncePlan
{
    bool sendUnicastSpdpReply = true;
    bool sendSpdpDiscoveryPortCopy = true;
    bool sendSedpRosDiscoveryWriter = true;
    bool sendSedpRosDiscoveryReader = true;
    bool sendSedpChatterWriter = true;
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
    ChatterWriter
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
    uint64_t livelinessSeqNum = 0;
    uint64_t rosDiscoveryUserDataSeqNum = 0;
};

struct RTPSInitialAnnounceSequence
{
    RTPSInitialAnnounceStep steps[7];
    uint8_t numSteps = 0;
};

// Returns the policy for first-contact announcement bundle on participant discovery.
RTPSInitialAnnouncePlan RTPSInitialAnnouncePlan_default();

// Builds ordered initial-announce actions with side-effect guidance.
RTPSInitialAnnounceSequence RTPSInitialAnnouncePlan_buildSequence(
    const RTPSInitialAnnouncePlan& plan,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor);

// Maps an action to its send socket/addressing policy.
RTPSInitialAnnounceSendTarget RTPSInitialAnnouncePlan_getSendTarget(
    RTPSInitialAnnounceAction action);

// Maps an action to its payload builder kind and endpoint profile.
RTPSInitialAnnounceBuildSpec RTPSInitialAnnouncePlan_getBuildSpec(
    RTPSInitialAnnounceAction action);

// Maps a SEDP endpoint profile to concrete entity/topic/type/qos values.
RTPSInitialAnnounceSedpEndpointSpec RTPSInitialAnnouncePlan_getSedpEndpointSpec(
    RTPSInitialAnnounceSedpEndpointProfile profile);

// Maps an action to shared logging metadata.
RTPSInitialAnnounceLogSpec RTPSInitialAnnouncePlan_getLogSpec(
    RTPSInitialAnnounceAction action);

// Applies sequence-counter mutation timing policy for a step and returns the sequence number to use.
uint64_t RTPSInitialAnnouncePlan_applySequencePolicy(
    const RTPSInitialAnnounceStep& step,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor,
    RTPSInitialAnnounceCounterState& counters);

// Maps action/runtime to optional debug behavior.
RTPSInitialAnnounceDebugSpec RTPSInitialAnnouncePlan_getDebugSpec(
    RTPSInitialAnnounceAction action,
    RTPSInitialAnnounceRuntimeFlavor runtimeFlavor);

// Classifies send result for shared instrumentation policy.
RTPSInitialAnnounceSendResultSpec RTPSInitialAnnouncePlan_classifySendResult(
    int sentBytes,
    uint32_t expectedBytes);

// Evaluates whether a step should be skipped before building payload.
RTPSInitialAnnounceStepEvaluation RTPSInitialAnnouncePlan_evaluatePreBuild(
    const RTPSInitialAnnounceStep& step,
    uint32_t previousPayloadLen);

// Evaluates whether a step should be skipped after build.
RTPSInitialAnnounceStepEvaluation RTPSInitialAnnouncePlan_evaluatePostBuild(
    uint32_t payloadLen);

// Applies heartbeat mutation policy for a step.
uint32_t RTPSInitialAnnouncePlan_applyHeartbeatPolicy(
    const RTPSInitialAnnounceStep& step,
    uint32_t heartbeatCount);
