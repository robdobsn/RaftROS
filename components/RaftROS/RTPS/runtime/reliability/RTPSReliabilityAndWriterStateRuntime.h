#pragma once

#include <cstdint>

namespace RaftRuntime::RTPS::Runtime::ReliabilityAndWriterState
{

enum class RTPSAckNackWriterKind
{
    Unknown = 0,
    SedpPublications,
    SedpSubscriptions,
    RosDiscoveryInfo,
    Chatter,
    ChatterReader,
    ParticipantMessage,
};

struct RTPSAckNackFields
{
    const uint8_t* readerEID = nullptr;
    const uint8_t* writerEID = nullptr;
    uint32_t bitmapBaseLow = 0;
    uint32_t numBits = 0;
};

enum class RTPSAckNackRuntimeFlavor : uint8_t
{
    EspStyle = 0,
    LinuxStandalone,
};

enum class RTPSAckNackDecisionAction
{
    RetransmitSedpRosDiscoveryPublication = 0,
    RetransmitSedpChatterPublication,
    RetransmitSedpRosDiscoverySubscription,
    RetransmitRosDiscoveryInfo,
    RetransmitChatterData,
};

struct RTPSAckNackDecisionOptions
{
    bool publicationsIncludesChatterAnnouncement = false;
    bool requirePublicationSeq2GateForRetransmit = false;
};

struct RTPSAckNackMutationPolicy
{
    bool incrementHeartbeatOnSedpRetransmit = false;
    bool incrementHeartbeatOnUserDataRetransmit = false;
};

struct RTPSAckActionSedpEndpointProfile
{
    const uint8_t* entityId = nullptr;
    const char* topicName = nullptr;
    const char* typeName = nullptr;
    uint32_t reliabilityKind = 0;
    uint32_t durabilityKind = 0;
};

struct RTPSAckActionSedpSequenceContext
{
    uint64_t rosDiscoveryPublicationSeqNum = 0;
    uint64_t rosDiscoverySubscriptionSeqNum = 0;
    uint64_t chatterPublicationSeqNum = 0;
    bool deriveChatterPublicationFromRosDiscoveryPublication = false;
};

struct RTPSAckActionSedpPlan
{
    RTPSAckActionSedpEndpointProfile endpoint;
    uint64_t sequenceNumber = 0;
};

struct RTPSAckActionUserDataSequenceContext
{
    uint64_t rosDiscoveryInfoSeqNum = 0;
    uint64_t chatterDataSeqNum = 0;
    bool chatterFirstSNMatchesSequence = false;
};

struct RTPSAckActionUserDataPlan
{
    const uint8_t* writerEntityId = nullptr;
    uint64_t sequenceNumber = 0;
    bool hasFirstSNOverride = false;
    uint64_t firstSN = 1;
};

RTPSAckActionSedpSequenceContext makeAckSedpSequenceContext(
    uint64_t rosDiscoveryPublicationSeqNum,
    uint64_t rosDiscoverySubscriptionSeqNum,
    uint64_t chatterPublicationSeqNum,
    bool deriveChatterPublicationFromRosDiscoveryPublication);

RTPSAckActionUserDataSequenceContext makeAckUserDataSequenceContext(
    uint64_t rosDiscoveryInfoSeqNum,
    uint64_t chatterDataSeqNum,
    bool chatterFirstSNMatchesSequence);

RTPSAckNackDecisionOptions makeAckNackDecisionOptionsForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor);

RTPSAckNackMutationPolicy makeAckNackMutationPolicyForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor);

RTPSAckActionSedpSequenceContext makeAckSedpSequenceContextForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor,
    uint64_t rosDiscoveryPublicationSeqNum,
    uint64_t rosDiscoverySubscriptionSeqNum,
    uint64_t chatterPublicationSeqNum);

RTPSAckActionUserDataSequenceContext makeAckUserDataSequenceContextForFlavor(
    RTPSAckNackRuntimeFlavor runtimeFlavor,
    uint64_t rosDiscoveryInfoSeqNum,
    uint64_t chatterDataSeqNum);

typedef void (*RTPSAckNackDecisionEmitActionFn)(
    void* emitCtx,
    RTPSAckNackDecisionAction action);

bool shouldRespondToHeartbeat(uint8_t heartbeatFlags);

bool acknackRequestsSeq(uint32_t bitmapBaseLow, uint64_t seqNum);

bool parseAckNack(const uint8_t* pContent,
                  uint32_t contentLen,
                  RTPSAckNackFields& outFields);

RTPSAckNackWriterKind classifyWriter(const uint8_t* writerEID);

const char* writerKindToStr(RTPSAckNackWriterKind writerKind);

const uint8_t* localReaderForRemoteWriter(const uint8_t* remoteWriterEID,
                                          const uint8_t* fallbackReaderEID);

void evaluateAckNackActions(const RTPSAckNackFields& fields,
                            RTPSAckNackWriterKind writerKind,
                            const RTPSAckNackDecisionOptions& options,
                            uint64_t chatterSeq,
                            RTPSAckNackDecisionEmitActionFn emitAction,
                            void* emitCtx);

void applyAckActionHeartbeatMutation(RTPSAckNackDecisionAction action,
                                     const RTPSAckNackMutationPolicy& mutationPolicy,
                                     uint32_t& heartbeatCount);

bool getAckActionSedpPublicationProfile(RTPSAckNackDecisionAction action,
                                        RTPSAckActionSedpEndpointProfile& outProfile);

bool getAckActionSedpSubscriptionProfile(RTPSAckNackDecisionAction action,
                                         RTPSAckActionSedpEndpointProfile& outProfile);

uint64_t getAckActionSedpSequenceNumber(
    RTPSAckNackDecisionAction action,
    const RTPSAckActionSedpSequenceContext& sequenceContext,
    bool& outHasSequence);

bool getAckActionSedpPlan(
    RTPSAckNackDecisionAction action,
    const RTPSAckActionSedpSequenceContext& sequenceContext,
    RTPSAckActionSedpPlan& outPlan);

bool getAckActionUserDataPlan(
    RTPSAckNackDecisionAction action,
    const RTPSAckActionUserDataSequenceContext& sequenceContext,
    RTPSAckActionUserDataPlan& outPlan);

}
