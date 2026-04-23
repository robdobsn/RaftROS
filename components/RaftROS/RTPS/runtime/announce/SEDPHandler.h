/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SEDP Handler - Simple Endpoint Discovery Protocol message building
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include "RTPSTypes.h"

class RTPSParticipant;

class SEDPHandler
{
public:
    SEDPHandler();
    ~SEDPHandler();

    /// Build a complete SEDP publication announcement RTPS message
    /// This announces a DataWriter endpoint to a remote participant
    /// Returns bytes written, or 0 on error
    uint32_t buildPublicationMessage(
        uint8_t* pBuf, uint32_t bufLen,
        const RTPSParticipant& participant,
        const uint8_t* destGuidPrefix,
        const uint8_t* writerEntityId,       // Entity ID of the DataWriter being announced
        const char* topicName,
        const char* typeName,
        uint32_t reliabilityKind,
        uint32_t durabilityKind,
        uint64_t sequenceNumber,
        uint32_t ipAddrNetOrder = 0,
        uint32_t heartbeatCount = 0);

    /// Build an SEDP publication DISPOSE message (DDSI-RTPS §8.7.3.5 / §9.6.3.9).
    /// Sent on the builtin PublicationsWriter to tell peers the advertised
    /// DataWriter is going away.  Payload carries PID_STATUS_INFO with the
    /// DisposedFlag|UnregisteredFlag bits set plus PID_KEY_HASH / PID_ENDPOINT_GUID
    /// naming the vanished writer.  Returns bytes written, or 0 on error.
    uint32_t buildPublicationDisposeMessage(
        uint8_t* pBuf, uint32_t bufLen,
        const RTPSParticipant& participant,
        const uint8_t* destGuidPrefix,
        const uint8_t* writerEntityId,
        uint64_t sequenceNumber,
        uint32_t heartbeatCount = 0);

    /// Build a user DATA message wrapping a payload from a specific writer
    /// Used for publishing on topics like ros_discovery_info
    /// Returns bytes written, or 0 on error
    uint32_t buildUserDataMessage(
        uint8_t* pBuf, uint32_t bufLen,
        const RTPSParticipant& participant,
        const uint8_t* destGuidPrefix,
        const uint8_t* writerEntityId,
        const uint8_t* pPayload, uint32_t payloadLen,
        uint64_t sequenceNumber,
        uint32_t heartbeatCount,
        uint64_t firstSN = 1);

    /// Build a complete SEDP subscription announcement RTPS message
    /// This announces a DataReader endpoint to a remote participant
    uint32_t buildSubscriptionMessage(
        uint8_t* pBuf, uint32_t bufLen,
        const RTPSParticipant& participant,
        const uint8_t* destGuidPrefix,
        const uint8_t* readerEntityId,
        const char* topicName,
        const char* typeName,
        uint32_t reliabilityKind,
        uint32_t durabilityKind,
        uint64_t sequenceNumber,
        uint32_t ipAddrNetOrder = 0,
        uint32_t heartbeatCount = 0);

    /// Build a Participant Message Data (liveliness assertion) RTPS message
    uint32_t buildParticipantMessageData(
        uint8_t* pBuf, uint32_t bufLen,
        const RTPSParticipant& participant,
        const uint8_t* destGuidPrefix,
        uint64_t sequenceNumber,
        uint32_t heartbeatCount);

private:
    /// Write a CDR string into a ParameterList value area
    /// Returns bytes written (including padding to 4-byte alignment)
    static uint32_t writePLString(uint8_t* pBuf, const char* str);
};
