/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubEndpointDesc - transport-neutral description of one auto-published endpoint
//
// What to publish, not how.  A descriptor owns its strings (bounded, no heap)
// so a backend can keep it for the lifetime of the endpoint without the caller
// having to keep the source buffers alive.  Backends derive their own wire
// names and identities from this (RTPS entity ids, Zenoh keys and tokens).
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include "AutoPubClassMap.h"
#include "AutoPubQoSProfile.h"

namespace RaftRuntime {
namespace AutoPub {

// Bounds mirror the buffers previously used in the RTPS wrapper
static constexpr uint32_t AUTOPUB_TOPIC_MAX_LEN = 64;
static constexpr uint32_t AUTOPUB_TYPE_MAX_LEN = 64;

/// @brief Identity of the bus device an endpoint publishes for.  `subIndex`
/// distinguishes the endpoints of a composite device (e.g. temperature and
/// humidity from one sensor).
struct AutoPubDeviceId
{
    uint8_t busNum = 0;
    uint32_t address = 0;
    uint8_t subIndex = 0;

    bool operator==(const AutoPubDeviceId& other) const
    {
        return busNum == other.busNum && address == other.address && subIndex == other.subIndex;
    }
};

/// @brief One endpoint to create: ROS topic and type, the message kind the
/// serializer emits, and semantic QoS.  Strings are owned and NUL-terminated.
struct AutoPubEndpointDesc
{
    AutoPubDeviceId deviceId;
    char topic[AUTOPUB_TOPIC_MAX_LEN] = {};
    char type[AUTOPUB_TYPE_MAX_LEN] = {};
    AutoPubMsgKind msgKind = AutoPubMsgKind::Unknown;
    AutoPubQoSProfileId qosProfileId = AutoPubQoSProfileId::FallbackString;

    bool isValid() const { return topic[0] != '\0' && type[0] != '\0'; }

    /// @brief Copy in topic/type, rejecting anything that would truncate
    bool setNames(const char* topicIn, const char* typeIn)
    {
        topic[0] = '\0';
        type[0] = '\0';
        if (!topicIn || !typeIn)
            return false;
        const size_t topicLen = std::strlen(topicIn);
        const size_t typeLen = std::strlen(typeIn);
        if ((topicLen == 0) || (topicLen >= AUTOPUB_TOPIC_MAX_LEN) ||
            (typeLen == 0) || (typeLen >= AUTOPUB_TYPE_MAX_LEN))
            return false;
        std::memcpy(topic, topicIn, topicLen + 1);
        std::memcpy(type, typeIn, typeLen + 1);
        return true;
    }
};

} // namespace AutoPub
} // namespace RaftRuntime
