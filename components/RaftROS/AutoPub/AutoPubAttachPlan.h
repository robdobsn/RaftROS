/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubAttachPlan - decide which endpoints a bus device should publish
//
// Pure decision logic shared by all backends: class mapping -> ROS topic/type
// names -> semantic QoS.  No transport, no allocation, no RaftCore types.  The
// caller supplies QoS override resolution (SysTypes alias/class overrides live
// in the SysMod), so this header stays unit-testable on the host.
//
// Output-only devices are excluded, an unmapped class falls back to a labelled
// std_msgs/Float64MultiArray with the "data" slug, and a composite device
// yields up to two further endpoints with subIndex 1 and 2.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <cstring>
#include "AutoPubClassMap.h"
#include "AutoPubEndpointDesc.h"
#include "AutoPubTopicNaming.h"

namespace RaftRuntime {
namespace AutoPub {

/// @brief Endpoints a device should publish (a primary and up to two composite extras)
struct AutoPubAttachPlan
{
    static constexpr uint8_t MAX_ENDPOINTS = 3;
    bool excluded = false;                          ///< Device is an actuator - publish nothing
    uint8_t endpointCount = 0;
    AutoPubEndpointDesc endpoints[MAX_ENDPOINTS];
};

/// @brief The per-device alias a QoS override matches against: the last path
/// segment of the topic (e.g. "range_1_29" in "rt/raft/range_1_29")
inline const char* AutoPubAttachPlan_topicAlias(const char* topic)
{
    if (!topic)
        return "";
    const char* slash = std::strrchr(topic, '/');
    return slash ? slash + 1 : topic;
}

/// @brief Build the plan for a device.
/// @param deviceId bus/address of the device (subIndex is assigned per endpoint)
/// @param clasArray device class tags (e.g. "TEMP", "RH"), may be null
/// @param clasCount number of class tags
/// @param deviceTypeName device type name from the type record, may be null
/// @param resolveQoS callable (alias, clasArray, clasCount, deviceTypeName) -> AutoPubQoSProfileId
/// @return plan; endpointCount 0 means nothing to publish (excluded, or a name
///         that would not fit the descriptor)
template<typename ResolveQoS>
inline AutoPubAttachPlan AutoPubAttachPlan_build(
        const AutoPubDeviceId& deviceId,
        const char* const* clasArray, size_t clasCount,
        const char* deviceTypeName,
        ResolveQoS&& resolveQoS)
{
    AutoPubAttachPlan plan;
    const AutoPubClassMapping mapping = AutoPubClassMap_lookup(clasArray, clasCount, deviceTypeName);
    if (mapping.excluded)
    {
        plan.excluded = true;
        return plan;
    }

    // Primary endpoint.  A successful lookup always yields a slug; fall back
    // explicitly in case a future mapping returns something unexpected.
    const char* const primarySlug = mapping.primaryTopicSlug ? mapping.primaryTopicSlug : "raw";
    const char* const primaryType = AutoPubClassMap_typeName(mapping.primaryKind)
                                  ? AutoPubClassMap_typeName(mapping.primaryKind)
                                  : AUTOPUB_FALLBACK_TYPE;
    char topicBuf[AUTOPUB_TOPIC_MAX_LEN];
    if (!AutoPubTopicNaming_formatClassTopic(topicBuf, sizeof(topicBuf), primarySlug,
                                             deviceId.busNum, deviceId.address))
        return plan;

    AutoPubEndpointDesc& primary = plan.endpoints[0];
    if (!primary.setNames(topicBuf, primaryType))
        return plan;
    primary.deviceId = deviceId;
    primary.deviceId.subIndex = 0;
    primary.msgKind = mapping.primaryKind;
    primary.qosProfileId = resolveQoS(AutoPubAttachPlan_topicAlias(primary.topic),
                                      clasArray, clasCount, deviceTypeName);
    plan.endpointCount = 1;

    // Composite extra endpoints (e.g. AHT20 -> Temperature + RelativeHumidity,
    // SCD40 -> those plus a CO2 Float32).  An extra that cannot be named is
    // dropped, along with any after it; the primary still publishes.
    const AutoPubMsgKind extraKinds[] = {mapping.secondaryKind, mapping.tertiaryKind};
    const char* const extraSlugs[] = {mapping.secondaryTopicSlug, mapping.tertiaryTopicSlug};
    for (uint8_t extraIndex = 0; extraIndex < 2; ++extraIndex)
    {
        if (extraKinds[extraIndex] == AutoPubMsgKind::Unknown)
            return plan;
        const char* const extraSlug = extraSlugs[extraIndex] ? extraSlugs[extraIndex] : "data2";
        const char* const extraType = AutoPubClassMap_typeName(extraKinds[extraIndex])
                                    ? AutoPubClassMap_typeName(extraKinds[extraIndex])
                                    : AUTOPUB_FALLBACK_TYPE;
        char extraTopicBuf[AUTOPUB_TOPIC_MAX_LEN];
        if (!AutoPubTopicNaming_formatClassTopic(extraTopicBuf, sizeof(extraTopicBuf), extraSlug,
                                                 deviceId.busNum, deviceId.address))
            return plan;

        AutoPubEndpointDesc& extra = plan.endpoints[1 + extraIndex];
        if (!extra.setNames(extraTopicBuf, extraType))
        {
            extra = AutoPubEndpointDesc();
            return plan;
        }
        extra.deviceId = deviceId;
        extra.deviceId.subIndex = 1 + extraIndex;
        extra.msgKind = extraKinds[extraIndex];
        extra.qosProfileId = resolveQoS(AutoPubAttachPlan_topicAlias(extra.topic),
                                        clasArray, clasCount, deviceTypeName);
        plan.endpointCount = 2 + extraIndex;
    }
    return plan;
}

} // namespace AutoPub
} // namespace RaftRuntime
