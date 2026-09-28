/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubServiceCodec - CDR for the std_srvs service types, whatever carried them
//
// A ROS 2 service request and its response are plain CDR, identical on both
// transports, so this sits above them like AutoPubStringMessage does.  Only
// the three std_srvs types are here: their type hashes ship with ROS 2 Jazzy,
// so a device can serve them without generating anything.
//
// One thing that is easy to get wrong: an *empty* ROS request is not empty on
// the wire.  `Trigger_Request` and both halves of `Empty` carry a single
// `uint8 structure_needs_at_least_one_member`, and a real client sends it.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

#include "CDRDecoder.h"
#include "CDREncoder.h"

namespace RaftRuntime::AutoPub
{

/// @brief The service types this codec knows
enum class AutoPubServiceKind : uint8_t
{
    Unknown = 0,
    Trigger,        ///< std_srvs/srv/Trigger: () -> (bool success, string message)
    SetBool,        ///< std_srvs/srv/SetBool: (bool data) -> (bool success, string message)
    Empty,          ///< std_srvs/srv/Empty:   () -> ()
};

/// @brief Wire type name for a kind, in the form a Zenoh service key carries
inline const char* AutoPubServiceCodec_wireType(AutoPubServiceKind kind)
{
    switch (kind)
    {
        case AutoPubServiceKind::Trigger: return "std_srvs::srv::dds_::Trigger_";
        case AutoPubServiceKind::SetBool: return "std_srvs::srv::dds_::SetBool_";
        case AutoPubServiceKind::Empty:   return "std_srvs::srv::dds_::Empty_";
        default:                          return nullptr;
    }
}

/// @brief Reverse of the above, for an application that names a type
inline AutoPubServiceKind AutoPubServiceCodec_kindForWireType(const char* wireType)
{
    if (!wireType)
        return AutoPubServiceKind::Unknown;
    for (uint8_t v = (uint8_t)AutoPubServiceKind::Trigger; v <= (uint8_t)AutoPubServiceKind::Empty; ++v)
    {
        const char* candidate = AutoPubServiceCodec_wireType((AutoPubServiceKind)v);
        if (candidate && std::strcmp(candidate, wireType) == 0)
            return (AutoPubServiceKind)v;
    }
    return AutoPubServiceKind::Unknown;
}

/// @brief A decoded request.  `data` is meaningful for SetBool only.
struct AutoPubServiceRequestFields
{
    AutoPubServiceKind kind = AutoPubServiceKind::Unknown;
    bool data = false;
};

/// @brief A response to encode.  `success`/`message` are used by Trigger and
/// SetBool; Empty ignores both.
struct AutoPubServiceResponseFields
{
    bool success = false;
    const char* message = "";
};

/// @brief Decode a request of the given kind
/// @return false if the payload is not a well-formed request of that kind
inline bool AutoPubServiceCodec_decodeRequest(AutoPubServiceKind kind, const uint8_t* payload, uint32_t length,
                                              AutoPubServiceRequestFields& out)
{
    if (!payload || kind == AutoPubServiceKind::Unknown)
        return false;
    CDRDecoder decoder;
    decoder.init(payload, length);
    if (!decoder.readEncapsulationHeader())
        return false;
    out.kind = kind;
    out.data = false;
    uint8_t byte = 0;
    switch (kind)
    {
        case AutoPubServiceKind::Trigger:
        case AutoPubServiceKind::Empty:
            // structure_needs_at_least_one_member - present, value ignored
            return decoder.readUint8(byte);
        case AutoPubServiceKind::SetBool:
            return decoder.readBool(out.data);
        default:
            return false;
    }
}

/// @brief Encode a response of the given kind
/// @return bytes written, 0 if it did not fit
inline uint32_t AutoPubServiceCodec_encodeResponse(AutoPubServiceKind kind, const AutoPubServiceResponseFields& in,
                                                   uint8_t* out, uint32_t capacity)
{
    if (!out || kind == AutoPubServiceKind::Unknown)
        return 0;
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader())
        return 0;
    switch (kind)
    {
        case AutoPubServiceKind::Trigger:
        case AutoPubServiceKind::SetBool:
            if (!encoder.writeBool(in.success) || !encoder.writeString(in.message ? in.message : ""))
                return 0;
            return encoder.getPos();
        case AutoPubServiceKind::Empty:
            if (!encoder.writeUint8(0))
                return 0;
            return encoder.getPos();
        default:
            return 0;
    }
}

} // namespace RaftRuntime::AutoPub
