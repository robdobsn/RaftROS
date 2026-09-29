/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubParamCodec.h
//
// CDR for the ROS 2 parameter services (rcl_interfaces, Jazzy): the requests
// the `ros2 param` tools send and the responses a node returns.  Byte layouts
// were taken from a capture of the real tools against an rclpy node
// (2026-09-28; linux_unit_tests/fixtures/zenoh_param_*.cdr.hex).
//
// RaftCore-free, like the rest of AutoPub's codecs.  Scalars only: a value
// that arrives as an array is reported as such (type kept, contents skipped)
// so the store can refuse it by name; we never hold one.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include "CDR/CDRDecoder.h"
#include "CDR/CDREncoder.h"

namespace RaftRuntime::AutoPub
{

static constexpr uint32_t AUTOPUB_PARAM_NAME_MAX = 48;      ///< Including the terminator
static constexpr uint32_t AUTOPUB_PARAM_STRING_MAX = 64;    ///< Including the terminator
static constexpr uint32_t AUTOPUB_PARAM_TEXT_MAX = 96;      ///< Description / reason, including the terminator

/// @brief rcl_interfaces/msg/ParameterType
enum class AutoPubParamType : uint8_t
{
    NotSet = 0, Bool = 1, Integer = 2, Double = 3, String = 4,
    ByteArray = 5, BoolArray = 6, IntegerArray = 7, DoubleArray = 8, StringArray = 9
};

/// @brief rcl_interfaces/msg/ParameterValue, scalars held, arrays noted
struct AutoPubParamValue
{
    AutoPubParamType type = AutoPubParamType::NotSet;
    bool boolValue = false;
    int64_t integerValue = 0;
    double doubleValue = 0.0;
    char stringValue[AUTOPUB_PARAM_STRING_MAX] = {};
    bool stringTooLong = false;     ///< The string on the wire did not fit; stringValue is truncated
    bool hasArrayData = false;      ///< One of the array fields was non-empty (contents skipped)
};

/// @brief rcl_interfaces/msg/Parameter
struct AutoPubParamEntry
{
    char name[AUTOPUB_PARAM_NAME_MAX] = {};
    bool nameTooLong = false;
    AutoPubParamValue value;
};

/// @brief rcl_interfaces/msg/SetParametersResult
struct AutoPubParamResult
{
    bool successful = false;
    const char* reason = "";
};

/// @brief rcl_interfaces/msg/ParameterDescriptor, the fields we fill
struct AutoPubParamDescriptor
{
    const char* name = "";
    AutoPubParamType type = AutoPubParamType::NotSet;
    const char* description = "";
    bool readOnly = false;
};

/// @brief A read string, truncated to the buffer; reports the wire length
inline bool AutoPubParamCodec_readString(CDRDecoder& decoder, char* out, uint32_t capacity, bool& tooLong)
{
    uint32_t strLen = 0;
    if (!decoder.readString(out, capacity, strLen))
        return false;
    tooLong = strLen > capacity - 1;
    return true;
}

/// @brief string[] into fixed name slots; names past `maxNames` are counted but not kept
inline bool AutoPubParamCodec_readNames(const uint8_t* payload, uint32_t length,
                                        char (*names)[AUTOPUB_PARAM_NAME_MAX], uint32_t maxNames, uint32_t& count)
{
    count = 0;
    if (!payload)
        return false;
    CDRDecoder decoder;
    decoder.init(payload, length);
    uint32_t wireCount = 0;
    if (!decoder.readEncapsulationHeader() || !decoder.readSequenceLength(wireCount))
        return false;
    for (uint32_t index = 0; index < wireCount; ++index)
    {
        char scratch[AUTOPUB_PARAM_NAME_MAX];
        bool tooLong = false;
        char* target = index < maxNames ? names[index] : scratch;
        if (!AutoPubParamCodec_readString(decoder, target, AUTOPUB_PARAM_NAME_MAX, tooLong))
            return false;
        if (tooLong)
            target[0] = '\0';               // cannot name any parameter we hold
    }
    count = wireCount;
    return true;
}

/// @brief ListParameters request: string[] prefixes, uint64 depth
inline bool AutoPubParamCodec_readListRequest(const uint8_t* payload, uint32_t length,
                                              char (*prefixes)[AUTOPUB_PARAM_NAME_MAX], uint32_t maxPrefixes,
                                              uint32_t& prefixCount, uint64_t& depth)
{
    prefixCount = 0;
    depth = 0;
    if (!payload)
        return false;
    CDRDecoder decoder;
    decoder.init(payload, length);
    uint32_t wireCount = 0;
    if (!decoder.readEncapsulationHeader() || !decoder.readSequenceLength(wireCount))
        return false;
    for (uint32_t index = 0; index < wireCount; ++index)
    {
        char scratch[AUTOPUB_PARAM_NAME_MAX];
        bool tooLong = false;
        char* target = index < maxPrefixes ? prefixes[index] : scratch;
        if (!AutoPubParamCodec_readString(decoder, target, AUTOPUB_PARAM_NAME_MAX, tooLong))
            return false;
        if (tooLong)
            target[0] = '\0';
    }
    prefixCount = wireCount;
    return decoder.readUint64(depth);
}

/// @brief One ParameterValue: scalars kept, array contents skipped
inline bool AutoPubParamCodec_readValue(CDRDecoder& decoder, AutoPubParamValue& value)
{
    value = AutoPubParamValue();
    uint8_t type = 0;
    if (!decoder.readUint8(type) || !decoder.readBool(value.boolValue) ||
        !decoder.readInt64(value.integerValue) || !decoder.readFloat64(value.doubleValue) ||
        !AutoPubParamCodec_readString(decoder, value.stringValue, sizeof(value.stringValue), value.stringTooLong))
        return false;
    value.type = type <= (uint8_t)AutoPubParamType::StringArray ? (AutoPubParamType)type : AutoPubParamType::NotSet;
    uint32_t count = 0;
    // byte[], bool[]: one byte each
    for (int seq = 0; seq < 2; ++seq)
    {
        if (!decoder.readSequenceLength(count) || !decoder.skip(count))
            return false;
        value.hasArrayData = value.hasArrayData || count != 0;
    }
    // int64[], float64[]: aligned to 8, eight bytes each
    for (int seq = 0; seq < 2; ++seq)
    {
        if (!decoder.readSequenceLength(count))
            return false;
        if (count != 0 && (!decoder.align(8) || !decoder.skip(count * 8)))
            return false;
        value.hasArrayData = value.hasArrayData || count != 0;
    }
    // string[]
    if (!decoder.readSequenceLength(count))
        return false;
    value.hasArrayData = value.hasArrayData || count != 0;
    for (uint32_t index = 0; index < count; ++index)
    {
        uint32_t strLen = 0;
        if (!decoder.readString(nullptr, 0, strLen))
            return false;
    }
    return true;
}

/// @brief SetParameters / SetParametersAtomically request: Parameter[]
inline bool AutoPubParamCodec_readSetRequest(const uint8_t* payload, uint32_t length,
                                             AutoPubParamEntry* entries, uint32_t maxEntries, uint32_t& count)
{
    count = 0;
    if (!payload)
        return false;
    CDRDecoder decoder;
    decoder.init(payload, length);
    uint32_t wireCount = 0;
    if (!decoder.readEncapsulationHeader() || !decoder.readSequenceLength(wireCount))
        return false;
    for (uint32_t index = 0; index < wireCount; ++index)
    {
        AutoPubParamEntry scratch;
        AutoPubParamEntry& target = index < maxEntries ? entries[index] : scratch;
        target = AutoPubParamEntry();
        if (!AutoPubParamCodec_readString(decoder, target.name, sizeof(target.name), target.nameTooLong) ||
            !AutoPubParamCodec_readValue(decoder, target.value))
            return false;
    }
    count = wireCount;
    return true;
}

/// @brief One ParameterValue, every field in order (arrays always empty)
inline bool AutoPubParamCodec_writeValue(CDREncoder& encoder, const AutoPubParamValue& value)
{
    if (!encoder.writeUint8((uint8_t)value.type) || !encoder.writeBool(value.boolValue) ||
        !encoder.writeInt64(value.integerValue) || !encoder.writeFloat64(value.doubleValue) ||
        !encoder.writeString(value.stringValue))
        return false;
    for (int seq = 0; seq < 5; ++seq)
        if (!encoder.writeSequenceLength(0))
            return false;
    return true;
}

/// @brief GetParameters response: ParameterValue[]
inline uint32_t AutoPubParamCodec_writeValues(uint8_t* out, uint32_t capacity,
                                              const AutoPubParamValue* values, uint32_t count)
{
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader() || !encoder.writeSequenceLength(count))
        return 0;
    for (uint32_t index = 0; index < count; ++index)
        if (!AutoPubParamCodec_writeValue(encoder, values[index]))
            return 0;
    return encoder.getPos();
}

/// @brief GetParameterTypes response: uint8[]
inline uint32_t AutoPubParamCodec_writeTypes(uint8_t* out, uint32_t capacity,
                                             const AutoPubParamType* types, uint32_t count)
{
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader() || !encoder.writeSequenceLength(count))
        return 0;
    for (uint32_t index = 0; index < count; ++index)
        if (!encoder.writeUint8((uint8_t)types[index]))
            return 0;
    return encoder.getPos();
}

inline bool AutoPubParamCodec_writeResult(CDREncoder& encoder, const AutoPubParamResult& result)
{
    return encoder.writeBool(result.successful) && encoder.writeString(result.reason ? result.reason : "");
}

/// @brief SetParameters response: SetParametersResult[]
inline uint32_t AutoPubParamCodec_writeResults(uint8_t* out, uint32_t capacity,
                                               const AutoPubParamResult* results, uint32_t count)
{
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader() || !encoder.writeSequenceLength(count))
        return 0;
    for (uint32_t index = 0; index < count; ++index)
        if (!AutoPubParamCodec_writeResult(encoder, results[index]))
            return 0;
    return encoder.getPos();
}

/// @brief SetParametersAtomically response: one SetParametersResult
inline uint32_t AutoPubParamCodec_writeSingleResult(uint8_t* out, uint32_t capacity, const AutoPubParamResult& result)
{
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader() || !AutoPubParamCodec_writeResult(encoder, result))
        return 0;
    return encoder.getPos();
}

/// @brief ListParameters response: ListParametersResult { string[] names; string[] prefixes }
inline uint32_t AutoPubParamCodec_writeListResult(uint8_t* out, uint32_t capacity,
                                                  const char* const* names, uint32_t nameCount,
                                                  const char* const* prefixes, uint32_t prefixCount)
{
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader() || !encoder.writeSequenceLength(nameCount))
        return 0;
    for (uint32_t index = 0; index < nameCount; ++index)
        if (!encoder.writeString(names[index]))
            return 0;
    if (!encoder.writeSequenceLength(prefixCount))
        return 0;
    for (uint32_t index = 0; index < prefixCount; ++index)
        if (!encoder.writeString(prefixes[index]))
            return 0;
    return encoder.getPos();
}

/// @brief DescribeParameters response: ParameterDescriptor[], ranges always empty
inline uint32_t AutoPubParamCodec_writeDescriptors(uint8_t* out, uint32_t capacity,
                                                   const AutoPubParamDescriptor* descriptors, uint32_t count)
{
    CDREncoder encoder;
    encoder.reset(out, capacity);
    if (!encoder.writeEncapsulationHeader() || !encoder.writeSequenceLength(count))
        return 0;
    for (uint32_t index = 0; index < count; ++index)
    {
        const AutoPubParamDescriptor& d = descriptors[index];
        if (!encoder.writeString(d.name) || !encoder.writeUint8((uint8_t)d.type) ||
            !encoder.writeString(d.description) || !encoder.writeString("") ||
            !encoder.writeBool(d.readOnly) || !encoder.writeBool(false) ||
            !encoder.writeSequenceLength(0) || !encoder.writeSequenceLength(0))
            return 0;
    }
    return encoder.getPos();
}

} // namespace RaftRuntime::AutoPub
