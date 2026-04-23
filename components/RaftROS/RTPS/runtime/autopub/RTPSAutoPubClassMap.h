/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubClassMap — Device `clas[]` → ROS 2 message-type mapping
//
// Phase 4 / Slice 4.4.  Pure, header-only, no RaftCore / ESP-IDF dependencies
// so the mapping logic can be unit-tested in the Linux host harness against
// every row in `DeviceTypeRecords.json`.
//
// API:
//   RTPSAutoPubClassMap_lookup(clas[], count, deviceTypeName)
//       → RTPSAutoPubClassMapping { primary, secondary, excluded }
//
// Precedence (first match wins):
//   1. Device-type-name overrides (e.g. MCP9808 tagged LGHT → TEMP).
//   2. Actuator classes → excluded (no publishing).
//   3. Composite rules — first match wins among:
//        a. {ACC, GYRO} ⊆ clas           → Imu (single writer).
//        b. {TEMP, RH}  ⊆ clas           → Temperature + RelativeHumidity.
//        c. {PRES, TEMP}⊆ clas           → FluidPressure + Temperature.
//   4. Single-class rules (TEMP, RH, PRES, LGHT, PROX, DIST, ANG, ROT,
//      ACC, TCH, BTN, FRCE, HRM, SOIL, GAME).
//   5. Fallback: std_msgs/String (slug "raw").
//
// The returned `primaryTopicSlug` / `secondaryTopicSlug` are short bare slugs
// (no "rt/raft/" prefix, no device-address suffix).  The full topic is
// formatted by the caller as `rt/<namespace>/<alias>/<slug>` in later slices.
//
// Slice 4.10 will extend the per-device-type overrides for BTHome
// per-attribute splitting; today BLEBTHome falls through to the fallback
// String writer, which keeps Success Criterion 5 satisfied.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace RaftRuntime {
namespace RTPS {
namespace Runtime {
namespace AutoPub {

// ---------------------------------------------------------------------------
// Message kinds supported by the auto-publisher.  "Accel" is Imu emitted from
// a device tagged only `ACC` (gyro / orientation fields zeroed with "unknown"
// covariance marker); it is a distinct enum value so §4.5 serialisers can
// pick a different CDR field-population path while reusing the Imu type.
// ---------------------------------------------------------------------------
enum class RTPSAutoPubMsgKind : uint8_t
{
    Unknown = 0,
    Imu,
    Accel,
    Temperature,
    RelativeHumidity,
    FluidPressure,
    Illuminance,
    Range,
    Float32,
    Int32,
    Bool,
    ByteMultiArray,
    Wrench,
    Float32MultiArray,
    Joy,
    String,
};

/// @brief Full ROS 2 wire type name for a message kind.
/// @return Static literal `"<pkg>::msg::dds_::<Type>_"`, or nullptr for Unknown.
inline const char* RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind kind)
{
    switch (kind)
    {
        case RTPSAutoPubMsgKind::Imu:
        case RTPSAutoPubMsgKind::Accel:
            return "sensor_msgs::msg::dds_::Imu_";
        case RTPSAutoPubMsgKind::Temperature:
            return "sensor_msgs::msg::dds_::Temperature_";
        case RTPSAutoPubMsgKind::RelativeHumidity:
            return "sensor_msgs::msg::dds_::RelativeHumidity_";
        case RTPSAutoPubMsgKind::FluidPressure:
            return "sensor_msgs::msg::dds_::FluidPressure_";
        case RTPSAutoPubMsgKind::Illuminance:
            return "sensor_msgs::msg::dds_::Illuminance_";
        case RTPSAutoPubMsgKind::Range:
            return "sensor_msgs::msg::dds_::Range_";
        case RTPSAutoPubMsgKind::Float32:
            return "std_msgs::msg::dds_::Float32_";
        case RTPSAutoPubMsgKind::Int32:
            return "std_msgs::msg::dds_::Int32_";
        case RTPSAutoPubMsgKind::Bool:
            return "std_msgs::msg::dds_::Bool_";
        case RTPSAutoPubMsgKind::ByteMultiArray:
            return "std_msgs::msg::dds_::ByteMultiArray_";
        case RTPSAutoPubMsgKind::Wrench:
            return "geometry_msgs::msg::dds_::Wrench_";
        case RTPSAutoPubMsgKind::Float32MultiArray:
            return "std_msgs::msg::dds_::Float32MultiArray_";
        case RTPSAutoPubMsgKind::Joy:
            return "sensor_msgs::msg::dds_::Joy_";
        case RTPSAutoPubMsgKind::String:
            return "std_msgs::msg::dds_::String_";
        case RTPSAutoPubMsgKind::Unknown:
        default:
            return nullptr;
    }
}

/// @brief Result of a class-map lookup.  `primary*` is always populated unless
///        `excluded == true`.  `secondary*` is populated only for composite
///        two-writer rules (TEMP+RH, PRES+TEMP).
struct RTPSAutoPubClassMapping
{
    RTPSAutoPubMsgKind primaryKind   = RTPSAutoPubMsgKind::Unknown;
    const char*        primaryTopicSlug   = nullptr;
    RTPSAutoPubMsgKind secondaryKind = RTPSAutoPubMsgKind::Unknown;
    const char*        secondaryTopicSlug = nullptr;
    bool               excluded = false;

    bool hasPrimary()   const { return primaryKind   != RTPSAutoPubMsgKind::Unknown; }
    bool hasSecondary() const { return secondaryKind != RTPSAutoPubMsgKind::Unknown; }
};

// ---------------------------------------------------------------------------
// Internal helpers — exposed inline so everything stays header-only.
// ---------------------------------------------------------------------------

inline bool RTPSAutoPubClassMap_hasClas(
        const char* const* clasArray, size_t clasCount, const char* code)
{
    if (!clasArray || !code)
        return false;
    for (size_t i = 0; i < clasCount; i++)
    {
        if (clasArray[i] && std::strcmp(clasArray[i], code) == 0)
            return true;
    }
    return false;
}

/// @brief Look up the auto-publish mapping for a bus device.
/// @param clasArray        array of class-code strings (from `devInfoJson.clas`)
/// @param clasCount        number of entries in clasArray
/// @param deviceTypeName   device type name (nullable) — used for per-device
///                         overrides such as the MCP9808 `LGHT` mis-tag.
/// @return mapping with `primary`/`secondary` kinds + topic slugs, or
///         `excluded = true` for actuators (SRVO/PUMP/PIX).
inline RTPSAutoPubClassMapping RTPSAutoPubClassMap_lookup(
        const char* const* clasArray, size_t clasCount,
        const char* deviceTypeName)
{
    RTPSAutoPubClassMapping out;

    // -----------------------------------------------------------------
    // 1. Device-type-name overrides
    // -----------------------------------------------------------------
    if (deviceTypeName)
    {
        if (std::strcmp(deviceTypeName, "MCP9808") == 0)
        {
            // JSON mis-tag workaround: MCP9808 is a temperature sensor but
            // currently carries clas=["LGHT"].  Force TEMP mapping regardless.
            out.primaryKind = RTPSAutoPubMsgKind::Temperature;
            out.primaryTopicSlug = "temperature";
            return out;
        }
        if (std::strcmp(deviceTypeName, "RoboticalLightSensor") == 0)
        {
            // 4-attribute light sensor: left/center/right.  Use MultiArray
            // rather than Illuminance so all three channels are carried.
            out.primaryKind = RTPSAutoPubMsgKind::Float32MultiArray;
            out.primaryTopicSlug = "light";
            return out;
        }
    }

    // -----------------------------------------------------------------
    // 2. Actuator classes → excluded (no poll data to publish).
    // -----------------------------------------------------------------
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "SRVO") ||
        RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "PUMP") ||
        RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "PIX"))
    {
        out.excluded = true;
        return out;
    }

    // -----------------------------------------------------------------
    // 3. Composite rules (first match wins)
    // -----------------------------------------------------------------
    const bool hasAcc  = RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "ACC");
    const bool hasGyro = RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "GYRO");
    const bool hasTemp = RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "TEMP");
    const bool hasRH   = RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "RH");
    const bool hasPres = RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "PRES");

    if (hasAcc && hasGyro)
    {
        out.primaryKind = RTPSAutoPubMsgKind::Imu;
        out.primaryTopicSlug = "imu";
        return out;
    }
    if (hasTemp && hasRH)
    {
        out.primaryKind   = RTPSAutoPubMsgKind::Temperature;
        out.primaryTopicSlug   = "temperature";
        out.secondaryKind = RTPSAutoPubMsgKind::RelativeHumidity;
        out.secondaryTopicSlug = "humidity";
        return out;
    }
    if (hasPres && hasTemp)
    {
        out.primaryKind   = RTPSAutoPubMsgKind::FluidPressure;
        out.primaryTopicSlug   = "pressure";
        out.secondaryKind = RTPSAutoPubMsgKind::Temperature;
        out.secondaryTopicSlug = "temperature";
        return out;
    }

    // -----------------------------------------------------------------
    // 4. Single-class rules
    // -----------------------------------------------------------------
    if (hasTemp) { out.primaryKind = RTPSAutoPubMsgKind::Temperature;      out.primaryTopicSlug = "temperature"; return out; }
    if (hasRH)   { out.primaryKind = RTPSAutoPubMsgKind::RelativeHumidity; out.primaryTopicSlug = "humidity";    return out; }
    if (hasPres) { out.primaryKind = RTPSAutoPubMsgKind::FluidPressure;    out.primaryTopicSlug = "pressure";    return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "LGHT"))
        { out.primaryKind = RTPSAutoPubMsgKind::Illuminance; out.primaryTopicSlug = "illuminance"; return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "PROX"))
        { out.primaryKind = RTPSAutoPubMsgKind::Range;       out.primaryTopicSlug = "proximity";   return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "DIST"))
        { out.primaryKind = RTPSAutoPubMsgKind::Range;       out.primaryTopicSlug = "range";       return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "ANG"))
        { out.primaryKind = RTPSAutoPubMsgKind::Float32;     out.primaryTopicSlug = "angle";       return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "ROT"))
        { out.primaryKind = RTPSAutoPubMsgKind::Int32;       out.primaryTopicSlug = "encoder";     return out; }
    if (hasAcc)
        { out.primaryKind = RTPSAutoPubMsgKind::Accel;       out.primaryTopicSlug = "accel";       return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "TCH"))
        { out.primaryKind = RTPSAutoPubMsgKind::ByteMultiArray; out.primaryTopicSlug = "touch";    return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "BTN"))
        { out.primaryKind = RTPSAutoPubMsgKind::Bool;        out.primaryTopicSlug = "button";      return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "FRCE"))
        { out.primaryKind = RTPSAutoPubMsgKind::Wrench;      out.primaryTopicSlug = "force";       return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "HRM"))
        { out.primaryKind = RTPSAutoPubMsgKind::Float32MultiArray; out.primaryTopicSlug = "ppg";   return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "SOIL"))
        { out.primaryKind = RTPSAutoPubMsgKind::Float32;     out.primaryTopicSlug = "soil_moisture"; return out; }
    if (RTPSAutoPubClassMap_hasClas(clasArray, clasCount, "GAME"))
        { out.primaryKind = RTPSAutoPubMsgKind::Joy;         out.primaryTopicSlug = "joy";         return out; }

    // -----------------------------------------------------------------
    // 5. Fallback — unknown clas, or `BTHM` (pending Slice 4.10 split).
    // -----------------------------------------------------------------
    out.primaryKind = RTPSAutoPubMsgKind::String;
    out.primaryTopicSlug = "raw";
    return out;
}

} // namespace AutoPub
} // namespace Runtime
} // namespace RTPS
} // namespace RaftRuntime
