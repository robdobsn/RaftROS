/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubClassMap — Device `clas[]` → ROS 2 message-type mapping
//
// Phase 4 / Slice 4.4.  Pure, header-only, no RaftCore / ESP-IDF dependencies
// so the mapping logic can be unit-tested in the Linux host harness against
// every row in `DeviceTypeRecords.json`.
//
// API:
//   AutoPubClassMap_lookup(clas[], count, deviceTypeName)
//       → AutoPubClassMapping { primary, secondary, excluded }
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
namespace AutoPub {

// ---------------------------------------------------------------------------
// Message kinds supported by the auto-publisher.  "Accel" is Imu emitted from
// a device tagged only `ACC` (gyro / orientation fields zeroed with "unknown"
// covariance marker); it is a distinct enum value so §4.5 serialisers can
// pick a different CDR field-population path while reusing the Imu type.
// ---------------------------------------------------------------------------
enum class AutoPubMsgKind : uint8_t
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
inline const char* AutoPubClassMap_typeName(AutoPubMsgKind kind)
{
    switch (kind)
    {
        case AutoPubMsgKind::Imu:
        case AutoPubMsgKind::Accel:
            return "sensor_msgs::msg::dds_::Imu_";
        case AutoPubMsgKind::Temperature:
            return "sensor_msgs::msg::dds_::Temperature_";
        case AutoPubMsgKind::RelativeHumidity:
            return "sensor_msgs::msg::dds_::RelativeHumidity_";
        case AutoPubMsgKind::FluidPressure:
            return "sensor_msgs::msg::dds_::FluidPressure_";
        case AutoPubMsgKind::Illuminance:
            return "sensor_msgs::msg::dds_::Illuminance_";
        case AutoPubMsgKind::Range:
            return "sensor_msgs::msg::dds_::Range_";
        case AutoPubMsgKind::Float32:
            return "std_msgs::msg::dds_::Float32_";
        case AutoPubMsgKind::Int32:
            return "std_msgs::msg::dds_::Int32_";
        case AutoPubMsgKind::Bool:
            return "std_msgs::msg::dds_::Bool_";
        case AutoPubMsgKind::ByteMultiArray:
            return "std_msgs::msg::dds_::ByteMultiArray_";
        case AutoPubMsgKind::Wrench:
            return "geometry_msgs::msg::dds_::Wrench_";
        case AutoPubMsgKind::Float32MultiArray:
            return "std_msgs::msg::dds_::Float32MultiArray_";
        case AutoPubMsgKind::Joy:
            return "sensor_msgs::msg::dds_::Joy_";
        case AutoPubMsgKind::String:
            return "std_msgs::msg::dds_::String_";
        case AutoPubMsgKind::Unknown:
        default:
            return nullptr;
    }
}

/// @brief Result of a class-map lookup.  `primary*` is always populated unless
///        `excluded == true`.  `secondary*` is populated only for composite
///        two-writer rules (TEMP+RH, PRES+TEMP).
struct AutoPubClassMapping
{
    AutoPubMsgKind primaryKind   = AutoPubMsgKind::Unknown;
    const char*        primaryTopicSlug   = nullptr;
    AutoPubMsgKind secondaryKind = AutoPubMsgKind::Unknown;
    const char*        secondaryTopicSlug = nullptr;
    bool               excluded = false;

    bool hasPrimary()   const { return primaryKind   != AutoPubMsgKind::Unknown; }
    bool hasSecondary() const { return secondaryKind != AutoPubMsgKind::Unknown; }
};

// ---------------------------------------------------------------------------
// Internal helpers — exposed inline so everything stays header-only.
// ---------------------------------------------------------------------------

inline bool AutoPubClassMap_hasClas(
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
inline AutoPubClassMapping AutoPubClassMap_lookup(
        const char* const* clasArray, size_t clasCount,
        const char* deviceTypeName)
{
    AutoPubClassMapping out;

    // -----------------------------------------------------------------
    // 1. Device-type-name overrides
    // -----------------------------------------------------------------
    if (deviceTypeName)
    {
        if (std::strcmp(deviceTypeName, "MCP9808") == 0)
        {
            // JSON mis-tag workaround: MCP9808 is a temperature sensor but
            // currently carries clas=["LGHT"].  Force TEMP mapping regardless.
            out.primaryKind = AutoPubMsgKind::Temperature;
            out.primaryTopicSlug = "temperature";
            return out;
        }
        if (std::strcmp(deviceTypeName, "RoboticalLightSensor") == 0)
        {
            // 4-attribute light sensor: left/center/right.  Use MultiArray
            // rather than Illuminance so all three channels are carried.
            out.primaryKind = AutoPubMsgKind::Float32MultiArray;
            out.primaryTopicSlug = "light";
            return out;
        }
    }

    // -----------------------------------------------------------------
    // 2. Actuator classes → excluded (no poll data to publish).
    // -----------------------------------------------------------------
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "SRVO") ||
        AutoPubClassMap_hasClas(clasArray, clasCount, "PUMP") ||
        AutoPubClassMap_hasClas(clasArray, clasCount, "PIX"))
    {
        out.excluded = true;
        return out;
    }

    // -----------------------------------------------------------------
    // 3. Composite rules (first match wins)
    // -----------------------------------------------------------------
    const bool hasAcc  = AutoPubClassMap_hasClas(clasArray, clasCount, "ACC");
    const bool hasGyro = AutoPubClassMap_hasClas(clasArray, clasCount, "GYRO");
    const bool hasTemp = AutoPubClassMap_hasClas(clasArray, clasCount, "TEMP");
    const bool hasRH   = AutoPubClassMap_hasClas(clasArray, clasCount, "RH");
    const bool hasPres = AutoPubClassMap_hasClas(clasArray, clasCount, "PRES");

    if (hasAcc && hasGyro)
    {
        out.primaryKind = AutoPubMsgKind::Imu;
        out.primaryTopicSlug = "imu";
        return out;
    }
    if (hasTemp && hasRH)
    {
        out.primaryKind   = AutoPubMsgKind::Temperature;
        out.primaryTopicSlug   = "temperature";
        out.secondaryKind = AutoPubMsgKind::RelativeHumidity;
        out.secondaryTopicSlug = "humidity";
        return out;
    }
    if (hasPres && hasTemp)
    {
        out.primaryKind   = AutoPubMsgKind::FluidPressure;
        out.primaryTopicSlug   = "pressure";
        out.secondaryKind = AutoPubMsgKind::Temperature;
        out.secondaryTopicSlug = "temperature";
        return out;
    }

    // -----------------------------------------------------------------
    // 4. Single-class rules
    // -----------------------------------------------------------------
    if (hasTemp) { out.primaryKind = AutoPubMsgKind::Temperature;      out.primaryTopicSlug = "temperature"; return out; }
    if (hasRH)   { out.primaryKind = AutoPubMsgKind::RelativeHumidity; out.primaryTopicSlug = "humidity";    return out; }
    if (hasPres) { out.primaryKind = AutoPubMsgKind::FluidPressure;    out.primaryTopicSlug = "pressure";    return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "LGHT"))
        { out.primaryKind = AutoPubMsgKind::Illuminance; out.primaryTopicSlug = "illuminance"; return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "PROX"))
        { out.primaryKind = AutoPubMsgKind::Range;       out.primaryTopicSlug = "proximity";   return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "DIST"))
        { out.primaryKind = AutoPubMsgKind::Range;       out.primaryTopicSlug = "range";       return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "ANG"))
        { out.primaryKind = AutoPubMsgKind::Float32;     out.primaryTopicSlug = "angle";       return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "ROT"))
        { out.primaryKind = AutoPubMsgKind::Int32;       out.primaryTopicSlug = "encoder";     return out; }
    if (hasAcc)
        { out.primaryKind = AutoPubMsgKind::Accel;       out.primaryTopicSlug = "accel";       return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "TCH"))
        { out.primaryKind = AutoPubMsgKind::ByteMultiArray; out.primaryTopicSlug = "touch";    return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "BTN"))
        { out.primaryKind = AutoPubMsgKind::Bool;        out.primaryTopicSlug = "button";      return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "FRCE"))
        { out.primaryKind = AutoPubMsgKind::Wrench;      out.primaryTopicSlug = "force";       return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "HRM"))
        { out.primaryKind = AutoPubMsgKind::Float32MultiArray; out.primaryTopicSlug = "ppg";   return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "SOIL"))
        { out.primaryKind = AutoPubMsgKind::Float32;     out.primaryTopicSlug = "soil_moisture"; return out; }
    if (AutoPubClassMap_hasClas(clasArray, clasCount, "GAME"))
        { out.primaryKind = AutoPubMsgKind::Joy;         out.primaryTopicSlug = "joy";         return out; }

    // -----------------------------------------------------------------
    // 5. Fallback — unknown clas, or `BTHM` (pending Slice 4.10 split).
    // -----------------------------------------------------------------
    out.primaryKind = AutoPubMsgKind::String;
    out.primaryTopicSlug = "raw";
    return out;
}

} // namespace AutoPub
} // namespace RaftRuntime
