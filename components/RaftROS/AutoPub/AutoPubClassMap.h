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

/// @brief ROS 2 type description hash (REP-2011 "RIHS01_...") for a message
/// kind.  Zenoh topic keys embed this hash, so a wrong value silently stops
/// subscribers matching.  Values are the `type_hashes` entries from the
/// rosidl-generated type descriptions shipped with ROS 2 Jazzy
/// (`/opt/ros/jazzy/share/<pkg>/msg/<Type>.json`), which are the same hashes
/// rmw_zenoh puts on the wire.
/// @return Static literal, or nullptr for Unknown.
inline const char* AutoPubClassMap_typeHash(AutoPubMsgKind kind)
{
    switch (kind)
    {
        case AutoPubMsgKind::Imu:
        case AutoPubMsgKind::Accel:
            return "RIHS01_7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b";
        case AutoPubMsgKind::Temperature:
            return "RIHS01_72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8";
        case AutoPubMsgKind::RelativeHumidity:
            return "RIHS01_8687c99b4fb393cb2e545e407b5ea7fd0b5d8960bcd849a0f86c544740138839";
        case AutoPubMsgKind::FluidPressure:
            return "RIHS01_22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32";
        case AutoPubMsgKind::Illuminance:
            return "RIHS01_b954b25f452fcf81a91c9c2a7e3b3fd85c4c873d452aecb3cfd8fd1da732a22d";
        case AutoPubMsgKind::Range:
            return "RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a";
        case AutoPubMsgKind::Float32:
            return "RIHS01_7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2";
        case AutoPubMsgKind::Int32:
            return "RIHS01_b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb";
        case AutoPubMsgKind::Bool:
            return "RIHS01_feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9";
        case AutoPubMsgKind::ByteMultiArray:
            return "RIHS01_972fec7f50ab3c1d06783c228e79e8a9a509021708c511c059926261ada901d4";
        case AutoPubMsgKind::Wrench:
            return "RIHS01_018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52";
        case AutoPubMsgKind::Float32MultiArray:
            return "RIHS01_0599f6f85b4bfca379873a0b4375a0aca022156bd2d7021275d116ed1fa8bfe0";
        case AutoPubMsgKind::Joy:
            return "RIHS01_0d356c79cad3401e35ffeb75a96a96e08be3ef896b8b83841d73e890989372c5";
        case AutoPubMsgKind::String:
            return "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18";
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
