/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubQoSProfile — per-writer QoS profile resolver
//
// Phase 4 / Slice 4.11.  Pure, header-only, no RaftCore / ESP-IDF dependencies
// so the resolver logic can be unit-tested in the Linux host harness.
//
// Profile table (design doc §7.2):
//   fast_sensor     : BEST_EFFORT, VOLATILE,         KEEP_LAST depth=10
//                     default for: ACC, GYRO, {ACC,GYRO}, PROX, LGHT, DIST,
//                                   ANG, HRM, FRCE
//   slow_sensor     : RELIABLE,    VOLATILE,         KEEP_LAST depth=5
//                     default for: TEMP, RH, PRES, SOIL, BTHM
//   event           : RELIABLE,    TRANSIENT_LOCAL,  KEEP_LAST depth=20
//                     default for: BTN, TCH, ROT, GAME
//   fallback_string : RELIABLE,    VOLATILE,         KEEP_LAST depth=10
//                     default for any unmapped class
//
// Resolution order (caller side):
//   1. per-device alias override (SysTypes `qosProfiles.<alias>`)
//   2. per-class override        (SysTypes `qosProfiles.classDefaults.<CLAS>`)
//   3. built-in default from the table above
//
// Integer QoS values mirror `RTPSTypes.h` (duplicated here to avoid pulling
// the heavier RTPS core into this leaf header).  `static_assert` lines in
// RaftROS.cpp lock the values to stay in sync.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>

namespace RaftRuntime {
namespace RTPS {
namespace Runtime {
namespace AutoPub {

/// @brief Named built-in QoS profiles.  Numeric value is stable so it can be
///        stored in a single byte on the lifecycle entry.
enum class RTPSAutoPubQoSProfileId : uint8_t
{
    FastSensor     = 0,
    SlowSensor     = 1,
    Event          = 2,
    FallbackString = 3,
};

// Mirrors of RTPSTypes values — kept local so this header has no deps.
static constexpr int32_t RTPS_AUTOPUB_RELIABILITY_BEST_EFFORT = 1;
static constexpr int32_t RTPS_AUTOPUB_RELIABILITY_RELIABLE    = 2;
static constexpr int32_t RTPS_AUTOPUB_DURABILITY_VOLATILE          = 0;
static constexpr int32_t RTPS_AUTOPUB_DURABILITY_TRANSIENT_LOCAL   = 1;

/// @brief Resolved QoS profile ready to hand to the SEDP announce / writer.
struct RTPSAutoPubQoSProfile
{
    RTPSAutoPubQoSProfileId id           = RTPSAutoPubQoSProfileId::FallbackString;
    int32_t                 reliability  = RTPS_AUTOPUB_RELIABILITY_RELIABLE;
    int32_t                 durability   = RTPS_AUTOPUB_DURABILITY_VOLATILE;
    uint16_t                historyDepth = 10;
};

/// @brief Returns the built-in QoS profile for the given profile id.
inline RTPSAutoPubQoSProfile RTPSAutoPubQoSProfile_get(RTPSAutoPubQoSProfileId id)
{
    RTPSAutoPubQoSProfile out;
    out.id = id;
    switch (id)
    {
        case RTPSAutoPubQoSProfileId::FastSensor:
            out.reliability  = RTPS_AUTOPUB_RELIABILITY_BEST_EFFORT;
            out.durability   = RTPS_AUTOPUB_DURABILITY_VOLATILE;
            out.historyDepth = 10;
            break;
        case RTPSAutoPubQoSProfileId::SlowSensor:
            out.reliability  = RTPS_AUTOPUB_RELIABILITY_RELIABLE;
            out.durability   = RTPS_AUTOPUB_DURABILITY_VOLATILE;
            out.historyDepth = 5;
            break;
        case RTPSAutoPubQoSProfileId::Event:
            out.reliability  = RTPS_AUTOPUB_RELIABILITY_RELIABLE;
            out.durability   = RTPS_AUTOPUB_DURABILITY_TRANSIENT_LOCAL;
            out.historyDepth = 20;
            break;
        case RTPSAutoPubQoSProfileId::FallbackString:
        default:
            out.id           = RTPSAutoPubQoSProfileId::FallbackString;
            out.reliability  = RTPS_AUTOPUB_RELIABILITY_RELIABLE;
            out.durability   = RTPS_AUTOPUB_DURABILITY_VOLATILE;
            out.historyDepth = 10;
            break;
    }
    return out;
}

/// @brief Returns the static profile name string for an id (for logging / SysTypes).
inline const char* RTPSAutoPubQoSProfile_name(RTPSAutoPubQoSProfileId id)
{
    switch (id)
    {
        case RTPSAutoPubQoSProfileId::FastSensor:     return "fast_sensor";
        case RTPSAutoPubQoSProfileId::SlowSensor:     return "slow_sensor";
        case RTPSAutoPubQoSProfileId::Event:          return "event";
        case RTPSAutoPubQoSProfileId::FallbackString: return "fallback_string";
    }
    return "fallback_string";
}

/// @brief Parse a profile name string (case-sensitive) into a profile id.
///        Returns `true` on match; `outId` is untouched on mismatch.
inline bool RTPSAutoPubQoSProfile_parseName(const char* name, RTPSAutoPubQoSProfileId& outId)
{
    if (!name || !*name)
        return false;
    if (std::strcmp(name, "fast_sensor")     == 0) { outId = RTPSAutoPubQoSProfileId::FastSensor;     return true; }
    if (std::strcmp(name, "slow_sensor")     == 0) { outId = RTPSAutoPubQoSProfileId::SlowSensor;     return true; }
    if (std::strcmp(name, "event")           == 0) { outId = RTPSAutoPubQoSProfileId::Event;          return true; }
    if (std::strcmp(name, "fallback_string") == 0) { outId = RTPSAutoPubQoSProfileId::FallbackString; return true; }
    return false;
}

/// @brief Returns the built-in default profile id for a class code (e.g. "ACC").
///        Unknown / null class → FallbackString.
inline RTPSAutoPubQoSProfileId RTPSAutoPubQoSProfile_defaultForClass(const char* clasCode)
{
    if (!clasCode || !*clasCode)
        return RTPSAutoPubQoSProfileId::FallbackString;

    // Fast-sensor classes (high-rate motion / ranging / force).
    if (std::strcmp(clasCode, "ACC")  == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "GYRO") == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "PROX") == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "LGHT") == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "DIST") == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "ANG")  == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "HRM")  == 0) return RTPSAutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "FRCE") == 0) return RTPSAutoPubQoSProfileId::FastSensor;

    // Slow environmental sensors.
    if (std::strcmp(clasCode, "TEMP") == 0) return RTPSAutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "RH")   == 0) return RTPSAutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "PRES") == 0) return RTPSAutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "SOIL") == 0) return RTPSAutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "BTHM") == 0) return RTPSAutoPubQoSProfileId::SlowSensor;

    // Event-like inputs (latched state, want last-value latch for late-joiners).
    if (std::strcmp(clasCode, "BTN")  == 0) return RTPSAutoPubQoSProfileId::Event;
    if (std::strcmp(clasCode, "TCH")  == 0) return RTPSAutoPubQoSProfileId::Event;
    if (std::strcmp(clasCode, "ROT")  == 0) return RTPSAutoPubQoSProfileId::Event;
    if (std::strcmp(clasCode, "GAME") == 0) return RTPSAutoPubQoSProfileId::Event;

    return RTPSAutoPubQoSProfileId::FallbackString;
}

/// @brief Resolve a default profile id from an array of class codes + a
///        device-type-name override hint.  Priority order mirrors the
///        class-map precedence: composite (ACC+GYRO = fast), then the first
///        class whose default is non-fallback, then FallbackString.
inline RTPSAutoPubQoSProfileId RTPSAutoPubQoSProfile_defaultForClasses(
        const char* const* clasArray, size_t clasCount,
        const char* /*deviceTypeName*/ = nullptr)
{
    if (!clasArray || clasCount == 0)
        return RTPSAutoPubQoSProfileId::FallbackString;

    // Composite IMU promotes to fast_sensor regardless of order.
    bool hasAcc = false, hasGyro = false;
    for (size_t i = 0; i < clasCount; i++)
    {
        if (!clasArray[i])
            continue;
        if (std::strcmp(clasArray[i], "ACC")  == 0) hasAcc  = true;
        if (std::strcmp(clasArray[i], "GYRO") == 0) hasGyro = true;
    }
    if (hasAcc && hasGyro)
        return RTPSAutoPubQoSProfileId::FastSensor;

    // First class that maps to something other than FallbackString wins.
    for (size_t i = 0; i < clasCount; i++)
    {
        const RTPSAutoPubQoSProfileId id =
            RTPSAutoPubQoSProfile_defaultForClass(clasArray[i]);
        if (id != RTPSAutoPubQoSProfileId::FallbackString)
            return id;
    }
    return RTPSAutoPubQoSProfileId::FallbackString;
}

} // namespace AutoPub
} // namespace Runtime
} // namespace RTPS
} // namespace RaftRuntime
