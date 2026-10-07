/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubQoSProfile — per-writer QoS profile resolver
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
namespace AutoPub {

/// @brief Named built-in QoS profiles.  Numeric value is stable so it can be
///        stored in a single byte on the lifecycle entry.
enum class AutoPubQoSProfileId : uint8_t
{
    FastSensor     = 0,
    SlowSensor     = 1,
    Event          = 2,
    FallbackString = 3,
};

// Semantic QoS values (numerically equal to the RTPS wire kinds, which the
// RTPS backend passes straight through; a non-RTPS backend maps them) — kept local so this header has no deps.
static constexpr int32_t AUTOPUB_RELIABILITY_BEST_EFFORT = 1;
static constexpr int32_t AUTOPUB_RELIABILITY_RELIABLE    = 2;
static constexpr int32_t AUTOPUB_DURABILITY_VOLATILE          = 0;
static constexpr int32_t AUTOPUB_DURABILITY_TRANSIENT_LOCAL   = 1;

/// @brief Resolved QoS profile ready to hand to the SEDP announce / writer.
struct AutoPubQoSProfile
{
    AutoPubQoSProfileId id           = AutoPubQoSProfileId::FallbackString;
    int32_t                 reliability  = AUTOPUB_RELIABILITY_RELIABLE;
    int32_t                 durability   = AUTOPUB_DURABILITY_VOLATILE;
    uint16_t                historyDepth = 10;
};

/// @brief Returns the built-in QoS profile for the given profile id.
inline AutoPubQoSProfile AutoPubQoSProfile_get(AutoPubQoSProfileId id)
{
    AutoPubQoSProfile out;
    out.id = id;
    switch (id)
    {
        case AutoPubQoSProfileId::FastSensor:
            out.reliability  = AUTOPUB_RELIABILITY_BEST_EFFORT;
            out.durability   = AUTOPUB_DURABILITY_VOLATILE;
            out.historyDepth = 10;
            break;
        case AutoPubQoSProfileId::SlowSensor:
            out.reliability  = AUTOPUB_RELIABILITY_RELIABLE;
            out.durability   = AUTOPUB_DURABILITY_VOLATILE;
            out.historyDepth = 5;
            break;
        case AutoPubQoSProfileId::Event:
            out.reliability  = AUTOPUB_RELIABILITY_RELIABLE;
            out.durability   = AUTOPUB_DURABILITY_TRANSIENT_LOCAL;
            out.historyDepth = 20;
            break;
        case AutoPubQoSProfileId::FallbackString:
        default:
            out.id           = AutoPubQoSProfileId::FallbackString;
            out.reliability  = AUTOPUB_RELIABILITY_RELIABLE;
            out.durability   = AUTOPUB_DURABILITY_VOLATILE;
            out.historyDepth = 10;
            break;
    }
    return out;
}

/// @brief Returns the static profile name string for an id (for logging / SysTypes).
inline const char* AutoPubQoSProfile_name(AutoPubQoSProfileId id)
{
    switch (id)
    {
        case AutoPubQoSProfileId::FastSensor:     return "fast_sensor";
        case AutoPubQoSProfileId::SlowSensor:     return "slow_sensor";
        case AutoPubQoSProfileId::Event:          return "event";
        case AutoPubQoSProfileId::FallbackString: return "fallback_string";
    }
    return "fallback_string";
}

/// @brief Parse a profile name string (case-sensitive) into a profile id.
///        Returns `true` on match; `outId` is untouched on mismatch.
inline bool AutoPubQoSProfile_parseName(const char* name, AutoPubQoSProfileId& outId)
{
    if (!name || !*name)
        return false;
    if (std::strcmp(name, "fast_sensor")     == 0) { outId = AutoPubQoSProfileId::FastSensor;     return true; }
    if (std::strcmp(name, "slow_sensor")     == 0) { outId = AutoPubQoSProfileId::SlowSensor;     return true; }
    if (std::strcmp(name, "event")           == 0) { outId = AutoPubQoSProfileId::Event;          return true; }
    if (std::strcmp(name, "fallback_string") == 0) { outId = AutoPubQoSProfileId::FallbackString; return true; }
    return false;
}

/// @brief Returns the built-in default profile id for a class code (e.g. "ACC").
///        Unknown / null class → FallbackString.
inline AutoPubQoSProfileId AutoPubQoSProfile_defaultForClass(const char* clasCode)
{
    if (!clasCode || !*clasCode)
        return AutoPubQoSProfileId::FallbackString;

    // Fast-sensor classes (high-rate motion / ranging / force).
    if (std::strcmp(clasCode, "ACC")  == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "GYRO") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "PROX") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "LGHT") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "DIST") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "ANG")  == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "HRM")  == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "FRCE") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "MAG")  == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "ANGL") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "SRVO") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "MOTR") == 0) return AutoPubQoSProfileId::FastSensor;
    if (std::strcmp(clasCode, "PUMP") == 0) return AutoPubQoSProfileId::FastSensor;

    // Slow environmental sensors.
    if (std::strcmp(clasCode, "TEMP") == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "RH")   == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "PRES") == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "SOIL") == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "BTHM") == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "CO2")  == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "O2")   == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "VOC")  == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "FUEL") == 0) return AutoPubQoSProfileId::SlowSensor;
    if (std::strcmp(clasCode, "BATT") == 0) return AutoPubQoSProfileId::SlowSensor;

    // Event-like inputs (latched state, want last-value latch for late-joiners).
    if (std::strcmp(clasCode, "BTN")  == 0) return AutoPubQoSProfileId::Event;
    if (std::strcmp(clasCode, "TCH")  == 0) return AutoPubQoSProfileId::Event;
    if (std::strcmp(clasCode, "ROT")  == 0) return AutoPubQoSProfileId::Event;
    if (std::strcmp(clasCode, "GAME") == 0) return AutoPubQoSProfileId::Event;

    return AutoPubQoSProfileId::FallbackString;
}

/// @brief Resolve a default profile id from an array of class codes + a
///        device-type-name override hint.  Priority order mirrors the
///        class-map precedence: composite (ACC+GYRO = fast), then the first
///        class whose default is non-fallback, then FallbackString.
inline AutoPubQoSProfileId AutoPubQoSProfile_defaultForClasses(
        const char* const* clasArray, size_t clasCount,
        const char* /*deviceTypeName*/ = nullptr)
{
    if (!clasArray || clasCount == 0)
        return AutoPubQoSProfileId::FallbackString;

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
        return AutoPubQoSProfileId::FastSensor;

    // First class that maps to something other than FallbackString wins.
    for (size_t i = 0; i < clasCount; i++)
    {
        const AutoPubQoSProfileId id =
            AutoPubQoSProfile_defaultForClass(clasArray[i]);
        if (id != AutoPubQoSProfileId::FallbackString)
            return id;
    }
    return AutoPubQoSProfileId::FallbackString;
}

} // namespace AutoPub
} // namespace RaftRuntime
