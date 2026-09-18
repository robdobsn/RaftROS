/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubCDRSerializer — XCDR1 serialisers per AutoPubMsgKind
//
// Phase 4 / Slice 4.5
//
// Serialises a decoded poll struct (described by an `AttrFieldDesc[]` table)
// into a ROS 2 XCDR1 message body for a given `AutoPubMsgKind`.
//
// The shared-runtime autopub path deliberately avoids depending on
// `DeviceTypeRecord.h` (which pulls RaftArduino / RaftBus into the
// Linux unit-test harness).  Instead we define local mirrors of the two
// types we actually read — `AutoPubAttrType` and
// `AutoPubAttrFieldDesc` — with identical binary layouts.  The
// wrapper (RaftROS.cpp) casts `const AttrFieldDesc*` to
// `const AutoPubAttrFieldDesc*` and is responsible for a
// `static_assert` that the layouts still match.
//
// Unit-scaling adjustments required by ROS 2 message semantics
// (e.g. g → m/s², °/s → rad/s, mm → m, hPa → Pa) are applied inside the
// serialisers on top of the per-field `divisor`/`addend` already baked
// into the AttrFieldDesc table by the JSON code generator.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstddef>
#include "AutoPubClassMap.h"

namespace RaftRuntime {
namespace AutoPub {

/// @brief Mirror of `AttrType` in RaftCore/DeviceTypeRecord.h.
/// Numeric values MUST match.
enum class AutoPubAttrType : uint8_t
{
    Float  = 0,
    Int32  = 1,
    Uint32 = 2,
    Int16  = 3,
    Uint16 = 4,
    Int8   = 5,
    Uint8  = 6,
    Bool   = 7,
};

/// @brief Mirror of `AttrFieldDesc` in RaftCore/DeviceTypeRecord.h.
/// Binary layout MUST match (checked via static_assert in RaftROS.cpp).
struct AutoPubAttrFieldDesc
{
    const char*          name;
    uint16_t             offset;
    AutoPubAttrType  type;
    const char*          fmtStr;
    float                divisor;
    float                addend;
};

/// @brief Serialisation context: everything needed to build one CDR message.
struct AutoPubCDRContext
{
    const AutoPubAttrFieldDesc* pFieldDescs = nullptr;
    uint16_t                         fieldCount = 0;
    const uint8_t*                   pStruct    = nullptr;
    uint32_t                         structSize = 0;
    uint32_t                         timestampMs = 0;   ///< Header stamp (boot ms OK for Humble).
    const char*                      frameId    = "raft";
};

/// @brief Serialise a single ROS 2 message of the given kind into `pOutBuf`.
/// @return true if the complete message fit; `outBytesWritten` holds payload length.
bool AutoPubCDRSerializer_serialize(
        AutoPubMsgKind kind,
        const AutoPubCDRContext& ctx,
        uint8_t* pOutBuf, uint32_t outBufLen,
        uint32_t& outBytesWritten);

/// @brief Serialise a `std_msgs/String` with an explicit body (used for the
///        Slice 4.9 fallback path — serialise arbitrary JSON text).
/// @return true on success; `outBytesWritten` is the payload length.
bool AutoPubCDRSerializer_serializeString(
        const char* pText,
        uint8_t* pOutBuf, uint32_t outBufLen,
        uint32_t& outBytesWritten);

// ---------------------------------------------------------------------------
// Internal helpers — exposed in the header so unit tests can exercise them
// without linking the dispatch entry point.
// ---------------------------------------------------------------------------

/// @brief Read a named field from the decoded struct as a `double`, applying
///        the AttrFieldDesc divisor+addend if `applyScale` is true.
/// @return true if the field was found and successfully read.
bool AutoPubCDRSerializer_readFieldDouble(
        const AutoPubCDRContext& ctx,
        const char* name,
        double& out,
        bool applyScale = true);

/// @brief Read a named field as an `int32_t` (truncation rounds toward zero).
bool AutoPubCDRSerializer_readFieldInt32(
        const AutoPubCDRContext& ctx,
        const char* name,
        int32_t& out,
        bool applyScale = true);

/// @brief Read a named field as a `bool` (non-zero is true).
bool AutoPubCDRSerializer_readFieldBool(
        const AutoPubCDRContext& ctx,
        const char* name,
        bool& out);

} // namespace AutoPub
} // namespace RaftRuntime
