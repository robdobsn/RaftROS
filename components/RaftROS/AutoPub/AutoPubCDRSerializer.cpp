/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubCDRSerializer — XCDR1 serialisers per AutoPubMsgKind
//
// See AutoPubCDRSerializer.h for API and design notes.
//
// All serialisers emit little-endian XCDR1 with the standard 4-byte
// encapsulation header (CDR_LE).  Alignment is computed from the start of the
// encapsulation, matching the existing CDREncoder behaviour that has already
// been validated against ROS 2 Humble / FastDDS in Phase 3.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "AutoPubCDRSerializer.h"
#include "../CDR/CDREncoder.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace RaftRuntime {
namespace AutoPub {

// ---------------------------------------------------------------------------
// Physical-unit scaling constants (ROS 2 REP-103 compliance, on top of any
// per-field divisor/addend applied by the decoder-generated AttrFieldDesc).
// ---------------------------------------------------------------------------
static constexpr double GRAVITY_MPS2         = 9.80665;     ///< ACC: g → m/s²
static constexpr double DEG_TO_RAD           = 0.017453292519943295; ///< GYRO/ANG: °/s → rad/s, ° → rad
static constexpr double MM_TO_M              = 0.001;       ///< DIST: mm → m
static constexpr double HPA_TO_PA            = 100.0;       ///< PRES: hPa → Pa
static constexpr double PERCENT_TO_UNIT      = 0.01;        ///< RH: % → 0..1
static constexpr double MICROTESLA_TO_T      = 1e-6;        ///< MAG: µT → T

// Range (Humble) radiation_type constants.
static constexpr uint8_t RANGE_RADIATION_INFRARED = 1;

// Default fields-of-view used by the Range serialiser (unknown → 0).
static constexpr float   RANGE_DEFAULT_FOV        = 0.0f;
static constexpr float   RANGE_MIN_DEFAULT        = 0.0f;
static constexpr float   RANGE_MAX_PROX_DEFAULT   = 1.0f;       // normalised proximity
static constexpr float   RANGE_MAX_DIST_DEFAULT   = 2.0f;       // ~2 m for VL6180 / VL53L4CD
static constexpr float   RANGE_MAX_MULTIZONE      = 4.0f;       // VL53L5CX / L7CX / L8CX

// ---------------------------------------------------------------------------
// Field lookup helpers
// ---------------------------------------------------------------------------

static const AutoPubAttrFieldDesc* findField(
        const AutoPubCDRContext& ctx, const char* name)
{
    if (!ctx.pFieldDescs || !name)
        return nullptr;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const AutoPubAttrFieldDesc& d = ctx.pFieldDescs[i];
        if (d.name && std::strcmp(d.name, name) == 0)
            return &d;
    }
    return nullptr;
}

/// Copy `n` bytes from the struct at the given offset, respecting
/// the buffer bound.  Returns false if out-of-range.
static bool readBytesFromStruct(const AutoPubCDRContext& ctx,
                                uint16_t offset, uint32_t n, void* pOut)
{
    if (!ctx.pStruct)
        return false;
    if ((uint32_t)offset + n > ctx.structSize)
        return false;
    std::memcpy(pOut, ctx.pStruct + offset, n);
    return true;
}

static uint32_t attrTypeSize(AutoPubAttrType type)
{
    switch (type)
    {
        case AutoPubAttrType::Float:
        case AutoPubAttrType::Int32:
        case AutoPubAttrType::Uint32: return 4;
        case AutoPubAttrType::Int16:
        case AutoPubAttrType::Uint16: return 2;
        case AutoPubAttrType::Int8:
        case AutoPubAttrType::Uint8:
        case AutoPubAttrType::Bool:   return 1;
        default:                      return 0;
    }
}

bool AutoPubCDRSerializer_readFieldElementDouble(
        const AutoPubCDRContext& ctx, const AutoPubAttrFieldDesc& desc,
        uint16_t index, double& out, bool applyScale)
{
    const uint32_t elemSize = attrTypeSize(desc.type);
    const uint16_t count = desc.count ? desc.count : 1;
    if (elemSize == 0 || index >= count)
        return false;
    const uint32_t offset = (uint32_t)desc.offset + (uint32_t)index * elemSize;
    if (offset > 0xFFFF)
        return false;

    double raw = 0.0;
    switch (desc.type)
    {
        case AutoPubAttrType::Float:
        {
            float v = 0.0f;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 4, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Int32:
        {
            int32_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 4, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Uint32:
        {
            uint32_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 4, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Int16:
        {
            int16_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 2, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Uint16:
        {
            uint16_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 2, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Int8:
        {
            int8_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 1, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Uint8:
        {
            uint8_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 1, &v)) return false;
            raw = (double)v;
            break;
        }
        case AutoPubAttrType::Bool:
        {
            uint8_t v = 0;
            if (!readBytesFromStruct(ctx, (uint16_t)offset, 1, &v)) return false;
            raw = v ? 1.0 : 0.0;
            break;
        }
        default:
            return false;
    }

    if (applyScale)
    {
        if (desc.divisor != 0.0f && desc.divisor != 1.0f)
            raw /= (double)desc.divisor;
        if (desc.addend != 0.0f)
            raw += (double)desc.addend;
    }
    out = raw;
    return true;
}

bool AutoPubCDRSerializer_readFieldDouble(
        const AutoPubCDRContext& ctx, const char* name,
        double& out, bool applyScale)
{
    const AutoPubAttrFieldDesc* pDesc = findField(ctx, name);
    if (!pDesc)
        return false;
    return AutoPubCDRSerializer_readFieldElementDouble(ctx, *pDesc, 0, out, applyScale);
}

bool AutoPubCDRSerializer_readFieldInt32(
        const AutoPubCDRContext& ctx, const char* name,
        int32_t& out, bool applyScale)
{
    double v = 0.0;
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, name, v, applyScale))
        return false;
    // Truncate toward zero (standard C cast semantics).
    if (v >  (double)INT32_MAX) { out = INT32_MAX; return true; }
    if (v < -(double)INT32_MAX) { out = INT32_MIN; return true; }
    out = (int32_t)v;
    return true;
}

bool AutoPubCDRSerializer_readFieldBool(
        const AutoPubCDRContext& ctx, const char* name, bool& out)
{
    const AutoPubAttrFieldDesc* pDesc = findField(ctx, name);
    if (!pDesc)
        return false;
    if (pDesc->type == AutoPubAttrType::Bool ||
        pDesc->type == AutoPubAttrType::Uint8 ||
        pDesc->type == AutoPubAttrType::Int8)
    {
        uint8_t v = 0;
        if (!readBytesFromStruct(ctx, pDesc->offset, 1, &v))
            return false;
        out = (v != 0);
        return true;
    }
    double v = 0.0;
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, name, v, false))
        return false;
    out = (v != 0.0);
    return true;
}

// ---------------------------------------------------------------------------
// std_msgs/Header helpers
// ---------------------------------------------------------------------------

static bool writeHeader(CDREncoder& enc,
                        uint32_t timestampMs, const char* frameId)
{
    // builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
    const int32_t sec = (int32_t)(timestampMs / 1000u);
    const uint32_t nanosec = (timestampMs % 1000u) * 1000000u;
    if (!enc.writeInt32(sec)) return false;
    if (!enc.writeUint32(nanosec)) return false;
    // string frame_id
    if (!enc.writeString(frameId ? frameId : "")) return false;
    return true;
}

static bool writeCovarianceUnknown9(CDREncoder& enc)
{
    // 9-element float64 covariance with [0] = -1 → "unknown" (REP-145 marker).
    if (!enc.writeFloat64(-1.0)) return false;
    for (int i = 1; i < 9; i++)
        if (!enc.writeFloat64(0.0)) return false;
    return true;
}

static bool writeCovarianceZero9(CDREncoder& enc)
{
    for (int i = 0; i < 9; i++)
        if (!enc.writeFloat64(0.0)) return false;
    return true;
}

// ---------------------------------------------------------------------------
// Per-kind serialisers
// ---------------------------------------------------------------------------

static bool serializeTemperature(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double temperature = 0.0;
    (void)AutoPubCDRSerializer_readFieldDouble(ctx, "temperature", temperature);
    if (!enc.writeFloat64(temperature)) return false;
    if (!enc.writeFloat64(0.0)) return false; // variance unknown
    return true;
}

static bool serializeRelativeHumidity(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double humidity = 0.0;
    (void)AutoPubCDRSerializer_readFieldDouble(ctx, "humidity", humidity);
    humidity *= PERCENT_TO_UNIT; // % → 0..1
    if (!enc.writeFloat64(humidity)) return false;
    if (!enc.writeFloat64(0.0)) return false;
    return true;
}

static bool serializeFluidPressure(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double pressure = 0.0;
    (void)AutoPubCDRSerializer_readFieldDouble(ctx, "pressure", pressure);
    pressure *= HPA_TO_PA;
    if (!enc.writeFloat64(pressure)) return false;
    if (!enc.writeFloat64(0.0)) return false;
    return true;
}

static bool serializeIlluminance(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double lux = 0.0;
    // Prefer `als` (VCNL/VEML) then `illuminance`.
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, "als", lux))
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "illuminance", lux);
    if (!enc.writeFloat64(lux)) return false;
    if (!enc.writeFloat64(0.0)) return false;
    return true;
}

/// @brief Is zone `index` of a multizone frame a valid reading?  The status
/// array, when the record has one, uses the ST target-status codes.
static bool multizoneValid(const AutoPubCDRContext& ctx, uint16_t index)
{
    const AutoPubAttrFieldDesc* pStatus = findField(ctx, "status");
    if (!pStatus || (pStatus->count ? pStatus->count : 1) <= index)
        return true;
    double status = 0.0;
    if (!AutoPubCDRSerializer_readFieldElementDouble(ctx, *pStatus, index, status, false))
        return true;
    return status == 5.0 || status == 6.0 || status == 9.0;
}

/// @brief Zones in the frame: the `grid` field (4 or 8 a side) when present,
/// else the whole distance array
static uint16_t multizoneZones(const AutoPubCDRContext& ctx, const AutoPubAttrFieldDesc& dist)
{
    const uint16_t count = dist.count ? dist.count : 1;
    double grid = 0.0;
    if (AutoPubCDRSerializer_readFieldDouble(ctx, "grid", grid, false) && grid >= 1.0 && grid * grid <= count)
        return (uint16_t)(grid * grid);
    return count;
}

/// @brief Nearest valid zone's distance in metres, or +inf with none valid
static double multizoneNearestM(const AutoPubCDRContext& ctx, const AutoPubAttrFieldDesc& dist)
{
    double nearestMm = std::numeric_limits<double>::infinity();
    const uint16_t zones = multizoneZones(ctx, dist);
    for (uint16_t zone = 0; zone < zones; zone++)
    {
        double mm = 0.0;
        if (!multizoneValid(ctx, zone) || !AutoPubCDRSerializer_readFieldElementDouble(ctx, dist, zone, mm))
            continue;
        if (mm < nearestMm)
            nearestMm = mm;
    }
    return std::isinf(nearestMm) ? nearestMm : nearestMm * MM_TO_M;
}

static bool serializeRange(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    if (!enc.writeUint8(RANGE_RADIATION_INFRARED)) return false;
    if (!enc.writeFloat32(RANGE_DEFAULT_FOV)) return false;
    if (!enc.writeFloat32(RANGE_MIN_DEFAULT)) return false;

    double rangeVal = 0.0;
    float  maxRange = RANGE_MAX_DIST_DEFAULT;
    const AutoPubAttrFieldDesc* pDist = findField(ctx, "dist");
    if (pDist && (pDist->count ? pDist->count : 1) > 1)
    {
        // A multizone frame: the nearest zone with a valid status (ST: 5 =
        // valid, 6 and 9 = valid with lower confidence; no status array =
        // every zone counts).  No valid zone gives +inf, per REP-117.
        rangeVal = multizoneNearestM(ctx, *pDist);
        maxRange = RANGE_MAX_MULTIZONE;
    }
    else if (AutoPubCDRSerializer_readFieldDouble(ctx, "dist", rangeVal))
    {
        rangeVal *= MM_TO_M;   // mm → m
        maxRange = RANGE_MAX_DIST_DEFAULT;
    }
    else if (AutoPubCDRSerializer_readFieldDouble(ctx, "prox", rangeVal))
    {
        // VCNL proximity count divisor already normalised via AttrFieldDesc;
        // if not divided it's in raw counts (0-65535).  Normalise to 0..1.
        if (rangeVal > 1.0) rangeVal /= 65535.0;
        maxRange = RANGE_MAX_PROX_DEFAULT;
    }
    if (!enc.writeFloat32(maxRange)) return false;
    if (!enc.writeFloat32((float)rangeVal)) return false;
    // variance (added in ROS 2 Jazzy); 0.0 = variance unknown per REP.
    if (!enc.writeFloat32(0.0f)) return false;
    return true;
}

static bool serializeImu(CDREncoder& enc, const AutoPubCDRContext& ctx, bool accelOnly)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;

    // orientation (quaternion) - identity, covariance[0] = -1 (unknown).
    if (!enc.writeFloat64(0.0)) return false; // x
    if (!enc.writeFloat64(0.0)) return false; // y
    if (!enc.writeFloat64(0.0)) return false; // z
    if (!enc.writeFloat64(1.0)) return false; // w
    if (!writeCovarianceUnknown9(enc)) return false;

    // angular_velocity (rad/s).  °/s → rad/s.
    double gx = 0.0, gy = 0.0, gz = 0.0;
    if (!accelOnly)
    {
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "gx", gx);
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "gy", gy);
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "gz", gz);
        gx *= DEG_TO_RAD; gy *= DEG_TO_RAD; gz *= DEG_TO_RAD;
    }
    if (!enc.writeFloat64(gx)) return false;
    if (!enc.writeFloat64(gy)) return false;
    if (!enc.writeFloat64(gz)) return false;
    if (accelOnly)
    {
        if (!writeCovarianceUnknown9(enc)) return false;
    }
    else
    {
        if (!writeCovarianceZero9(enc)) return false;
    }

    // linear_acceleration (m/s²).  g → m/s².  Accept x/y/z or ax/ay/az.
    double ax = 0.0, ay = 0.0, az = 0.0;
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, "ax", ax))
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "x", ax);
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, "ay", ay))
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "y", ay);
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, "az", az))
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, "z", az);
    ax *= GRAVITY_MPS2; ay *= GRAVITY_MPS2; az *= GRAVITY_MPS2;
    if (!enc.writeFloat64(ax)) return false;
    if (!enc.writeFloat64(ay)) return false;
    if (!enc.writeFloat64(az)) return false;
    if (!writeCovarianceZero9(enc)) return false;

    return true;
}

static bool serializeFloat32(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // Try common scalar names in priority order.
    double v = 0.0;
    if (!AutoPubCDRSerializer_readFieldDouble(ctx, "angle", v) &&
        !AutoPubCDRSerializer_readFieldDouble(ctx, "moisture", v) &&
        !AutoPubCDRSerializer_readFieldDouble(ctx, "value", v) &&
        !AutoPubCDRSerializer_readFieldDouble(ctx, "data", v))
    {
        // Fall back to the first non-timestamp field.
        for (uint16_t i = 0; i < ctx.fieldCount; i++)
        {
            const auto& d = ctx.pFieldDescs[i];
            if (d.name && std::strcmp(d.name, "timeMs") != 0)
            {
                (void)AutoPubCDRSerializer_readFieldDouble(ctx, d.name, v);
                break;
            }
        }
    }
    return enc.writeFloat32((float)v);
}

static bool serializeInt32(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    int32_t v = 0;
    if (!AutoPubCDRSerializer_readFieldInt32(ctx, "rotation", v) &&
        !AutoPubCDRSerializer_readFieldInt32(ctx, "count", v) &&
        !AutoPubCDRSerializer_readFieldInt32(ctx, "value", v) &&
        !AutoPubCDRSerializer_readFieldInt32(ctx, "data", v))
    {
        for (uint16_t i = 0; i < ctx.fieldCount; i++)
        {
            const auto& d = ctx.pFieldDescs[i];
            if (d.name && std::strcmp(d.name, "timeMs") != 0)
            {
                (void)AutoPubCDRSerializer_readFieldInt32(ctx, d.name, v);
                break;
            }
        }
    }
    return enc.writeInt32(v);
}

static bool serializeBool(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    bool v = false;
    if (!AutoPubCDRSerializer_readFieldBool(ctx, "press", v) &&
        !AutoPubCDRSerializer_readFieldBool(ctx, "state", v) &&
        !AutoPubCDRSerializer_readFieldBool(ctx, "value", v) &&
        !AutoPubCDRSerializer_readFieldBool(ctx, "data", v))
    {
        for (uint16_t i = 0; i < ctx.fieldCount; i++)
        {
            const auto& d = ctx.pFieldDescs[i];
            if (d.name && std::strcmp(d.name, "timeMs") != 0)
            {
                (void)AutoPubCDRSerializer_readFieldBool(ctx, d.name, v);
                break;
            }
        }
    }
    return enc.writeBool(v);
}

static bool serializeByteMultiArray(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // std_msgs/ByteMultiArray:
    //   MultiArrayLayout layout          { MultiArrayDimension[] dim; uint32 data_offset; }
    //   byte[] data (sequence<uint8>)
    // Emit an empty layout (dim[] length 0, data_offset=0) then pack every
    // non-timestamp boolean field into the byte sequence.
    if (!enc.writeSequenceLength(0)) return false; // dim[].length
    if (!enc.writeUint32(0)) return false;          // data_offset

    // Count eligible fields (bool/uint8) excluding timestamp.
    uint32_t count = 0;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        if (d.type == AutoPubAttrType::Bool ||
            d.type == AutoPubAttrType::Uint8 ||
            d.type == AutoPubAttrType::Int8)
            count++;
    }
    if (!enc.writeSequenceLength(count)) return false;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        if (d.type != AutoPubAttrType::Bool &&
            d.type != AutoPubAttrType::Uint8 &&
            d.type != AutoPubAttrType::Int8)
            continue;
        bool b = false;
        AutoPubCDRSerializer_readFieldBool(ctx, d.name, b);
        if (!enc.writeUint8(b ? 1 : 0)) return false;
    }
    return true;
}

static bool serializeFloat32MultiArray(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // MultiArrayLayout (empty) + float32 data[]
    if (!enc.writeSequenceLength(0)) return false; // dim[]
    if (!enc.writeUint32(0)) return false;          // data_offset

    uint32_t count = 0;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        // Skip status/valid flags.
        if (std::strcmp(d.name, "status") == 0) continue;
        if (std::strcmp(d.name, "valid")  == 0) continue;
        count++;
    }
    if (!enc.writeSequenceLength(count)) return false;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        if (std::strcmp(d.name, "status") == 0) continue;
        if (std::strcmp(d.name, "valid")  == 0) continue;
        double v = 0.0;
        AutoPubCDRSerializer_readFieldDouble(ctx, d.name, v);
        if (!enc.writeFloat32((float)v)) return false;
    }
    return true;
}

static bool serializeWrench(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // geometry_msgs/Wrench: Vector3 force, Vector3 torque (all float64).
    // HX711 → force.z = "force" field (N).  Valid flag gates the caller.
    double f = 0.0;
    (void)AutoPubCDRSerializer_readFieldDouble(ctx, "force", f);
    if (!enc.writeFloat64(0.0)) return false; // force.x
    if (!enc.writeFloat64(0.0)) return false; // force.y
    if (!enc.writeFloat64(f)) return false;   // force.z
    if (!enc.writeFloat64(0.0)) return false; // torque.x
    if (!enc.writeFloat64(0.0)) return false; // torque.y
    if (!enc.writeFloat64(0.0)) return false; // torque.z
    return true;
}

static bool serializeJoy(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // sensor_msgs/Joy: Header header, float32[] axes, int32[] buttons.
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;

    // Axes = fields named x/y; buttons = all other scalar fields (excl timeMs).
    const char* const axisNames[] = { "x", "y" };
    uint32_t nAxes = 0;
    for (auto an : axisNames) if (findField(ctx, an)) nAxes++;
    if (!enc.writeSequenceLength(nAxes)) return false;
    for (auto an : axisNames)
    {
        if (!findField(ctx, an)) continue;
        double v = 0.0;
        AutoPubCDRSerializer_readFieldDouble(ctx, an, v);
        if (!enc.writeFloat32((float)v)) return false;
    }

    // Buttons: every non-axis, non-timestamp field.
    uint32_t nBtn = 0;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        if (std::strcmp(d.name, "x") == 0 || std::strcmp(d.name, "y") == 0) continue;
        nBtn++;
    }
    if (!enc.writeSequenceLength(nBtn)) return false;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        if (std::strcmp(d.name, "x") == 0 || std::strcmp(d.name, "y") == 0) continue;
        int32_t v = 0;
        AutoPubCDRSerializer_readFieldInt32(ctx, d.name, v);
        if (!enc.writeInt32(v)) return false;
    }
    return true;
}

static bool serializeMagneticField(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // sensor_msgs/MagneticField: Header, Vector3 magnetic_field (T), float64[9]
    // covariance (all zero = unknown).  Device records give µT.
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    const char* const axes[] = {"x", "y", "z"};
    for (auto axis : axes)
    {
        double v = 0.0;
        (void)AutoPubCDRSerializer_readFieldDouble(ctx, axis, v);
        if (!enc.writeFloat64(v * MICROTESLA_TO_T)) return false;
    }
    return writeCovarianceZero9(enc);
}

static bool serializeBatteryState(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // sensor_msgs/BatteryState.  A fuel gauge (MAX17048) reports voltage (V),
    // charge (%) and chargeRate (%/h); everything it cannot measure is NaN,
    // as the message definition asks.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double voltage = 0.0, charge = 0.0, chargeRate = 0.0;
    const bool haveVoltage = AutoPubCDRSerializer_readFieldDouble(ctx, "voltage", voltage);
    const bool haveCharge = AutoPubCDRSerializer_readFieldDouble(ctx, "charge", charge);
    const bool haveRate = AutoPubCDRSerializer_readFieldDouble(ctx, "chargeRate", chargeRate);
    if (!enc.writeFloat32(haveVoltage ? (float)voltage : nan)) return false;   // voltage
    if (!enc.writeFloat32(nan)) return false;                                   // temperature
    if (!enc.writeFloat32(nan)) return false;                                   // current
    if (!enc.writeFloat32(nan)) return false;                                   // charge (Ah)
    if (!enc.writeFloat32(nan)) return false;                                   // capacity
    if (!enc.writeFloat32(nan)) return false;                                   // design_capacity
    if (!enc.writeFloat32(haveCharge ? (float)(charge * PERCENT_TO_UNIT) : nan)) return false;   // percentage 0..1
    // POWER_SUPPLY_STATUS_: UNKNOWN 0, CHARGING 1, DISCHARGING 2, NOT_CHARGING 3
    uint8_t status = 0;
    if (haveRate)
        status = chargeRate > 0.5 ? 1 : chargeRate < -0.5 ? 2 : 3;
    if (!enc.writeUint8(status)) return false;
    if (!enc.writeUint8(0)) return false;           // power_supply_health UNKNOWN
    if (!enc.writeUint8(0)) return false;           // power_supply_technology UNKNOWN
    if (!enc.writeBool(true)) return false;         // present
    if (!enc.writeSequenceLength(0)) return false;  // cell_voltage[]
    if (!enc.writeSequenceLength(0)) return false;  // cell_temperature[]
    if (!enc.writeString("")) return false;         // location
    if (!enc.writeString("")) return false;         // serial_number
    return true;
}

static bool serializeJointState(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    // sensor_msgs/JointState for a servo, motor or pump that reports back:
    // one joint, position from "angle" (degrees -> rad), velocity from
    // "velocity" when the device has it (taken as degrees/s -> rad/s, the
    // same unit as its position), effort from "current" when it has that (as
    // the device reports it - the records do not give its unit).
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double angle = 0.0, velocity = 0.0, current = 0.0;
    (void)AutoPubCDRSerializer_readFieldDouble(ctx, "angle", angle);
    const bool haveVelocity = AutoPubCDRSerializer_readFieldDouble(ctx, "velocity", velocity);
    const bool haveCurrent = AutoPubCDRSerializer_readFieldDouble(ctx, "current", current);
    if (!enc.writeSequenceLength(1)) return false;                  // name[]
    if (!enc.writeString("joint")) return false;
    if (!enc.writeSequenceLength(1)) return false;                  // position[]
    if (!enc.writeFloat64(angle * DEG_TO_RAD)) return false;
    if (!enc.writeSequenceLength(haveVelocity ? 1 : 0)) return false;   // velocity[]
    if (haveVelocity && !enc.writeFloat64(velocity * DEG_TO_RAD)) return false;
    if (!enc.writeSequenceLength(haveCurrent ? 1 : 0)) return false;    // effort[]
    if (haveCurrent && !enc.writeFloat64(current)) return false;
    return true;
}

/// @brief The numeric fallback: every attribute except the timestamp, as a
/// float64 with a layout dimension labelled with the attribute's name, so
/// `ros2 topic echo` shows which value is which and a subscriber can pick a
/// value by label rather than position.
static bool serializeFloat64MultiArray(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    auto included = [](const AutoPubAttrFieldDesc& d) {
        return d.name && *d.name && std::strcmp(d.name, "timeMs") != 0;
    };
    auto elements = [](const AutoPubAttrFieldDesc& d) -> uint32_t { return d.count ? d.count : 1; };
    uint32_t dims = 0, values = 0;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        if (!included(ctx.pFieldDescs[i])) continue;
        dims++;
        values += elements(ctx.pFieldDescs[i]);
    }

    // MultiArrayLayout: dim[] of {string label, uint32 size, uint32 stride},
    // uint32 data_offset.  An array attribute is one dim of its element count.
    if (!enc.writeSequenceLength(dims)) return false;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!included(d)) continue;
        if (!enc.writeString(d.name)) return false;
        if (!enc.writeUint32(elements(d))) return false;
        if (!enc.writeUint32(elements(d))) return false;
    }
    if (!enc.writeUint32(0)) return false;

    // float64 data[]
    if (!enc.writeSequenceLength(values)) return false;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!included(d)) continue;
        for (uint16_t element = 0; element < elements(d); element++)
        {
            double v = 0.0;
            (void)AutoPubCDRSerializer_readFieldElementDouble(ctx, d, element, v);
            if (!enc.writeFloat64(v)) return false;
        }
    }
    return true;
}

/// @brief sensor_msgs/Image, 32FC1, from a multizone ToF frame: one float32
/// per zone in metres, row-major, NaN where the zone has no valid reading.
/// Height and width come from the `grid` field (4 or 8); without one the
/// frame is taken as square.
static bool serializeDepthImage(CDREncoder& enc, const AutoPubCDRContext& ctx)
{
    const AutoPubAttrFieldDesc* pDist = findField(ctx, "dist");
    if (!pDist)
        return false;
    const uint16_t zones = multizoneZones(ctx, *pDist);
    uint32_t side = 1;
    while ((side + 1) * (side + 1) <= zones)
        side++;
    const uint32_t width = side, height = zones / side;

    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    if (!enc.writeUint32(height)) return false;
    if (!enc.writeUint32(width)) return false;
    if (!enc.writeString("32FC1")) return false;
    if (!enc.writeUint8(0)) return false;                       // is_bigendian: CDR LE
    if (!enc.writeUint32(width * sizeof(float))) return false;  // step
    if (!enc.writeSequenceLength(height * width * sizeof(float))) return false;
    for (uint32_t zone = 0; zone < height * width; zone++)
    {
        double mm = 0.0;
        float metres = std::numeric_limits<float>::quiet_NaN();
        if (multizoneValid(ctx, (uint16_t)zone) && AutoPubCDRSerializer_readFieldElementDouble(ctx, *pDist, (uint16_t)zone, mm))
            metres = (float)(mm * MM_TO_M);
        uint8_t bytes[sizeof(float)];
        std::memcpy(bytes, &metres, sizeof(bytes));
        if (!enc.writeBytes(bytes, sizeof(bytes))) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

bool AutoPubCDRSerializer_serializeString(
        const char* pText,
        uint8_t* pOutBuf, uint32_t outBufLen,
        uint32_t& outBytesWritten)
{
    outBytesWritten = 0;
    if (!pOutBuf || outBufLen == 0)
        return false;
    CDREncoder enc;
    enc.reset(pOutBuf, outBufLen);
    if (!enc.writeEncapsulationHeader(true)) return false;
    if (!enc.writeString(pText ? pText : "")) return false;
    outBytesWritten = enc.getPos();
    return true;
}

bool AutoPubCDRSerializer_serialize(
        AutoPubMsgKind kind,
        const AutoPubCDRContext& ctx,
        uint8_t* pOutBuf, uint32_t outBufLen,
        uint32_t& outBytesWritten)
{
    outBytesWritten = 0;
    if (!pOutBuf || outBufLen == 0)
        return false;

    CDREncoder enc;
    enc.reset(pOutBuf, outBufLen);
    if (!enc.writeEncapsulationHeader(true)) return false;

    bool ok = false;
    switch (kind)
    {
        case AutoPubMsgKind::Temperature:       ok = serializeTemperature(enc, ctx); break;
        case AutoPubMsgKind::RelativeHumidity:  ok = serializeRelativeHumidity(enc, ctx); break;
        case AutoPubMsgKind::FluidPressure:     ok = serializeFluidPressure(enc, ctx); break;
        case AutoPubMsgKind::Illuminance:       ok = serializeIlluminance(enc, ctx); break;
        case AutoPubMsgKind::Range:             ok = serializeRange(enc, ctx); break;
        case AutoPubMsgKind::Imu:               ok = serializeImu(enc, ctx, /*accelOnly=*/false); break;
        case AutoPubMsgKind::Accel:             ok = serializeImu(enc, ctx, /*accelOnly=*/true);  break;
        case AutoPubMsgKind::Float32:           ok = serializeFloat32(enc, ctx); break;
        case AutoPubMsgKind::Int32:             ok = serializeInt32(enc, ctx); break;
        case AutoPubMsgKind::Bool:              ok = serializeBool(enc, ctx); break;
        case AutoPubMsgKind::ByteMultiArray:    ok = serializeByteMultiArray(enc, ctx); break;
        case AutoPubMsgKind::Float32MultiArray: ok = serializeFloat32MultiArray(enc, ctx); break;
        case AutoPubMsgKind::Wrench:            ok = serializeWrench(enc, ctx); break;
        case AutoPubMsgKind::Joy:               ok = serializeJoy(enc, ctx); break;
        case AutoPubMsgKind::MagneticField:     ok = serializeMagneticField(enc, ctx); break;
        case AutoPubMsgKind::BatteryState:      ok = serializeBatteryState(enc, ctx); break;
        case AutoPubMsgKind::JointState:        ok = serializeJointState(enc, ctx); break;
        case AutoPubMsgKind::Float64MultiArray: ok = serializeFloat64MultiArray(enc, ctx); break;
        case AutoPubMsgKind::DepthImage:        ok = serializeDepthImage(enc, ctx); break;
        case AutoPubMsgKind::String:
        {
            // Slice 4.9 — generic JSON body.  Iterates every attribute in
            // the field-desc list (skipping the synthetic `timeMs` header
            // field) and formats scaled values into a compact JSON object:
            //   {"ts":123456,"temp":23.5,"humidity":54.1,"press":true}
            // Bool fields render as `true`/`false`; everything else as a
            // scaled double (with `%g`, capped at 4 fractional digits).
            // Output truncates gracefully once the buffer runs out; the
            // JSON is always closed with `}` so subscribers see well-formed
            // text even on overflow.
            char body[384];
            uint32_t bp = 0;
            auto emit = [&](const char* s) {
                while (*s && bp + 1 < sizeof(body))
                    body[bp++] = *s++;
            };
            auto emitChar = [&](char c) {
                if (bp + 1 < sizeof(body)) body[bp++] = c;
            };

            emitChar('{');
            {
                char tmp[32];
                std::snprintf(tmp, sizeof(tmp), "\"ts\":%u", (unsigned)ctx.timestampMs);
                emit(tmp);
            }
            for (uint16_t i = 0; i < ctx.fieldCount; i++)
            {
                const AutoPubAttrFieldDesc& d = ctx.pFieldDescs[i];
                if (!d.name || !*d.name)
                    continue;
                if (std::strcmp(d.name, "timeMs") == 0)
                    continue;

                emitChar(',');
                emitChar('"');
                emit(d.name);
                emit("\":");

                char val[40];
                if (d.type == AutoPubAttrType::Bool)
                {
                    bool bv = false;
                    (void)AutoPubCDRSerializer_readFieldBool(ctx, d.name, bv);
                    std::snprintf(val, sizeof(val), "%s", bv ? "true" : "false");
                }
                else
                {
                    double dv = 0.0;
                    (void)AutoPubCDRSerializer_readFieldDouble(ctx, d.name, dv, true);
                    std::snprintf(val, sizeof(val), "%.4g", dv);
                }
                emit(val);

                // Abort the loop if we're running out of buffer — we still
                // need to close the JSON object.
                if (bp + 2 >= sizeof(body))
                    break;
            }
            emitChar('}');
            body[bp < sizeof(body) ? bp : sizeof(body) - 1] = '\0';

            ok = enc.writeString(body);
            break;
        }
        case AutoPubMsgKind::Unknown:
        default:
            return false;
    }

    if (!ok)
        return false;
    outBytesWritten = enc.getPos();
    return true;
}

} // namespace AutoPub
} // namespace RaftRuntime
