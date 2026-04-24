/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPSAutoPubCDRSerializer — XCDR1 serialisers per RTPSAutoPubMsgKind
//
// See RTPSAutoPubCDRSerializer.h for API and design notes.
//
// All serialisers emit little-endian XCDR1 with the standard 4-byte
// encapsulation header (CDR_LE).  Alignment is computed from the start of the
// encapsulation, matching the existing CDREncoder behaviour that has already
// been validated against ROS 2 Humble / FastDDS in Phase 3.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RTPSAutoPubCDRSerializer.h"
#include "CDREncoder.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace RaftRuntime {
namespace RTPS {
namespace Runtime {
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

// Range (Humble) radiation_type constants.
static constexpr uint8_t RANGE_RADIATION_INFRARED = 1;

// Default fields-of-view used by the Range serialiser (unknown → 0).
static constexpr float   RANGE_DEFAULT_FOV        = 0.0f;
static constexpr float   RANGE_MIN_DEFAULT        = 0.0f;
static constexpr float   RANGE_MAX_PROX_DEFAULT   = 1.0f;       // normalised proximity
static constexpr float   RANGE_MAX_DIST_DEFAULT   = 2.0f;       // ~2 m for VL6180 / VL53L4CD

// ---------------------------------------------------------------------------
// Field lookup helpers
// ---------------------------------------------------------------------------

static const RTPSAutoPubAttrFieldDesc* findField(
        const RTPSAutoPubCDRContext& ctx, const char* name)
{
    if (!ctx.pFieldDescs || !name)
        return nullptr;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const RTPSAutoPubAttrFieldDesc& d = ctx.pFieldDescs[i];
        if (d.name && std::strcmp(d.name, name) == 0)
            return &d;
    }
    return nullptr;
}

/// Copy `n` bytes from the struct at the given offset, respecting
/// the buffer bound.  Returns false if out-of-range.
static bool readBytesFromStruct(const RTPSAutoPubCDRContext& ctx,
                                uint16_t offset, uint32_t n, void* pOut)
{
    if (!ctx.pStruct)
        return false;
    if ((uint32_t)offset + n > ctx.structSize)
        return false;
    std::memcpy(pOut, ctx.pStruct + offset, n);
    return true;
}

bool RTPSAutoPubCDRSerializer_readFieldDouble(
        const RTPSAutoPubCDRContext& ctx, const char* name,
        double& out, bool applyScale)
{
    const RTPSAutoPubAttrFieldDesc* pDesc = findField(ctx, name);
    if (!pDesc)
        return false;

    double raw = 0.0;
    switch (pDesc->type)
    {
        case RTPSAutoPubAttrType::Float:
        {
            float v = 0.0f;
            if (!readBytesFromStruct(ctx, pDesc->offset, 4, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Int32:
        {
            int32_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 4, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Uint32:
        {
            uint32_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 4, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Int16:
        {
            int16_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 2, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Uint16:
        {
            uint16_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 2, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Int8:
        {
            int8_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 1, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Uint8:
        {
            uint8_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 1, &v)) return false;
            raw = (double)v;
            break;
        }
        case RTPSAutoPubAttrType::Bool:
        {
            uint8_t v = 0;
            if (!readBytesFromStruct(ctx, pDesc->offset, 1, &v)) return false;
            raw = v ? 1.0 : 0.0;
            break;
        }
        default:
            return false;
    }

    if (applyScale)
    {
        if (pDesc->divisor != 0.0f && pDesc->divisor != 1.0f)
            raw /= (double)pDesc->divisor;
        if (pDesc->addend != 0.0f)
            raw += (double)pDesc->addend;
    }
    out = raw;
    return true;
}

bool RTPSAutoPubCDRSerializer_readFieldInt32(
        const RTPSAutoPubCDRContext& ctx, const char* name,
        int32_t& out, bool applyScale)
{
    double v = 0.0;
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, name, v, applyScale))
        return false;
    // Truncate toward zero (standard C cast semantics).
    if (v >  (double)INT32_MAX) { out = INT32_MAX; return true; }
    if (v < -(double)INT32_MAX) { out = INT32_MIN; return true; }
    out = (int32_t)v;
    return true;
}

bool RTPSAutoPubCDRSerializer_readFieldBool(
        const RTPSAutoPubCDRContext& ctx, const char* name, bool& out)
{
    const RTPSAutoPubAttrFieldDesc* pDesc = findField(ctx, name);
    if (!pDesc)
        return false;
    if (pDesc->type == RTPSAutoPubAttrType::Bool ||
        pDesc->type == RTPSAutoPubAttrType::Uint8 ||
        pDesc->type == RTPSAutoPubAttrType::Int8)
    {
        uint8_t v = 0;
        if (!readBytesFromStruct(ctx, pDesc->offset, 1, &v))
            return false;
        out = (v != 0);
        return true;
    }
    double v = 0.0;
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, name, v, false))
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

static bool serializeTemperature(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double temperature = 0.0;
    (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "temperature", temperature);
    if (!enc.writeFloat64(temperature)) return false;
    if (!enc.writeFloat64(0.0)) return false; // variance unknown
    return true;
}

static bool serializeRelativeHumidity(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double humidity = 0.0;
    (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "humidity", humidity);
    humidity *= PERCENT_TO_UNIT; // % → 0..1
    if (!enc.writeFloat64(humidity)) return false;
    if (!enc.writeFloat64(0.0)) return false;
    return true;
}

static bool serializeFluidPressure(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double pressure = 0.0;
    (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "pressure", pressure);
    pressure *= HPA_TO_PA;
    if (!enc.writeFloat64(pressure)) return false;
    if (!enc.writeFloat64(0.0)) return false;
    return true;
}

static bool serializeIlluminance(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    double lux = 0.0;
    // Prefer `als` (VCNL/VEML) then `illuminance`.
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "als", lux))
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "illuminance", lux);
    if (!enc.writeFloat64(lux)) return false;
    if (!enc.writeFloat64(0.0)) return false;
    return true;
}

static bool serializeRange(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    if (!writeHeader(enc, ctx.timestampMs, ctx.frameId)) return false;
    if (!enc.writeUint8(RANGE_RADIATION_INFRARED)) return false;
    if (!enc.writeFloat32(RANGE_DEFAULT_FOV)) return false;
    if (!enc.writeFloat32(RANGE_MIN_DEFAULT)) return false;

    double rangeVal = 0.0;
    float  maxRange = RANGE_MAX_DIST_DEFAULT;
    if (RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "dist", rangeVal))
    {
        rangeVal *= MM_TO_M;   // mm → m
        maxRange = RANGE_MAX_DIST_DEFAULT;
    }
    else if (RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "prox", rangeVal))
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

static bool serializeImu(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx, bool accelOnly)
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
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "gx", gx);
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "gy", gy);
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "gz", gz);
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
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "ax", ax))
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "x", ax);
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "ay", ay))
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "y", ay);
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "az", az))
        (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "z", az);
    ax *= GRAVITY_MPS2; ay *= GRAVITY_MPS2; az *= GRAVITY_MPS2;
    if (!enc.writeFloat64(ax)) return false;
    if (!enc.writeFloat64(ay)) return false;
    if (!enc.writeFloat64(az)) return false;
    if (!writeCovarianceZero9(enc)) return false;

    return true;
}

static bool serializeFloat32(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    // Try common scalar names in priority order.
    double v = 0.0;
    if (!RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "angle", v) &&
        !RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "moisture", v) &&
        !RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "value", v) &&
        !RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "data", v))
    {
        // Fall back to the first non-timestamp field.
        for (uint16_t i = 0; i < ctx.fieldCount; i++)
        {
            const auto& d = ctx.pFieldDescs[i];
            if (d.name && std::strcmp(d.name, "timeMs") != 0)
            {
                (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, d.name, v);
                break;
            }
        }
    }
    return enc.writeFloat32((float)v);
}

static bool serializeInt32(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    int32_t v = 0;
    if (!RTPSAutoPubCDRSerializer_readFieldInt32(ctx, "rotation", v) &&
        !RTPSAutoPubCDRSerializer_readFieldInt32(ctx, "count", v) &&
        !RTPSAutoPubCDRSerializer_readFieldInt32(ctx, "value", v) &&
        !RTPSAutoPubCDRSerializer_readFieldInt32(ctx, "data", v))
    {
        for (uint16_t i = 0; i < ctx.fieldCount; i++)
        {
            const auto& d = ctx.pFieldDescs[i];
            if (d.name && std::strcmp(d.name, "timeMs") != 0)
            {
                (void)RTPSAutoPubCDRSerializer_readFieldInt32(ctx, d.name, v);
                break;
            }
        }
    }
    return enc.writeInt32(v);
}

static bool serializeBool(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    bool v = false;
    if (!RTPSAutoPubCDRSerializer_readFieldBool(ctx, "press", v) &&
        !RTPSAutoPubCDRSerializer_readFieldBool(ctx, "state", v) &&
        !RTPSAutoPubCDRSerializer_readFieldBool(ctx, "value", v) &&
        !RTPSAutoPubCDRSerializer_readFieldBool(ctx, "data", v))
    {
        for (uint16_t i = 0; i < ctx.fieldCount; i++)
        {
            const auto& d = ctx.pFieldDescs[i];
            if (d.name && std::strcmp(d.name, "timeMs") != 0)
            {
                (void)RTPSAutoPubCDRSerializer_readFieldBool(ctx, d.name, v);
                break;
            }
        }
    }
    return enc.writeBool(v);
}

static bool serializeByteMultiArray(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
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
        if (d.type == RTPSAutoPubAttrType::Bool ||
            d.type == RTPSAutoPubAttrType::Uint8 ||
            d.type == RTPSAutoPubAttrType::Int8)
            count++;
    }
    if (!enc.writeSequenceLength(count)) return false;
    for (uint16_t i = 0; i < ctx.fieldCount; i++)
    {
        const auto& d = ctx.pFieldDescs[i];
        if (!d.name) continue;
        if (std::strcmp(d.name, "timeMs") == 0) continue;
        if (d.type != RTPSAutoPubAttrType::Bool &&
            d.type != RTPSAutoPubAttrType::Uint8 &&
            d.type != RTPSAutoPubAttrType::Int8)
            continue;
        bool b = false;
        RTPSAutoPubCDRSerializer_readFieldBool(ctx, d.name, b);
        if (!enc.writeUint8(b ? 1 : 0)) return false;
    }
    return true;
}

static bool serializeFloat32MultiArray(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
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
        RTPSAutoPubCDRSerializer_readFieldDouble(ctx, d.name, v);
        if (!enc.writeFloat32((float)v)) return false;
    }
    return true;
}

static bool serializeWrench(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
{
    // geometry_msgs/Wrench: Vector3 force, Vector3 torque (all float64).
    // HX711 → force.z = "force" field (N).  Valid flag gates the caller.
    double f = 0.0;
    (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, "force", f);
    if (!enc.writeFloat64(0.0)) return false; // force.x
    if (!enc.writeFloat64(0.0)) return false; // force.y
    if (!enc.writeFloat64(f)) return false;   // force.z
    if (!enc.writeFloat64(0.0)) return false; // torque.x
    if (!enc.writeFloat64(0.0)) return false; // torque.y
    if (!enc.writeFloat64(0.0)) return false; // torque.z
    return true;
}

static bool serializeJoy(CDREncoder& enc, const RTPSAutoPubCDRContext& ctx)
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
        RTPSAutoPubCDRSerializer_readFieldDouble(ctx, an, v);
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
        RTPSAutoPubCDRSerializer_readFieldInt32(ctx, d.name, v);
        if (!enc.writeInt32(v)) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

bool RTPSAutoPubCDRSerializer_serializeString(
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

bool RTPSAutoPubCDRSerializer_serialize(
        RTPSAutoPubMsgKind kind,
        const RTPSAutoPubCDRContext& ctx,
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
        case RTPSAutoPubMsgKind::Temperature:       ok = serializeTemperature(enc, ctx); break;
        case RTPSAutoPubMsgKind::RelativeHumidity:  ok = serializeRelativeHumidity(enc, ctx); break;
        case RTPSAutoPubMsgKind::FluidPressure:     ok = serializeFluidPressure(enc, ctx); break;
        case RTPSAutoPubMsgKind::Illuminance:       ok = serializeIlluminance(enc, ctx); break;
        case RTPSAutoPubMsgKind::Range:             ok = serializeRange(enc, ctx); break;
        case RTPSAutoPubMsgKind::Imu:               ok = serializeImu(enc, ctx, /*accelOnly=*/false); break;
        case RTPSAutoPubMsgKind::Accel:             ok = serializeImu(enc, ctx, /*accelOnly=*/true);  break;
        case RTPSAutoPubMsgKind::Float32:           ok = serializeFloat32(enc, ctx); break;
        case RTPSAutoPubMsgKind::Int32:             ok = serializeInt32(enc, ctx); break;
        case RTPSAutoPubMsgKind::Bool:              ok = serializeBool(enc, ctx); break;
        case RTPSAutoPubMsgKind::ByteMultiArray:    ok = serializeByteMultiArray(enc, ctx); break;
        case RTPSAutoPubMsgKind::Float32MultiArray: ok = serializeFloat32MultiArray(enc, ctx); break;
        case RTPSAutoPubMsgKind::Wrench:            ok = serializeWrench(enc, ctx); break;
        case RTPSAutoPubMsgKind::Joy:               ok = serializeJoy(enc, ctx); break;
        case RTPSAutoPubMsgKind::String:
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
                const RTPSAutoPubAttrFieldDesc& d = ctx.pFieldDescs[i];
                if (!d.name || !*d.name)
                    continue;
                if (std::strcmp(d.name, "timeMs") == 0)
                    continue;

                emitChar(',');
                emitChar('"');
                emit(d.name);
                emit("\":");

                char val[40];
                if (d.type == RTPSAutoPubAttrType::Bool)
                {
                    bool bv = false;
                    (void)RTPSAutoPubCDRSerializer_readFieldBool(ctx, d.name, bv);
                    std::snprintf(val, sizeof(val), "%s", bv ? "true" : "false");
                }
                else
                {
                    double dv = 0.0;
                    (void)RTPSAutoPubCDRSerializer_readFieldDouble(ctx, d.name, dv, true);
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
        case RTPSAutoPubMsgKind::Unknown:
        default:
            return false;
    }

    if (!ok)
        return false;
    outBytesWritten = enc.getPos();
    return true;
}

} // namespace AutoPub
} // namespace Runtime
} // namespace RTPS
} // namespace RaftRuntime
