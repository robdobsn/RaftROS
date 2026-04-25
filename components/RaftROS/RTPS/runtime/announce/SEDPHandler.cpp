/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SEDP Handler - Simple Endpoint Discovery Protocol message building
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "SEDPHandler.h"
#include "runtime/wire/RTPSMessage.h"
#include "runtime/core/RTPSParticipant.h"

SEDPHandler::SEDPHandler()
{
}

SEDPHandler::~SEDPHandler()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Write a CDR string into a ParameterList value: uint32 length + chars (incl null) + padding
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::writePLString(uint8_t* pBuf, const char* str)
{
    uint32_t slen = (uint32_t)strlen(str) + 1;  // include null terminator
    uint32_t pos = 0;
    // String length (LE uint32)
    pBuf[pos++] = slen & 0xFF;
    pBuf[pos++] = (slen >> 8) & 0xFF;
    pBuf[pos++] = (slen >> 16) & 0xFF;
    pBuf[pos++] = (slen >> 24) & 0xFF;
    // String data including null
    memcpy(pBuf + pos, str, slen);
    pos += slen;
    // Pad to 4-byte alignment
    while (pos % 4 != 0)
        pBuf[pos++] = 0;
    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helpers for writing LE values into buffer
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void putLE16(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static void putLE32(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Emit the full set of default QoS PIDs that Fast DDS (rmw_fastrtps) writes
// on every SEDP DATA submessage.  Although each QoS has a well-defined spec
// default, Fast DDS' QoS matcher / type-introspection path has been observed
// to silently reject DATA samples when any of these PIDs is missing from the
// publication / subscription announcement — the raw SEDP matches but typed
// `ros2 topic echo` delivers nothing.
//
// Durations are Duration_t (int32 sec + uint32 nanosec).  INFINITE is
// {0x7FFFFFFF, 0xFFFFFFFF}; ZERO is {0, 0}.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t writeDefaultEndpointQoS(uint8_t* p)
{
    uint32_t pp = 0;

    // PID_DEADLINE (len 8) — INFINITE
    putLE16(p + pp, PID_DEADLINE); pp += 2;
    putLE16(p + pp, 8); pp += 2;
    putLE32(p + pp, 0x7FFFFFFFu); pp += 4;   // sec
    putLE32(p + pp, 0xFFFFFFFFu); pp += 4;   // nanosec

    // PID_LATENCY_BUDGET (len 8) — ZERO
    putLE16(p + pp, PID_LATENCY_BUDGET); pp += 2;
    putLE16(p + pp, 8); pp += 2;
    putLE32(p + pp, 0); pp += 4;
    putLE32(p + pp, 0); pp += 4;

    // PID_LIVELINESS (len 12) — kind=AUTOMATIC(0), lease=INFINITE
    putLE16(p + pp, PID_LIVELINESS); pp += 2;
    putLE16(p + pp, 12); pp += 2;
    putLE32(p + pp, 0); pp += 4;             // AUTOMATIC
    putLE32(p + pp, 0x7FFFFFFFu); pp += 4;   // sec
    putLE32(p + pp, 0xFFFFFFFFu); pp += 4;   // nanosec

    // PID_LIFESPAN (len 8) — INFINITE
    putLE16(p + pp, PID_LIFESPAN); pp += 2;
    putLE16(p + pp, 8); pp += 2;
    putLE32(p + pp, 0x7FFFFFFFu); pp += 4;
    putLE32(p + pp, 0xFFFFFFFFu); pp += 4;

    // PID_OWNERSHIP (len 4) — SHARED(0)
    putLE16(p + pp, PID_OWNERSHIP); pp += 2;
    putLE16(p + pp, 4); pp += 2;
    putLE32(p + pp, 0); pp += 4;

    // PID_DESTINATION_ORDER (len 4) — BY_RECEPTION_TIMESTAMP(0)
    putLE16(p + pp, PID_DESTINATION_ORDER); pp += 2;
    putLE16(p + pp, 4); pp += 2;
    putLE32(p + pp, 0); pp += 4;

    // PID_PRESENTATION (len 8) — access_scope=INSTANCE(0), coherent=0, ordered=0
    putLE16(p + pp, PID_PRESENTATION); pp += 2;
    putLE16(p + pp, 8); pp += 2;
    putLE32(p + pp, 0); pp += 4;             // access_scope
    p[pp++] = 0; p[pp++] = 0;                // coherent, ordered
    p[pp++] = 0; p[pp++] = 0;                // pad

    // PID_PARTITION (len 4) — empty string sequence (count=0)
    putLE16(p + pp, PID_PARTITION); pp += 2;
    putLE16(p + pp, 4); pp += 2;
    putLE32(p + pp, 0); pp += 4;

    // PID_TIME_BASED_FILTER (len 8) — ZERO
    putLE16(p + pp, PID_TIME_BASED_FILTER); pp += 2;
    putLE16(p + pp, 8); pp += 2;
    putLE32(p + pp, 0); pp += 4;
    putLE32(p + pp, 0); pp += 4;

    // PID_TOPIC_DATA (len 4) — empty
    putLE16(p + pp, PID_TOPIC_DATA); pp += 2;
    putLE16(p + pp, 4); pp += 2;
    putLE32(p + pp, 0); pp += 4;

    // PID_GROUP_DATA (len 4) — empty
    putLE16(p + pp, PID_GROUP_DATA); pp += 2;
    putLE16(p + pp, 4); pp += 2;
    putLE32(p + pp, 0); pp += 4;

    // PID_DURABILITY_SERVICE (len 28) — defaults: cleanup=0, KEEP_LAST, depth=1, -1/-1/-1
    putLE16(p + pp, PID_DURABILITY_SERVICE); pp += 2;
    putLE16(p + pp, 28); pp += 2;
    putLE32(p + pp, 0); pp += 4;             // cleanup sec
    putLE32(p + pp, 0); pp += 4;             // cleanup nanosec
    putLE32(p + pp, 0); pp += 4;             // history_kind = KEEP_LAST
    putLE32(p + pp, 1); pp += 4;             // history_depth
    putLE32(p + pp, 0xFFFFFFFFu); pp += 4;   // max_samples = -1
    putLE32(p + pp, 0xFFFFFFFFu); pp += 4;   // max_instances = -1
    putLE32(p + pp, 0xFFFFFFFFu); pp += 4;   // max_samples_per_instance = -1

    // PID_TYPE_MAX_SIZE_SERIALIZED (len 4) — 0 (unbounded / unknown)
    putLE16(p + pp, PID_TYPE_MAX_SIZE_SERIALIZED); pp += 2;
    putLE16(p + pp, 4); pp += 2;
    putLE32(p + pp, 0); pp += 4;

    return pp;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS 2 type-hash lookup.  rmw_fastrtps (ROS 2 Jazzy) encodes the RIHS01 type
// hash inside PID_USER_DATA on every publication/subscription SEDP DATA:
//
//     octet-seq payload = "typehash=RIHS01_<64-hex-chars>;"
//
// The typed reader (`ros2 topic echo <topic> <type>`) silently discards
// samples from any endpoint whose advertised typehash does not match its own
// generated type support. Without this PID the reader matches at the RTPS
// layer but never delivers samples to the RMW/RCL.
//
// This table maps DDS type names we emit (the `sensor_msgs::msg::dds_::Xxx_`
// style used by rmw_fastrtps) to the hash string that ROS 2 Jazzy computes
// for that IDL. The value is the contents of the `hash_string` field in the
// installed `share/<pkg>/msg/<Type>.json` descriptor (ROS 2 REP-2011).
// Returns nullptr when no match is known; callers should emit an empty
// PID_USER_DATA in that case (keeps wire format valid, but typed readers
// for unknown types won't receive).
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const char* lookupRos2TypeHash(const char* typeName)
{
    if (!typeName)
        return nullptr;
    struct Entry { const char* type; const char* hash; };
    static const Entry table[] = {
        { "sensor_msgs::msg::dds_::Range_",
          "b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a" },
        { "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
          "91a0593bacdcc50ea9bdcf849a938b128412cc1ea821245c663bcd26f83c295e" },
    };
    for (const auto& e : table)
    {
        if (strcmp(e.type, typeName) == 0)
            return e.hash;
    }
    return nullptr;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Emit PID_USER_DATA carrying the ROS 2 typehash if known, else empty.
// The payload is a DDS octet-sequence: uint32 length + bytes + pad-to-4.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t writeTypeHashUserData(uint8_t* p, const char* typeName)
{
    const char* hashHex = lookupRos2TypeHash(typeName);
    if (!hashHex)
    {
        // Empty octet sequence (count=0)
        putLE16(p + 0, PID_USER_DATA);
        putLE16(p + 2, 4);
        putLE32(p + 4, 0);
        return 8;
    }

    // Build the string:  "typehash=RIHS01_<64hex>;"  (no trailing NUL).
    // ROS 2 writes this as an octet sequence, not a CDR string, so no
    // terminator is included in the length.
    char buf[96];
    const int len = snprintf(buf, sizeof(buf), "typehash=RIHS01_%s;", hashHex);
    if (len <= 0 || len >= (int)sizeof(buf))
    {
        // Fallback: emit empty if formatting failed.
        putLE16(p + 0, PID_USER_DATA);
        putLE16(p + 2, 4);
        putLE32(p + 4, 0);
        return 8;
    }

    const uint32_t seqLen = (uint32_t)len;                      // octet count
    const uint32_t valueBytes = 4 + seqLen;                     // uint32 + bytes
    const uint32_t valuePadded = (valueBytes + 3) & ~3u;        // 4-byte align
    putLE16(p + 0, PID_USER_DATA);
    putLE16(p + 2, (uint16_t)valuePadded);
    putLE32(p + 4, seqLen);
    memcpy(p + 8, buf, seqLen);
    // zero-pad to alignment
    for (uint32_t i = valueBytes; i < valuePadded; ++i)
        p[4 + i] = 0;
    return 4 + valuePadded;  // parameter header (4) + value
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build SEDP publication announcement RTPS message
// Announces a DataWriter endpoint to a remote participant
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildPublicationMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* writerEntityId,
    const char* topicName,
    const char* typeName,
    uint32_t reliabilityKind,
    uint32_t durabilityKind,
    uint64_t sequenceNumber,
    uint32_t ipAddrNetOrder,
    uint32_t heartbeatCount,
    uint64_t hbLastSN)
{
    if (!pBuf || bufLen < 900)
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST - set destination participant
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // Build SEDP publication ParameterList payload
    uint8_t payload[800];
    uint32_t pp = 0;

    // CDR Encapsulation: PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;  // PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;

    // PID_PROTOCOL_VERSION (value: 2 bytes + 2 pad) — Fast DDS includes
    // this in every SEDP DATA; omitting it appears to cause the reader to
    // silently drop samples ("A message was lost" on the RX side).
    putLE16(payload + pp, PID_PROTOCOL_VERSION); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    payload[pp++] = RTPS_VERSION_MAJOR;
    payload[pp++] = RTPS_VERSION_MINOR;
    payload[pp++] = 0;
    payload[pp++] = 0;

    // PID_VENDORID (value: 2 bytes + 2 pad)
    putLE16(payload + pp, PID_VENDORID); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    payload[pp++] = RTPS_VENDOR_ID[0];
    payload[pp++] = RTPS_VENDOR_ID[1];
    payload[pp++] = 0;
    payload[pp++] = 0;

    // PID_ENDPOINT_GUID (guidPrefix + entityId of the announced writer)
    putLE16(payload + pp, PID_ENDPOINT_GUID); pp += 2;
    putLE16(payload + pp, 16);                pp += 2;  // length = 16
    memcpy(payload + pp, participant.getGuidPrefix(), 12);
    pp += 12;
    memcpy(payload + pp, writerEntityId, 4);
    pp += 4;

    // PID_KEY_HASH (16 bytes) — key of the BuiltinPublicationsTopic, which
    // is the endpoint GUID (guidPrefix + entityId).  Fast DDS emits this on
    // every SEDP publication DATA; without it the typed reader matches but
    // discards samples on receipt.
    putLE16(payload + pp, PID_KEY_HASH); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, writerEntityId, 4); pp += 4;

    // PID_TOPIC_NAME (CDR string in parameter value)
    uint32_t topicStrLen = (uint32_t)strlen(topicName) + 1;
    uint32_t topicPadded = (topicStrLen + 3) & ~3u;
    putLE16(payload + pp, PID_TOPIC_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + topicPadded)); pp += 2;
    pp += writePLString(payload + pp, topicName);

    // PID_TYPE_NAME (CDR string in parameter value)
    uint32_t typeStrLen = (uint32_t)strlen(typeName) + 1;
    uint32_t typePadded = (typeStrLen + 3) & ~3u;
    putLE16(payload + pp, PID_TYPE_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + typePadded)); pp += 2;
    pp += writePLString(payload + pp, typeName);

    // PID_RELIABILITY (kind: uint32 + max_blocking_time: Duration_t = 8 bytes)
    putLE16(payload + pp, PID_RELIABILITY); pp += 2;
    putLE16(payload + pp, 12); pp += 2;  // length = 12
    putLE32(payload + pp, reliabilityKind); pp += 4;
    // max_blocking_time: 0.1s for RELIABLE
    putLE32(payload + pp, 0); pp += 4;   // seconds
    putLE32(payload + pp, 100000000); pp += 4; // fraction (~0.1s)

    // PID_DURABILITY (kind: uint32)
    putLE16(payload + pp, PID_DURABILITY); pp += 2;
    putLE16(payload + pp, 4); pp += 2;   // length = 4
    putLE32(payload + pp, durabilityKind); pp += 4;

    // PID_UNICAST_LOCATOR — tells remote where to send user data
    if (ipAddrNetOrder != 0)
    {
        putLE16(payload + pp, PID_UNICAST_LOCATOR); pp += 2;
        putLE16(payload + pp, 24); pp += 2;  // length = 24 (locator struct)
        RTPSParticipant::buildLocatorUDPv4(payload + pp, ipAddrNetOrder,
                                           participant.getUserUnicastPort());
        pp += 24;
    }

    // PID_PARTICIPANT_GUID
    putLE16(payload + pp, PID_PARTICIPANT_GUID); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, ENTITYID_PARTICIPANT, 4); pp += 4;

    // PID_DATA_REPRESENTATION — declare XCDR1 (classic CDR_LE) support.
    // Required by Fast DDS 3.x (ROS 2 Jazzy) when the reader's generated
    // type support is `@appendable` (the ROS 2 IDL default). Without this,
    // Fast CDR rejects our CDR_LE payloads with a deserialization exception
    // even though the bytes are structurally correct.
    //
    // DataRepresentationQosPolicy wire format:
    //   sequence<int16> value;
    // CDR: uint32 length, then N*int16, padded to 4-byte boundary.
    // Advertise ONLY XCDR1 (matches reference publisher exactly). Earlier
    // variants advertised [XCDR1, XCDR2]; while raw rclpy subscribers
    // received samples under that, TYPED subscribers silently dropped them
    // (Fast DDS appears to negotiate XCDR2 as the preferred format when
    // offered, then fails to deliver XCDR1-encap payloads through the typed
    // deserialization path).
#ifndef RAFTROS_DISABLE_DATA_REPRESENTATION_PID
    putLE16(payload + pp, PID_DATA_REPRESENTATION); pp += 2;
    putLE16(payload + pp, 8); pp += 2;                  // parameter length (aligned)
    putLE32(payload + pp, 1); pp += 4;                  // sequence length = 1
    putLE16(payload + pp, (uint16_t)DATA_REPRESENTATION_XCDR1); pp += 2;
    putLE16(payload + pp, 0); pp += 2;                  // pad to 4
#endif

    // NOTE: PID_TYPE_CONSISTENCY is intentionally NOT emitted here. Per
    // DDS-XTypes §7.6.3.5, TypeConsistencyEnforcementQosPolicy is a reader-
    // side QoS only. Fast DDS 3.x logs an RTPS_PROXY_DATA error and rejects
    // the remote writer if a publication advertises it. Subscription
    // messages emit it (see buildSubscriptionMessage below).

    // Default QoS policy PIDs (DEADLINE, LIVELINESS, etc.)
    pp += writeDefaultEndpointQoS(payload + pp);

    // PID_USER_DATA carrying ROS 2 typehash (required by rmw_fastrtps typed readers)
    pp += writeTypeHashUserData(payload + pp, typeName);

    // PID_SENTINEL
    putLE16(payload + pp, PID_SENTINEL); pp += 2;
    putLE16(payload + pp, 0); pp += 2;

    // Write DATA submessage: writerEntityId=SEDP_PUBLICATIONS_WRITER,
    // readerEntityId=SEDP_PUBLICATIONS_READER (remote)
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT for this SEDP writer (RELIABLE built-in endpoint)
    uint32_t hbCount = heartbeatCount > 0 ? heartbeatCount : (uint32_t)sequenceNumber;
    // lastSN MUST be the writer's true high-water mark across all of its
    // endpoints.  If the caller passes hbLastSN==0 we fall back to the
    // per-DATA seq, which is only safe in unit tests / single-endpoint
    // scenarios; the runtime announce path threads through the actual max.
    uint64_t hbLast = hbLastSN > 0 ? hbLastSN : sequenceNumber;
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
        0, 1,  // firstSN = 1
        (uint32_t)(hbLast >> 32), (uint32_t)(hbLast & 0xFFFFFFFF),  // lastSN
        hbCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build SEDP publication DISPOSE message.  Carries PID_STATUS_INFO with
// DisposedFlag | UnregisteredFlag set so peers drop the matching writer.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildPublicationDisposeMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* writerEntityId,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount)
{
    if (!pBuf || bufLen < 200)
        return 0;

    uint32_t pos = 0;

    // RTPS Header + INFO_DST + INFO_TS
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // Dispose ParameterList payload — minimal: status-info + key identifying
    // the writer being disposed (PID_KEY_HASH + PID_ENDPOINT_GUID), plus a
    // sentinel.  FastDDS matches the disposed writer by GUID; topic/type
    // parameters are not required on a dispose.
    uint8_t payload[80];
    uint32_t pp = 0;

    // CDR Encapsulation: PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;

    // PID_STATUS_INFO (0x0071), len 4.  StatusInfo_t is a 4-octet big-endian
    // value; the last octet carries bit 0 = DisposedFlag, bit 1 = UnregisteredFlag.
    putLE16(payload + pp, PID_STATUS_INFO); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;  // Disposed | Unregistered

    // PID_KEY_HASH (0x0070), len 16 — endpoint GUID as the dispose key.
    putLE16(payload + pp, PID_KEY_HASH); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, writerEntityId, 4); pp += 4;

    // PID_ENDPOINT_GUID — same GUID in the documented dispose parameter.
    putLE16(payload + pp, PID_ENDPOINT_GUID); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, writerEntityId, 4); pp += 4;

    // PID_SENTINEL
    putLE16(payload + pp, PID_SENTINEL); pp += 2;
    putLE16(payload + pp, 0); pp += 2;

    // DATA submessage on the builtin PublicationsWriter endpoint.
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT for this built-in writer (same as announce path).
    uint32_t hbCount = heartbeatCount > 0 ? heartbeatCount : (uint32_t)sequenceNumber;
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER,
        ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER,
        0, 1,                                          // firstSN
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),    // lastSN
        hbCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build user DATA message (e.g., ros_discovery_info) with HEARTBEAT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildUserDataMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* writerEntityId,
    const uint8_t* pPayload, uint32_t payloadLen,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount,
    uint64_t firstSN,
    const uint8_t* keyHash16)
{
    // Reserve 24 extra bytes if inline QoS w/ PID_KEY_HASH is requested.
    const uint32_t inlineQosOverhead = (keyHash16 != nullptr) ? 24 : 0;
    if (!pBuf || bufLen < (uint32_t)(80 + inlineQosOverhead + payloadLen))
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // DATA submessage with user payload (with inline-QoS PID_KEY_HASH if requested)
    if (keyHash16)
    {
        pos += RTPSMessage::writeDataSubmessageWithKeyHash(pBuf + pos, bufLen - pos,
            ENTITYID_UNKNOWN,
            writerEntityId,
            0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
            keyHash16,
            pPayload, payloadLen);
    }
    else
    {
        pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
            ENTITYID_UNKNOWN,
            writerEntityId,
            0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
            pPayload, payloadLen);
    }

    // HEARTBEAT (RELIABLE DataWriter)
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_UNKNOWN,
        writerEntityId,
        0, (uint32_t)(firstSN & 0xFFFFFFFF),
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        heartbeatCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build SEDP subscription announcement RTPS message
// Announces a DataReader endpoint to a remote participant
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildSubscriptionMessage(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    const uint8_t* readerEntityId,
    const char* topicName,
    const char* typeName,
    uint32_t reliabilityKind,
    uint32_t durabilityKind,
    uint64_t sequenceNumber,
    uint32_t ipAddrNetOrder,
    uint32_t heartbeatCount,
    uint64_t hbLastSN)
{
    if (!pBuf || bufLen < 900)
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // Build SEDP subscription ParameterList payload
    uint8_t payload[800];
    uint32_t pp = 0;

    // CDR Encapsulation: PL_CDR_LE
    payload[pp++] = 0x00;
    payload[pp++] = 0x03;
    payload[pp++] = 0x00;
    payload[pp++] = 0x00;

    // PID_PROTOCOL_VERSION (see buildPublicationMessage notes)
    putLE16(payload + pp, PID_PROTOCOL_VERSION); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    payload[pp++] = RTPS_VERSION_MAJOR;
    payload[pp++] = RTPS_VERSION_MINOR;
    payload[pp++] = 0;
    payload[pp++] = 0;

    // PID_VENDORID
    putLE16(payload + pp, PID_VENDORID); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    payload[pp++] = RTPS_VENDOR_ID[0];
    payload[pp++] = RTPS_VENDOR_ID[1];
    payload[pp++] = 0;
    payload[pp++] = 0;

    // PID_ENDPOINT_GUID
    putLE16(payload + pp, PID_ENDPOINT_GUID); pp += 2;
    putLE16(payload + pp, 16);                pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12);
    pp += 12;
    memcpy(payload + pp, readerEntityId, 4);
    pp += 4;

    // PID_KEY_HASH — endpoint GUID, key of BuiltinSubscriptionsTopic
    putLE16(payload + pp, PID_KEY_HASH); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, readerEntityId, 4); pp += 4;

    // PID_TOPIC_NAME
    uint32_t topicPadded = ((uint32_t)strlen(topicName) + 1 + 3) & ~3u;
    putLE16(payload + pp, PID_TOPIC_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + topicPadded)); pp += 2;
    pp += writePLString(payload + pp, topicName);

    // PID_TYPE_NAME
    uint32_t typePadded = ((uint32_t)strlen(typeName) + 1 + 3) & ~3u;
    putLE16(payload + pp, PID_TYPE_NAME); pp += 2;
    putLE16(payload + pp, (uint16_t)(4 + typePadded)); pp += 2;
    pp += writePLString(payload + pp, typeName);

    // PID_RELIABILITY
    putLE16(payload + pp, PID_RELIABILITY); pp += 2;
    putLE16(payload + pp, 12); pp += 2;
    putLE32(payload + pp, reliabilityKind); pp += 4;
    putLE32(payload + pp, 0); pp += 4;
    putLE32(payload + pp, 100000000); pp += 4;

    // PID_DURABILITY
    putLE16(payload + pp, PID_DURABILITY); pp += 2;
    putLE16(payload + pp, 4); pp += 2;
    putLE32(payload + pp, durabilityKind); pp += 4;

    // PID_UNICAST_LOCATOR — tells remote where to send user data
    if (ipAddrNetOrder != 0)
    {
        putLE16(payload + pp, PID_UNICAST_LOCATOR); pp += 2;
        putLE16(payload + pp, 24); pp += 2;
        RTPSParticipant::buildLocatorUDPv4(payload + pp, ipAddrNetOrder,
                                           participant.getUserUnicastPort());
        pp += 24;
    }

    // PID_PARTICIPANT_GUID
    putLE16(payload + pp, PID_PARTICIPANT_GUID); pp += 2;
    putLE16(payload + pp, 16); pp += 2;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    memcpy(payload + pp, ENTITYID_PARTICIPANT, 4); pp += 4;

    // PID_DATA_REPRESENTATION — XCDR1 only (see notes in
    // buildPublicationMessage above; advertising XCDR2 as well breaks
    // typed rclpy subscribers).
#ifndef RAFTROS_DISABLE_DATA_REPRESENTATION_PID
    putLE16(payload + pp, PID_DATA_REPRESENTATION); pp += 2;
    putLE16(payload + pp, 8); pp += 2;
    putLE32(payload + pp, 1); pp += 4;
    putLE16(payload + pp, (uint16_t)DATA_REPRESENTATION_XCDR1); pp += 2;
    putLE16(payload + pp, 0); pp += 2;
#endif

    // PID_TYPE_CONSISTENCY — see notes in buildPublicationMessage.
    putLE16(payload + pp, PID_TYPE_CONSISTENCY); pp += 2;
    putLE16(payload + pp, 8); pp += 2;
    putLE16(payload + pp, 1); pp += 2;  // kind = ALLOW_TYPE_COERCION
    payload[pp++] = 0;  // ignore_sequence_bounds
    payload[pp++] = 1;  // ignore_string_bounds
    payload[pp++] = 0;  // ignore_member_names
    payload[pp++] = 0;  // prevent_type_widening
    payload[pp++] = 0;  // force_type_validation
    payload[pp++] = 0;  // pad
    payload[pp++] = 0;  // pad


    // Default QoS policy PIDs (DEADLINE, LIVELINESS, etc.)
    pp += writeDefaultEndpointQoS(payload + pp);

    // PID_USER_DATA carrying ROS 2 typehash (required by rmw_fastrtps typed readers)
    pp += writeTypeHashUserData(payload + pp, typeName);

    // PID_SENTINEL
    putLE16(payload + pp, PID_SENTINEL); pp += 2;
    putLE16(payload + pp, 0); pp += 2;

    // Write DATA submessage via SEDP subscriptions writer
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT
    uint32_t hbCount = heartbeatCount > 0 ? heartbeatCount : (uint32_t)sequenceNumber;
    // See note in buildPublicationMessage — lastSN must be the writer's
    // monotonically-increasing high-water mark.
    uint64_t hbLast = hbLastSN > 0 ? hbLastSN : sequenceNumber;
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER,
        ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER,
        0, 1,
        (uint32_t)(hbLast >> 32), (uint32_t)(hbLast & 0xFFFFFFFF),
        hbCount);

    return pos;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Build Participant Message Data (liveliness assertion)
// GuidPrefix + PARTICIPANT_MESSAGE_DATA_WRITER -> DATA + HB
// Payload: participantGuidPrefix(12) + kind(4) + data(4+0)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint32_t SEDPHandler::buildParticipantMessageData(
    uint8_t* pBuf, uint32_t bufLen,
    const RTPSParticipant& participant,
    const uint8_t* destGuidPrefix,
    uint64_t sequenceNumber,
    uint32_t heartbeatCount)
{
    if (!pBuf || bufLen < 120)
        return 0;

    uint32_t pos = 0;

    // RTPS Header
    pos += RTPSMessage::writeHeader(pBuf + pos, bufLen - pos, participant.getGuidPrefix());

    // INFO_DST
    pos += RTPSMessage::writeInfoDST(pBuf + pos, bufLen - pos, destGuidPrefix);

    // INFO_TS
    pos += RTPSMessage::writeInfoTS(pBuf + pos, bufLen - pos, 0, 0);

    // ParticipantMessageData payload:
    // CDR encapsulation (4 bytes) + participantGuidPrefix (12 bytes)
    // + kind (4 bytes: AUTOMATIC_LIVELINESS_UPDATE = {0,0,0,1})
    // + sequence of octets length (4 bytes: 0 = empty)
    uint8_t payload[24];
    uint32_t pp = 0;
    payload[pp++] = 0x00; payload[pp++] = 0x01;  // CDR_LE (plain CDR)
    payload[pp++] = 0x00; payload[pp++] = 0x00;
    memcpy(payload + pp, participant.getGuidPrefix(), 12); pp += 12;
    // kind = AUTOMATIC_LIVELINESS_UPDATE
    payload[pp++] = 0x00; payload[pp++] = 0x00;
    payload[pp++] = 0x00; payload[pp++] = 0x01;
    // data length = 0
    payload[pp++] = 0x00; payload[pp++] = 0x00;
    payload[pp++] = 0x00; payload[pp++] = 0x00;

    // DATA submessage
    pos += RTPSMessage::writeDataSubmessage(pBuf + pos, bufLen - pos,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
        0, (uint32_t)(sequenceNumber & 0xFFFFFFFF),
        payload, pp);

    // HEARTBEAT
    // Advertise only the current sample as available (firstSN == lastSN == sequenceNumber).
    // The liveliness topic is a keep-alive; older samples have no semantic value and we do
    // not cache them for retransmit.  Advertising the full history (firstSN = 1) caused
    // ROS 2 peers to NACK every missed sample (numBits growing to 166 / 256), flooding the
    // metatraffic socket.  firstSN = lastSN tells the peer those samples are GAP'd and it
    // should just advance its base past the current SN.
    const uint32_t liveSnLow = (uint32_t)(sequenceNumber & 0xFFFFFFFF);
    const uint32_t liveSnHigh = (uint32_t)((sequenceNumber >> 32) & 0xFFFFFFFF);
    pos += RTPSMessage::writeHeartbeat(pBuf + pos, bufLen - pos,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER,
        ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER,
        liveSnHigh, liveSnLow,
        liveSnHigh, liveSnLow,
        heartbeatCount);

    return pos;
}
