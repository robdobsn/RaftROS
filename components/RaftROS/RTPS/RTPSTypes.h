/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Types - Constants, entity IDs, parameter IDs, port calculation
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>

// === Entity IDs (DDSI-RTPS §9.3.1) ===
// EntityId_t = 3-byte entityKey + 1-byte entityKind

static const uint8_t ENTITYID_PARTICIPANT[4] = {0x00, 0x00, 0x01, 0xC1};
static const uint8_t ENTITYID_UNKNOWN[4] = {0x00, 0x00, 0x00, 0x00};

// Built-in SPDP endpoints
static const uint8_t ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER[4] = {0x00, 0x01, 0x00, 0xC2};
static const uint8_t ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER[4] = {0x00, 0x01, 0x00, 0xC7};

// Built-in SEDP endpoints
static const uint8_t ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER[4] = {0x00, 0x00, 0x03, 0xC2};
static const uint8_t ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER[4] = {0x00, 0x00, 0x03, 0xC7};
static const uint8_t ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER[4] = {0x00, 0x00, 0x04, 0xC2};
static const uint8_t ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER[4] = {0x00, 0x00, 0x04, 0xC7};

// User-defined writer for ros_discovery_info topic
static const uint8_t ENTITYID_ROS_DISC_INFO_WRITER[4] = {0x00, 0x00, 0x01, 0x03};
static const uint8_t ENTITYID_ROS_DISC_INFO_READER[4] = {0x00, 0x00, 0x02, 0x04};

// User-defined writer/reader for /chatter topic (std_msgs/String, NO_KEY)
static const uint8_t ENTITYID_CHATTER_WRITER[4] = {0x00, 0x01, 0x01, 0x03};

// User-defined reader for /chatter_in topic (std_msgs/String, NO_KEY).
// Mirrors the chatter writer entityKey but with the reader entityKind (0x04 = no-key).
static const uint8_t ENTITYID_CHATTER_READER[4] = {0x00, 0x01, 0x02, 0x04};

// Participant Message Data (liveliness)
static const uint8_t ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER[4] = {0x00, 0x02, 0x00, 0xC2};
static const uint8_t ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER[4] = {0x00, 0x02, 0x00, 0xC7};

// === Parameter IDs (DDSI-RTPS §9.6.2) ===
enum RTPSParameterId : uint16_t
{
    PID_PAD                           = 0x0000,
    PID_SENTINEL                      = 0x0001,
    PID_PARTICIPANT_LEASE_DURATION    = 0x0002,
    PID_TOPIC_NAME                    = 0x0005,
    PID_TYPE_NAME                     = 0x0007,
    PID_TIME_BASED_FILTER             = 0x0004,
    PID_PROTOCOL_VERSION              = 0x0015,
    PID_VENDORID                      = 0x0016,
    PID_RELIABILITY                   = 0x001A,
    PID_LIVELINESS                    = 0x001B,
    PID_DURABILITY                    = 0x001D,
    PID_DURABILITY_SERVICE            = 0x001E,
    PID_OWNERSHIP                     = 0x001F,
    PID_PRESENTATION                  = 0x0021,
    PID_DEADLINE                      = 0x0023,
    PID_DESTINATION_ORDER             = 0x0025,
    PID_LATENCY_BUDGET                = 0x0027,
    PID_PARTITION                     = 0x0029,
    PID_LIFESPAN                      = 0x002B,
    PID_UNICAST_LOCATOR               = 0x002F,
    PID_USER_DATA                     = 0x002C,
    PID_GROUP_DATA                    = 0x002D,
    PID_TOPIC_DATA                    = 0x002E,
    PID_TYPE_MAX_SIZE_SERIALIZED      = 0x0060,
    PID_DEFAULT_UNICAST_LOCATOR       = 0x0031,
    PID_METATRAFFIC_UNICAST_LOCATOR   = 0x0032,
    PID_METATRAFFIC_MULTICAST_LOCATOR = 0x0033,
    PID_DEFAULT_MULTICAST_LOCATOR     = 0x0048,
    PID_PARTICIPANT_GUID              = 0x0050,
    PID_BUILTIN_ENDPOINT_SET          = 0x0058,
    PID_PROPERTY_LIST                 = 0x0059,
    PID_ENDPOINT_GUID                 = 0x005A,
    PID_ENTITY_NAME                   = 0x0062,
    PID_KEY_HASH                      = 0x0070,
    PID_STATUS_INFO                   = 0x0071,
    PID_DATA_REPRESENTATION           = 0x0073,
    PID_TYPE_CONSISTENCY              = 0x0074,
};

// === Data Representation IDs (OMG DDS-XTypes §7.6.3.1.2) ===
//   Value advertised in PID_DATA_REPRESENTATION to negotiate CDR wire format
//   between matching endpoints. ROS 2 Jazzy (Fast DDS 3.x) requires readers
//   and writers to agree on a representation; absence of PID_DATA_REPRESENTATION
//   on a writer has been observed to cause Fast CDR deserialization failures
//   on `@appendable` types (ROS 2 IDL default).
static const int16_t DATA_REPRESENTATION_XCDR1  = 0;
static const int16_t DATA_REPRESENTATION_XML    = 1;
static const int16_t DATA_REPRESENTATION_XCDR2  = 2;

// === Port Calculation (DDSI-RTPS §9.6.1) ===
static const uint16_t RTPS_PB = 7400;
static const uint16_t RTPS_DG = 250;
static const uint16_t RTPS_PG = 2;
static const uint16_t RTPS_D0 = 0;
static const uint16_t RTPS_D1 = 10;
static const uint16_t RTPS_D2 = 1;
static const uint16_t RTPS_D3 = 11;

inline uint16_t rtpsDiscoveryMulticastPort(uint32_t domainId) {
    return RTPS_PB + RTPS_DG * domainId + RTPS_D0;
}
inline uint16_t rtpsDiscoveryUnicastPort(uint32_t domainId, uint32_t participantId) {
    return RTPS_PB + RTPS_DG * domainId + RTPS_D1 + RTPS_PG * participantId;
}
inline uint16_t rtpsUserUnicastPort(uint32_t domainId, uint32_t participantId) {
    return RTPS_PB + RTPS_DG * domainId + RTPS_D3 + RTPS_PG * participantId;
}

// === Multicast Address ===
inline const char* RTPS_DEFAULT_MULTICAST_ADDR = "239.255.0.1";

// === Built-in Endpoint Set Bits ===
static const uint32_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER       = (1u << 0);
static const uint32_t DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR        = (1u << 1);
static const uint32_t DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER      = (1u << 2);
static const uint32_t DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR       = (1u << 3);
static const uint32_t DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER     = (1u << 4);
static const uint32_t DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR      = (1u << 5);
static const uint32_t BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER   = (1u << 10);
static const uint32_t BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER   = (1u << 11);

// Standard set for a minimal participant (all SEDP endpoints)
static const uint32_t BUILTIN_ENDPOINT_SET_DEFAULT =
    DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER |
    DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR |
    DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER |
    DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR |
    DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER |
    DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR;

// === Locator Kind ===
static const int32_t LOCATOR_KIND_UDPv4 = 1;

// === Reliability / Durability QoS kinds for SEDP ===
static const int32_t RELIABILITY_BEST_EFFORT = 1;
static const int32_t RELIABILITY_RELIABLE = 2;
static const int32_t DURABILITY_VOLATILE = 0;
static const int32_t DURABILITY_TRANSIENT_LOCAL = 1;

// === RTPS Header Size ===
static const uint32_t RTPS_HEADER_SIZE = 20;

// === ROS 2 Discovery Info Topic ===
inline const char* ROS_DISCOVERY_INFO_TOPIC = "ros_discovery_info";
inline const char* ROS_DISCOVERY_INFO_TYPE = "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_";

// === ROS 2 /chatter Topic (Phase 2) ===
inline const char* CHATTER_DDS_TOPIC = "rt/chatter";
inline const char* CHATTER_DDS_TYPE = "std_msgs::msg::dds_::String_";

// Inbound user-data topic subscribed by this participant.
inline const char* CHATTER_IN_DDS_TOPIC = "rt/chatter_in";
inline const char* CHATTER_IN_DDS_TYPE = "std_msgs::msg::dds_::String_";
