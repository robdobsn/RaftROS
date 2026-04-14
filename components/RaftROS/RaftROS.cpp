/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftROS.h"
#include "CommsCoreIF.h"
#include "RestAPIEndpointManager.h"

static const char* MODULE_PREFIX = "RaftROS";

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor / Destructor
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftROS::RaftROS(const char* pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
}

RaftROS::~RaftROS()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::setup()
{
    // Get configuration
    _isEnabled = configGetBool("enable", false);
    _domainId = configGetLong("domainId", 0);
    _nodeName = configGetString("nodeName", "raft_esp32");

    if (!_isEnabled)
    {
        LOG_I(MODULE_PREFIX, "setup DISABLED");
        return;
    }

    LOG_I(MODULE_PREFIX, "setup domainId %d nodeName %s", (int)_domainId, _nodeName.c_str());

    // Initialize RTPS participant
    _participant.init(_domainId, _nodeName);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::loop()
{
    if (!_isEnabled)
        return;

    // Service RTPS connection state machine
    switch (_connState)
    {
        case ConnState::DISCONNECTED:
        {
            // TODO: Join multicast group on well-known SPDP port
            break;
        }
        case ConnState::MULTICAST_JOIN:
        {
            // TODO: Start sending SPDP announcements
            break;
        }
        case ConnState::SPDP_ANNOUNCING:
        {
            // TODO: Process SEDP endpoint matching
            break;
        }
        case ConnState::ACTIVE:
        {
            // TODO: Handle data exchange, periodic SPDP keepalive
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Add comms channels
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::addCommsChannels(CommsCoreIF& commsCoreIF)
{
    if (!_isEnabled)
        return;

    // TODO: Register CommsChannels for each configured ROS topic
    // Following MQTTManager pattern:
    // - For each outbound topic, register a channel with sendRTPSMsg callback
    // - Store channelID in _topicChannelIDs
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Post setup - wire StatePublisher subscriptions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::postSetup()
{
    if (!_isEnabled)
        return;

    // TODO: Get StatePublisher via getSysManager()->getSysMod("Publish")
    // For each configured pubSource:
    //   createSubscription(pubTopic, channelID, rateHz, trigger)
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// REST API endpoints
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::addRestAPIEndpoints(RestAPIEndpointManager& endpointManager)
{
    endpointManager.addEndpoint("rosstat", RestAPIEndpoint::EndpointType::ENDPOINT_CALLBACK,
                                RestAPIEndpoint::EndpointMethod::ENDPOINT_GET,
                                std::bind(&RaftROS::apiStatus, this,
                                          std::placeholders::_1, std::placeholders::_2,
                                          std::placeholders::_3),
                                "Get RaftROS status");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Status JSON
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

String RaftROS::getStatusJSON() const
{
    String connStateStr;
    switch (_connState)
    {
        case ConnState::DISCONNECTED: connStateStr = "disconnected"; break;
        case ConnState::MULTICAST_JOIN: connStateStr = "joining"; break;
        case ConnState::SPDP_ANNOUNCING: connStateStr = "announcing"; break;
        case ConnState::ACTIVE: connStateStr = "active"; break;
    }
    char statusStr[200];
    snprintf(statusStr, sizeof(statusStr),
             R"({"rslt":"ok","en":%s,"domId":%d,"node":"%s","conn":"%s"})",
             _isEnabled ? "true" : "false",
             (int)_domainId,
             _nodeName.c_str(),
             connStateStr.c_str());
    return statusStr;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// API handler
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftRetCode RaftROS::apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo)
{
    respStr = getStatusJSON();
    return RaftRetCode::RAFT_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Send RTPS message (CommsChannel callback)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RaftROS::sendRTPSMsg(const String& topicName, CommsChannelMsg& msg)
{
    // TODO: Convert CommsChannelMsg payload to CDR, wrap in RTPS DATA submessage, send via UDP
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ready to send (CommsChannel callback)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool RaftROS::readyToSend(uint32_t channelID, CommsMsgTypeCode msgType, bool& noConn)
{
    noConn = (_connState != ConnState::ACTIVE);
    return (_connState == ConnState::ACTIVE);
}
