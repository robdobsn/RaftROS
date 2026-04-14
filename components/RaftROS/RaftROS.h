/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftSysMod.h"
#include "RTPSParticipant.h"
#include "SPDPHandler.h"
#include "SEDPHandler.h"
#include "CDREncoder.h"
#include <map>

class CommsCoreIF;

class RaftROS : public RaftSysMod
{
public:
    RaftROS(const char* pModuleName, RaftJsonIF& sysConfig);
    virtual ~RaftROS();

    // SysMod lifecycle
    void setup() override;
    void loop() override;
    void addCommsChannels(CommsCoreIF& commsCoreIF) override;
    void postSetup() override;

    // REST API and status
    void addRestAPIEndpoints(RestAPIEndpointManager& endpointManager) override;
    String getStatusJSON() const override;

private:
    // Configuration
    bool _isEnabled = false;
    uint32_t _domainId = 0;
    String _nodeName;

    // RTPS participant and discovery
    RTPSParticipant _participant;
    SPDPHandler _spdp;
    SEDPHandler _sedp;

    // CDR serialization
    CDREncoder _cdrEncoder;

    // Channel integration
    std::map<String, uint32_t> _topicChannelIDs;

    // Connection state
    enum class ConnState { DISCONNECTED, MULTICAST_JOIN, SPDP_ANNOUNCING, ACTIVE };
    ConnState _connState = ConnState::DISCONNECTED;

    // Helpers
    bool sendRTPSMsg(const String& topicName, CommsChannelMsg& msg);
    bool readyToSend(uint32_t channelID, CommsMsgTypeCode msgType, bool& noConn);

    // API handler
    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);
};
