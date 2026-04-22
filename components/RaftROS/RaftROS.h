/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS - Native ROS 2 Node for Raft Framework
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftSysMod.h"
#include "runtime/core/RTPSParticipant.h"
#include "runtime/discovery/SPDPHandler.h"
#include "runtime/announce/SEDPHandler.h"
#include "runtime/reliability/RTPSReaderStateMap.h"
#include <functional>
#include <vector>

class APISourceInfo;

class RaftROS : public RaftSysMod
{
public:
    RaftROS(const char* pModuleName, RaftJsonIF& sysConfig);
    virtual ~RaftROS();

    // SysMod lifecycle
    void setup() override;
    void loop() override;
    void addRestAPIEndpoints(RestAPIEndpointManager& endpointManager) override;
    String getStatusJSON() const override;

    // Signature: (writerEID[4], srcGuidPrefix[12], text, textLen)
    using StringMessageHandler = std::function<void(const uint8_t*, const uint8_t*, const char*, uint32_t)>;

    // Register a handler invoked when a std_msgs/String arrives on any user-topic writer
    // (except built-in ros_discovery_info). Pass an empty std::function to clear.
    void setStringMessageHandler(StringMessageHandler handler)
    {
        _stringMessageHandler = std::move(handler);
    }

    // Factory for SysMod registration
    static RaftSysMod* create(const char* pModuleName, RaftJsonIF& sysConfig)
    {
        return new RaftROS(pModuleName, sysConfig);
    }

private:
    // Configuration
    bool _isEnabled = false;
    uint32_t _domainId = 0;
    String _nodeName;
    String _nodeNamespace;
    uint32_t _leaseDurationSec = 120;
    uint32_t _spdpIntervalMs = 30000;

    // Protocol handlers (platform-independent)
    RTPSParticipant _participant;
    SPDPHandler _spdpHandler;
    SEDPHandler _sedpHandler;

    // UDP sockets (-1 = not created)
    int _spdpSock = -1;          // Multicast for SPDP (port 7400)
    int _metatrafficSock = -1;   // Unicast for SEDP  (port 7410)
    int _userDataSock = -1;      // Unicast for data   (port 7411)

    // Our IP (network byte order)
    uint32_t _myIpAddr = 0;

    // Connection state
    enum class ConnState { DISCONNECTED, ANNOUNCING, ACTIVE };
    ConnState _connState = ConnState::DISCONNECTED;

    // Sequence numbers and counters
    uint32_t _lastSpdpSendMs = 0;
    uint32_t _lastWriterHbMs = 0;
    static const uint32_t WRITER_HB_INTERVAL_MS = 1000;
    uint64_t _spdpSeqNum = 0;
    uint64_t _sedpSeqNum = 1;     // seq 1 on SEDP pubs writer = ros_disc_info publication
    uint64_t _sedpSubSeqNum = 1;  // always 1 (single SEDP sub, never changes)
    uint64_t _rosDiscSeqNum = 1;  // always 1 (single ros_discovery_info sample)
    uint64_t _livelinessSeqNum = 0; // incremented each liveliness send
    uint32_t _heartbeatCount = 0;
    uint32_t _acknackCount = 0;

    // Per-(remote participant, writer entity id) reader state for reliable delivery bookkeeping.
    RaftRuntime::RTPS::Runtime::Reader::RTPSReaderStateMap _readerStateMap;

    // Optional user-topic std_msgs/String dispatch handler; invoked by recvUserData.
    StringMessageHandler _stringMessageHandler;

    // Chatter topic (Phase 2) - sequence numbers and timing
    uint64_t _chatterSeqNum = 0;       // increments each publish
    uint64_t _chatterSedpSeqNum = 2;   // seq 2 on SEDP pubs writer = chatter publication
    uint32_t _lastChatterSendMs = 0;
    static const uint32_t CHATTER_PUBLISH_INTERVAL_MS = 1000;
    uint32_t _chatterMsgIndex = 0;     // counter for message content

    // Discovered remote participants
    std::vector<DiscoveredParticipant> _discovered;
    static const uint32_t MAX_DISCOVERED = 8;

    // Diagnostic tracking (for detecting silent list/state transitions)
    std::size_t _lastLoggedDiscoveredCount = 0;
    uint32_t _lastDiscoveredHealthLogMs = 0;
    static const uint32_t DISCOVERED_HEALTH_LOG_INTERVAL_MS = 5000;

    // Buffers for UDP I/O
    uint8_t _sendBuf[1024] = {};
    uint8_t _recvBuf[2048] = {};

    // Networking helpers
    uint32_t getLocalIP();
    bool createSockets();
    void closeSockets();
    void sendSPDP();
    void recvSPDP();
    void recvMetatraffic();
    void recvUserData();
    void sendWriterHeartbeats();
    void handleNewParticipant(const DiscoveredParticipant& remote, const struct sockaddr_in& senderAddr);
    void purgeStaleParticipants();
    void processDiscoveredParticipant(DiscoveredParticipant& remote, const struct sockaddr_in& fromAddr);
    void handleAcknack(const uint8_t* srcGuidPrefix, const uint8_t* pContent, uint32_t contentLen,
                       const struct sockaddr_in& fromAddr);

    // Chatter topic helpers
    void publishChatter();
    uint32_t buildChatterPayload(uint8_t* pBuf, uint32_t bufLen, const char* message);

    // Helper to build ros_discovery_info payload with our writer GIDs
    uint32_t buildRosDiscInfoWithGids(uint8_t* pBuf, uint32_t bufLen);

    // REST API handler
    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);

    static constexpr const char* MODULE_PREFIX = "RaftROS";

};
