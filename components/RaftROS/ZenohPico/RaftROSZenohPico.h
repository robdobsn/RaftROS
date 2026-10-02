/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS (zenoh-pico build) - the Zenoh ROS 2 node with Eclipse zenoh-pico as the session
//
// Selected by CONFIG_RAFTROS_BACKEND_ZENOH_PICO (experimental).  Applications
// include RaftROS.h, not this header; the public API is the same as the other
// builds, so the example application builds unchanged.
//
// What is shared with Raft's own Zenoh build: everything that makes the device
// a ROS 2 node - key expressions, liveliness tokens, type hashes, attachments
// (ZenohROSCodec), CDR, DeviceManager auto-publishing, the service registry and
// the parameter store.  What zenoh-pico replaces: the session - TCP, the
// INIT/OPEN handshake, framing, keep-alives, interests and reconnection.
//
// Threading, which is the main structural difference: on ESP-IDF zenoh-pico's
// client socket blocks, so the library cannot be stepped from the Raft loop.
// It runs as designed instead - its own FreeRTOS task receives and keeps the
// lease, a short-lived helper task opens the session (its connect blocks), and
// samples and service requests are copied into small queues that the loop
// drains.  Declarations, publishing and replies stay on the loop.
//
// No zenoh-pico type appears here: the application's component is not built
// with zenoh-pico's platform defines, so the library is held behind PicoState.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftROSBackendSelect.h"
#if !RAFTROS_BACKEND_ZENOH_PICO
    #error "RaftROSZenohPico.h is the zenoh-pico build's SysMod - include RaftROS.h, which selects the backend"
#endif

#include "RaftSysMod.h"
#include "AutoPub/AutoPubDeviceSource.h"
#include "AutoPub/AutoPubStringMessage.h"
#include "AutoPub/AutoPubServiceRegistry.h"
#include "AutoPub/AutoPubParameterStore.h"
#include <array>
#include <atomic>
#include <functional>
#include <memory>

class APISourceInfo;

namespace RaftRuntime::ZenohPico
{
class ZenohPicoAutoPubBackend;
struct PicoState;
static constexpr uint8_t ZENOH_PICO_AUTOPUB_CAPACITY = 16;
}

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

    /// @brief Called when a std_msgs/String arrives on a subscribed topic -
    /// (writerEID[4], publisherGuid[12], text, textLen), as on the other builds
    using StringMessageHandler = std::function<void(const uint8_t*, const uint8_t*, const char*, uint32_t)>;

    /// @brief Subscribe to a std_msgs/String topic ("/chatter_in" or "rt/chatter_in")
    /// @return the slot index, or -1 if the table is full or the type has no known hash
    int addStringSubscription(const char* topic, const char* type, StringMessageHandler handler = {});
    void setStringSubscription(const char* topic, const char* type, StringMessageHandler handler)
    {
        _defaultHandler = std::move(handler);
        addStringSubscription(topic, type, _defaultHandler);
    }
    void setStringMessageHandler(StringMessageHandler handler) { _defaultHandler = std::move(handler); }

    // ---- Services (server side) - see AutoPubServiceRegistry ----
    using ServiceHandler = RaftRuntime::AutoPub::AutoPubServiceHandler;
    using ServiceRequest = RaftRuntime::AutoPub::AutoPubServiceRequest;
    using ServiceReply = RaftRuntime::AutoPub::AutoPubServiceReply;
    using ServiceOutcome = RaftRuntime::AutoPub::AutoPubServiceOutcome;
    int addService(const char* name, const char* type, ServiceHandler handler);
    bool completeService(uint32_t token, const ServiceReply& reply) { return _services.complete(token, reply); }

    // ---- Parameters ----
    using ParamValue = RaftRuntime::AutoPub::AutoPubParamValue;
    using ParamType = RaftRuntime::AutoPub::AutoPubParamType;
    using ParamSetCallback = RaftRuntime::AutoPub::AutoPubParamSetCallback;
    bool declareParameter(const char* name, const ParamValue& value, const char* description = "",
                          bool readOnly = false, ParamSetCallback onSet = {})
    {
        return _params.declare(name, value, description, readOnly, std::move(onSet)) != decltype(_params)::INVALID_SLOT;
    }
    bool declareParameter(const char* name, bool value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::Bool; v.boolValue = value;
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    bool declareParameter(const char* name, int64_t value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::Integer; v.integerValue = value;
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    bool declareParameter(const char* name, double value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::Double; v.doubleValue = value;
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    bool declareParameter(const char* name, const char* value, const char* description = "", bool readOnly = false, ParamSetCallback onSet = {})
    {
        ParamValue v; v.type = ParamType::String;
        if (!value || strlen(value) >= sizeof(v.stringValue))
            return false;
        strcpy(v.stringValue, value);
        return declareParameter(name, v, description, readOnly, std::move(onSet));
    }
    const ParamValue* parameter(const char* name) const { return _params.value(name); }

    void setChatterEnabled(bool enabled)
    {
        _chatterEnabled = enabled;
        ParamValue v; v.type = ParamType::Bool; v.boolValue = enabled;
        _params.setLocal("chatterEnable", v);
    }
    bool isChatterEnabled() const { return _chatterEnabled; }
    uint8_t attachedDeviceCount() const { return _autoPubSource.attachedCount(); }

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
    String _routerHost;
    uint32_t _routerPort = 7447;
    bool _routerFromConfig = false;

    /// @brief DISCONNECTED -> OPENING (helper task in z_open) -> READY.  While
    /// READY zenoh-pico reconnects by itself and re-declares what it holds.
    enum class ConnState : uint8_t { DISCONNECTED, OPENING, READY };
    ConnState _connState = ConnState::DISCONNECTED;
    uint32_t _lastConnectAttemptMs = 0;
    uint32_t _reconnectDelayMs = 1000;
    static const uint32_t RECONNECT_DELAY_MIN_MS = 1000;
    static const uint32_t RECONNECT_DELAY_MAX_MS = 30000;
    uint32_t _connectFailures = 0;
    uint32_t _lastSessionMs = 0;
    uint32_t _sessionCount = 0;
    int _lastOpenResult = 0;
    uint32_t _lastOpenMs = 0;           ///< How long the last z_open took (helper task, not the loop)

    // zenoh-pico session, queues and the helper task - defined in the .cpp
    std::unique_ptr<RaftRuntime::ZenohPico::PicoState> _pico;

    // Identity
    std::array<uint8_t, 16> _identity{};
    char _sessionIdStr[33] = {};
    char _nodeToken[448] = {};
    bool _nodeTokenDeclared = false;
    bool buildNodeIdentity();

    // Auto-publish
    std::unique_ptr<RaftRuntime::ZenohPico::ZenohPicoAutoPubBackend> _autoPubBackend;
    RaftRuntime::AutoPub::AutoPubDeviceSource<
        RaftRuntime::ZenohPico::ZenohPicoAutoPubBackend,
        RaftRuntime::ZenohPico::ZENOH_PICO_AUTOPUB_CAPACITY> _autoPubSource;

    // /chatter
    bool _chatterEnabled = true;
    uint8_t _chatterSlot = 0xFF;
    uint32_t _lastChatterSendMs = 0;
    uint32_t _chatterMsgIndex = 0;
    uint32_t _chatterPeriodMs = 1000;
    void publishChatter(uint32_t nowMs);

    // Subscriptions
    struct Subscription
    {
        bool inUse = false;
        bool declared = false;
        char rosTopic[RaftRuntime::AutoPub::AUTOPUB_TOPIC_MAX_LEN] = {};
        char type[RaftRuntime::AutoPub::AUTOPUB_TYPE_MAX_LEN] = {};
        char key[256] = {};
        const char* typeHash = nullptr;
        uint64_t entityId = 0;
        uint32_t received = 0;
        StringMessageHandler handler;
    };
    static const uint8_t MAX_SUBSCRIPTIONS = 4;
    Subscription _subscriptions[MAX_SUBSCRIPTIONS];
    uint8_t _subscriptionCount = 0;
    uint64_t _nextSubscriptionEntityId = 1000;
    StringMessageHandler _defaultHandler;
    uint32_t _samplesDropped = 0;

    // Services
    struct ServiceSlot
    {
        bool inUse = false;
        bool declared = false;
        char key[256] = {};
        const char* typeHash = nullptr;
        char wireType[64] = {};
        uint64_t entityId = 0;
    };
    static const uint8_t MAX_SERVICES = 12;
    static const uint8_t SERVICE_INFLIGHT = 4;
    static const uint8_t SERVICE_DISPATCH_BUDGET = 2;
    ServiceSlot _serviceSlots[MAX_SERVICES];
    RaftRuntime::AutoPub::AutoPubServiceRegistry<MAX_SERVICES, SERVICE_INFLIGHT, 1024, 512> _services;
    uint64_t _nextServiceEntityId = 2000;
    uint32_t _requestsDropped = 0;
    uint32_t _nextRequestId = 1;

    // Parameters
    static const uint8_t MAX_PARAMETERS = 16;
    RaftRuntime::AutoPub::AutoPubParameterStore<MAX_PARAMETERS> _params;
    uint8_t _paramReplyBuf[1024] = {};
    RaftJsonIF& _sysConfig;
    char _routerHostPending[16] = {};
    void setupParameters();
    bool persistRouterHost(const char* host);
    void applyPendingRouterHost();

    // Loop
    void startOpen(uint32_t nowMs);
    void checkOpen(uint32_t nowMs);
    void closeSession(const char* reason);
    bool stepDeclarations();
    void drainSamples();
    void drainRequests(uint32_t nowMs);
    bool sendServiceReply();
    uint32_t getLocalIP();

    // Loop-budget diagnostics, named as on the other builds where they mean the same
    uint32_t _loopPassMaxUs = 0;
    uint32_t _loopDrainMaxUs = 0;       ///< device sample drain (publishing)
    uint32_t _loopConnMaxUs = 0;        ///< starting the open task / taking its result
    uint32_t _loopRxMaxUs = 0;          ///< draining the sample and request queues, handlers included
    uint32_t _loopTxMaxUs = 0;          ///< declarations, replies, /chatter
    uint32_t _loopDeclMaxUs = 0;        ///< worst single declaration (zenoh-pico declare + token)
    uint32_t _loopPutMaxUs = 0;         ///< worst single publish
    uint32_t _loopReplyMaxUs = 0;       ///< worst single service reply

    RaftRetCode apiStatus(const String& reqStr, String& respStr, const APISourceInfo& sourceInfo);
};
