/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RaftROS (zenoh-pico build) - see RaftROSZenohPico.h
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftROSZenohPico.h"
#include "RaftJson.h"
#include "RestAPIEndpointManager.h"
#include "RaftUtils.h"
#include "CDREncoder.h"
#include "AutoPub/AutoPubClassMap.h"
#include "AutoPub/AutoPubEndpointDesc.h"
#include "AutoPub/AutoPubQoSProfile.h"
#include "AutoPub/AutoPubSampleRunner.h"
#include "AutoPub/AutoPubServiceCodec.h"
#include "AutoPub/AutoPubDeviceSource.hpp"
#include "Zenoh/ZenohROSCodec.h"
#include "Zenoh/ZenohROSIdentity.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <arpa/inet.h>

// zenoh-pico declares its C API with C linkage itself, and adds C++
// overloads of z_loan/z_move/z_drop that an extern "C" wrapper would break
// Its headers use C-style partial initialisers, which this component's C++
// warnings reject; the exemption covers zenoh-pico's headers only
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "zenoh-pico.h"
#pragma GCC diagnostic pop

static const char* MODULE_PREFIX = "RaftROS";

using RaftRuntime::Zenoh::ZenohROSCodec;
using RaftRuntime::Zenoh::ZenohROSIdentity;

namespace RaftRuntime::ZenohPico
{

static constexpr size_t KEY_MAX = 256;
static constexpr size_t TOKEN_MAX = 448;
static constexpr uint32_t SAMPLE_PAYLOAD_MAX = 256;     ///< A std_msgs/String the handlers can take
static constexpr uint32_t REQUEST_PAYLOAD_MAX = 512;    ///< The service registry's Raw request limit
static constexpr uint8_t QUEUE_DEPTH = 4;
static constexpr uint32_t OPEN_TASK_STACK = 6144;
#ifdef CONFIG_RAFTROS_ZENOH_PICO_TUNED
static constexpr UBaseType_t HELPER_PRIORITY = 1;      ///< Below everything that matters
static constexpr BaseType_t HELPER_CORE = 0;            ///< Off the main loop's core
#else
static constexpr UBaseType_t HELPER_PRIORITY = 5;
static constexpr BaseType_t HELPER_CORE = tskNO_AFFINITY;
#endif

/// @brief A sample, copied out of zenoh-pico's task for the loop
struct SampleItem
{
    uint8_t subscription = 0;
    uint8_t attachmentLen = 0;
    uint16_t payloadLen = 0;
    uint8_t attachment[ZenohROSCodec::ATTACHMENT_SIZE] = {};
    uint8_t payload[SAMPLE_PAYLOAD_MAX] = {};
};

/// @brief A service request: the query is moved (not copied) into the item,
/// so whoever takes the item owns the query and must drop it - which is what
/// sends the RESPONSE_FINAL
struct RequestItem
{
    uint8_t service = 0;
    uint8_t attachmentLen = 0;
    uint16_t payloadLen = 0;
    z_owned_query_t query;
    uint8_t attachment[ZenohROSCodec::ATTACHMENT_SIZE] = {};
    uint8_t payload[REQUEST_PAYLOAD_MAX] = {};
};

/// @brief Everything belonging to one zenoh-pico session.  Replaced as a whole
/// when a session is closed, and handed to a task to tear down, because
/// closing a session joins zenoh-pico's task and that can wait on its read.
struct Live
{
    z_owned_session_t session;
    z_owned_liveliness_token_t nodeToken;
    bool nodeTokenValid = false;
    struct Sub { z_owned_subscriber_t sub; z_owned_liveliness_token_t token; bool valid = false; };
    Sub subs[4];
    struct Svc { z_owned_queryable_t queryable; z_owned_liveliness_token_t token; bool valid = false; };
    Svc svcs[12];
    Live()
    {
        z_internal_null(&session);
        z_internal_null(&nodeToken);
        for (auto& s : subs) { z_internal_null(&s.sub); z_internal_null(&s.token); }
        for (auto& s : svcs) { z_internal_null(&s.queryable); z_internal_null(&s.token); }
    }
    ~Live()
    {
        // Entities before the session that holds them
        for (auto& s : subs)
            if (s.valid) { z_drop(z_move(s.token)); z_drop(z_move(s.sub)); }
        for (auto& s : svcs)
            if (s.valid) { z_drop(z_move(s.token)); z_drop(z_move(s.queryable)); }
        if (nodeTokenValid)
            z_drop(z_move(nodeToken));
        z_drop(z_move(session));
    }
};

struct CallbackCtx
{
    struct PicoState* state = nullptr;
    uint8_t index = 0;
};

struct PicoState
{
    Live* live = nullptr;                   ///< Owned; nullptr when there is no session
    QueueHandle_t sampleQueue = nullptr;
    QueueHandle_t requestQueue = nullptr;
    CallbackCtx subCtx[4];
    CallbackCtx svcCtx[12];
    std::atomic<uint32_t> samplesQueueFull{0};
    std::atomic<uint32_t> samplesOversized{0};
    std::atomic<uint32_t> requestsQueueFull{0};
    std::atomic<uint32_t> requestsOversized{0};

    /// @brief A session is being torn down by closeTask.  No new session may
    /// open until it has gone: the next one reuses the same session id, and a
    /// router that still holds the old session drops the new one's transport -
    /// which zenoh-pico 1.10.1 then dereferences on the next declaration.
    std::atomic<bool> closing{false};

    // The open task
    enum OpenState : int { OPEN_IDLE = 0, OPEN_RUNNING, OPEN_DONE_OK, OPEN_DONE_FAIL };
    std::atomic<int> openState{OPEN_IDLE};
    Live* openLive = nullptr;
    int openResult = 0;
    uint32_t openMs = 0;
    char locator[48] = {};
    char zid[33] = {};

    // Queries the registry is holding, by request id
    struct Inflight { uint32_t requestId = 0; uint32_t sinceMs = 0; bool used = false; z_owned_query_t query; };
    Inflight inflight[8];
    PicoState()
    {
        for (auto& f : inflight)
            z_internal_null(&f.query);
    }
};

// ---- zenoh-pico task side: copy out and queue, never block ----

static void onSample(z_loaned_sample_t* sample, void* context)
{
    auto* ctx = static_cast<CallbackCtx*>(context);
    SampleItem item;
    item.subscription = ctx->index;
    const z_loaned_bytes_t* payload = z_sample_payload(sample);
    const size_t len = payload ? z_bytes_len(payload) : 0;
    if (len > SAMPLE_PAYLOAD_MAX)
    {
        ++ctx->state->samplesOversized;
        return;
    }
    if (payload)
    {
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        item.payloadLen = (uint16_t)z_bytes_reader_read(&reader, item.payload, len);
    }
    const z_loaned_bytes_t* attachment = z_sample_attachment(sample);
    if (attachment && z_bytes_len(attachment) == ZenohROSCodec::ATTACHMENT_SIZE)
    {
        z_bytes_reader_t reader = z_bytes_get_reader(attachment);
        item.attachmentLen = (uint8_t)z_bytes_reader_read(&reader, item.attachment, ZenohROSCodec::ATTACHMENT_SIZE);
    }
    if (xQueueSend(ctx->state->sampleQueue, &item, 0) != pdTRUE)
        ++ctx->state->samplesQueueFull;
}

static void onQuery(z_loaned_query_t* query, void* context)
{
    auto* ctx = static_cast<CallbackCtx*>(context);
    RequestItem item;
    item.service = ctx->index;
    z_internal_null(&item.query);
    const z_loaned_bytes_t* payload = z_query_payload(query);
    const size_t len = payload ? z_bytes_len(payload) : 0;
    if (len > REQUEST_PAYLOAD_MAX)
    {
        // Dropping the loaned query without replying sends the final: the
        // client hears "no reply" at once rather than waiting out its timeout
        ++ctx->state->requestsOversized;
        return;
    }
    if (payload)
    {
        z_bytes_reader_t reader = z_bytes_get_reader(payload);
        item.payloadLen = (uint16_t)z_bytes_reader_read(&reader, item.payload, len);
    }
    const z_loaned_bytes_t* attachment = z_query_attachment(query);
    if (attachment && z_bytes_len(attachment) == ZenohROSCodec::ATTACHMENT_SIZE)
    {
        z_bytes_reader_t reader = z_bytes_get_reader(attachment);
        item.attachmentLen = (uint8_t)z_bytes_reader_read(&reader, item.attachment, ZenohROSCodec::ATTACHMENT_SIZE);
    }
    if (z_query_clone(&item.query, query) != Z_OK)
        return;
    if (xQueueSend(ctx->state->requestQueue, &item, 0) != pdTRUE)
    {
        ++ctx->state->requestsQueueFull;
        z_drop(z_move(item.query));
    }
}

/// @brief Open a session off the loop: z_open connects with a blocking socket
static void openTask(void* arg)
{
    auto* state = static_cast<PicoState*>(arg);
    const int64_t startUs = esp_timer_get_time();
    Live* live = new Live();
    z_owned_config_t config;
    int rslt = z_config_default(&config);
    if (rslt == Z_OK)
    {
        zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, "client");
        zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, state->locator);
        zp_config_insert(z_loan_mut(config), Z_CONFIG_SESSION_ZID_KEY, state->zid);
        z_open_options_t options;
        z_open_options_default(&options);
#ifdef CONFIG_RAFTROS_ZENOH_PICO_TUNED
        // zenoh-pico's executor defaults to priority 12, unpinned: it can run on
        // the loop's core and preempt it whenever data arrives.  Run it at the
        // main loop's priority instead (its attributes cannot pin a core).
        static z_task_attr_t executorAttr = {"zpExec", 1, 5120, false, nullptr, nullptr};
        options.executor_task_attributes = &executorAttr;
#endif
        rslt = z_open(&live->session, z_move(config), &options);
    }
    state->openResult = rslt;
    state->openMs = (uint32_t)((esp_timer_get_time() - startUs) / 1000);
    if (rslt == Z_OK)
    {
        state->openLive = live;
        state->openState = PicoState::OPEN_DONE_OK;
    }
    else
    {
        delete live;
        state->openState = PicoState::OPEN_DONE_FAIL;
    }
    vTaskDelete(nullptr);
}

/// @brief Tear a session down off the loop (it joins zenoh-pico's task)
struct CloseJob { Live* live; PicoState* state; };
static void closeTask(void* arg)
{
    auto* job = static_cast<CloseJob*>(arg);
    delete job->live;
    job->state->closing = false;
    delete job;
    vTaskDelete(nullptr);
}

static bool toBytes(z_owned_bytes_t& out, const uint8_t* data, size_t len)
{
    return z_bytes_copy_from_buf(&out, data, len) == Z_OK;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Auto-publish backend: the same contract as ZenohAutoPubBackend, over zenoh-pico publishers
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ZenohPicoAutoPubBackend
{
public:
    static constexpr uint8_t INVALID_SLOT = 0xFF;

    struct Deps
    {
        uint32_t domainId = 0;
        const char* sessionId = nullptr;
        const char* nodeNamespace = "/";
        const char* nodeName = nullptr;
    };

    ZenohPicoAutoPubBackend()
    {
        for (auto& e : _entries)
        {
            z_internal_null(&e.pub);
            z_internal_null(&e.token);
        }
    }

    void setup(const Deps& deps) { _deps = deps; }

    /// @brief Bind to a session (nullptr when there is none).  A new session
    /// starts with nothing declared, so every live endpoint is staged again.
    void setLive(Live* live)
    {
        _live = live;
        for (auto& e : _entries)
        {
            if (e.state == Entry::State::Declared)
            {
                // The handles belonged to the old session, which drops them
                z_internal_null(&e.pub);
                z_internal_null(&e.token);
                e.state = Entry::State::PendingDeclare;
            }
            else if (e.state == Entry::State::PendingUndeclare)
                release(e);
        }
    }

    uint8_t createPublisher(const RaftRuntime::AutoPub::AutoPubEndpointDesc& desc)
    {
        if (!_deps.sessionId || !desc.isValid())
            return INVALID_SLOT;
        const char* typeHash = RaftRuntime::AutoPub::AutoPubClassMap_typeHash(desc.msgKind);
        if (!typeHash)
            return INVALID_SLOT;
        for (uint8_t slot = 0; slot < ZENOH_PICO_AUTOPUB_CAPACITY; ++slot)
        {
            Entry& e = _entries[slot];
            if (e.state != Entry::State::Free)
                continue;
            const ZenohROSCodec::NodeIdentity node{_deps.domainId, _deps.sessionId, 1, "/", _deps.nodeNamespace, _deps.nodeName};
            const ZenohROSCodec::Endpoint endpoint{_nextEntityId, ZenohROSCodec::EndpointKind::Publisher,
                desc.topic, desc.type, typeHash, qosForProfile(desc.qosProfileId)};
            if (!ZenohROSCodec::formatTopicKey(e.key, sizeof(e.key), node.domainId, endpoint.topic,
                                               endpoint.wireType, endpoint.typeHash) ||
                !ZenohROSCodec::formatEndpointToken(e.tokenKey, sizeof(e.tokenKey), node, endpoint) ||
                !ZenohROSIdentity::deriveEndpointGid(node, endpoint, e.gid))
                return INVALID_SLOT;
            ++_nextEntityId;
            e.sequence = 0;
            e.state = Entry::State::PendingDeclare;
            return slot;
        }
        return INVALID_SLOT;
    }

    void destroyPublisher(uint8_t slot)
    {
        if (slot >= ZENOH_PICO_AUTOPUB_CAPACITY)
            return;
        Entry& e = _entries[slot];
        if (e.state == Entry::State::Declared)
            e.state = Entry::State::PendingUndeclare;
        else if (e.state == Entry::State::PendingDeclare)
            release(e);
    }

    RaftRuntime::AutoPub::AutoPubPublishResult publish(uint8_t slot, const uint8_t* payload, uint32_t length,
                                                       uint64_t* outSequence = nullptr, uint32_t* outPeersSent = nullptr)
    {
        using RaftRuntime::AutoPub::AutoPubPublishResult;
        if (outSequence) *outSequence = 0;
        if (outPeersSent) *outPeersSent = 0;
        if (slot >= ZENOH_PICO_AUTOPUB_CAPACITY || !payload || length == 0)
            return AutoPubPublishResult::InvalidHandle;
        Entry& e = _entries[slot];
        if (e.state == Entry::State::Free)
            return AutoPubPublishResult::InvalidHandle;
        if (!_live)
            return AutoPubPublishResult::Disconnected;
        if (e.state != Entry::State::Declared)
            return AutoPubPublishResult::QueueFull;

        uint8_t attachment[ZenohROSCodec::ATTACHMENT_SIZE];
        const ZenohROSCodec::Attachment metadata{(int64_t)(e.sequence + 1), (int64_t)_nowMs * 1000000, e.gid};
        if (!ZenohROSCodec::encodeAttachment(attachment, sizeof(attachment), metadata))
            return AutoPubPublishResult::SendFailed;
        const int64_t startUs = esp_timer_get_time();
        z_owned_bytes_t bytes, att;
        if (!toBytes(bytes, payload, length))
            return AutoPubPublishResult::SendFailed;
        if (!toBytes(att, attachment, sizeof(attachment)))
        {
            z_drop(z_move(bytes));
            return AutoPubPublishResult::SendFailed;
        }
        z_publisher_put_options_t options;
        z_publisher_put_options_default(&options);
        options.attachment = z_move(att);
        const int rslt = z_publisher_put(z_loan(e.pub), z_move(bytes), &options);
        const uint32_t elapsedUs = (uint32_t)(esp_timer_get_time() - startUs);
        if (elapsedUs > _putMaxUs)
            _putMaxUs = elapsedUs;
        if (rslt != Z_OK)
        {
            ++_putFailures;
            return AutoPubPublishResult::SendFailed;
        }
        ++e.sequence;
        ++_published;
        if (outSequence) *outSequence = e.sequence;
        if (outPeersSent) *outPeersSent = 1;
        return AutoPubPublishResult::Accepted;
    }

    void setNow(uint64_t nowMs) { _nowMs = nowMs; }

    /// @brief Declare or undeclare at most one endpoint (loop task)
    /// @return true if anything was sent
    bool service(uint64_t nowMs)
    {
        _nowMs = nowMs;
        if (!_live)
            return false;
        for (uint8_t offset = 0; offset < ZENOH_PICO_AUTOPUB_CAPACITY; ++offset)
        {
            const uint8_t slot = (uint8_t)((_cursor + offset) % ZENOH_PICO_AUTOPUB_CAPACITY);
            Entry& e = _entries[slot];
            if (e.state == Entry::State::PendingDeclare)
            {
                z_view_keyexpr_t key, token;
                if (z_view_keyexpr_from_str(&key, e.key) != Z_OK ||
                    z_view_keyexpr_from_str(&token, e.tokenKey) != Z_OK ||
                    z_declare_publisher(z_loan(_live->session), &e.pub, z_loan(key), nullptr) != Z_OK)
                {
                    ++_declareFailures;
                    release(e);
                    continue;
                }
                if (z_liveliness_declare_token(z_loan(_live->session), &e.token, z_loan(token), nullptr) != Z_OK)
                {
                    ++_declareFailures;
                    z_drop(z_move(e.pub));
                    release(e);
                    continue;
                }
                e.state = Entry::State::Declared;
            }
            else if (e.state == Entry::State::PendingUndeclare)
            {
                // Explicit undeclares rather than z_drop, which discards the result
                const int tokenRslt = z_liveliness_undeclare_token(z_move(e.token));
                const int pubRslt = z_undeclare_publisher(z_move(e.pub));
                ++_undeclares;
                if (tokenRslt != Z_OK || pubRslt != Z_OK)
                {
                    ++_undeclareFailures;
                    _lastUndeclareRslt = tokenRslt != Z_OK ? tokenRslt : pubRslt;
                }
                LOG_I("RaftROS", "undeclared publisher %s (token %d, publisher %d)", e.key, tokenRslt, pubRslt);
                release(e);
            }
            else
                continue;
            _cursor = (uint8_t)((slot + 1) % ZENOH_PICO_AUTOPUB_CAPACITY);
            return true;
        }
        return false;
    }

    uint8_t inUseCount() const
    {
        uint8_t n = 0;
        for (const auto& e : _entries)
            n += e.state != Entry::State::Free;
        return n;
    }
    uint8_t pendingCount() const
    {
        uint8_t n = 0;
        for (const auto& e : _entries)
            n += e.state == Entry::State::PendingDeclare || e.state == Entry::State::PendingUndeclare;
        return n;
    }
    uint32_t published() const { return _published; }
    uint32_t putFailures() const { return _putFailures; }
    uint32_t declareFailures() const { return _declareFailures; }
    uint32_t putMaxUs() const { return _putMaxUs; }
    uint32_t undeclares() const { return _undeclares; }
    uint32_t undeclareFailures() const { return _undeclareFailures; }
    int lastUndeclareRslt() const { return _lastUndeclareRslt; }

    static ZenohROSCodec::QoS qosForProfile(RaftRuntime::AutoPub::AutoPubQoSProfileId profileId)
    {
        const auto profile = RaftRuntime::AutoPub::AutoPubQoSProfile_get(profileId);
        ZenohROSCodec::QoS qos;
        qos.reliability = profile.reliability == RaftRuntime::AutoPub::AUTOPUB_RELIABILITY_RELIABLE ?
            ZenohROSCodec::Reliability::Reliable : ZenohROSCodec::Reliability::BestEffort;
        qos.durability = profile.durability == RaftRuntime::AutoPub::AUTOPUB_DURABILITY_TRANSIENT_LOCAL ?
            ZenohROSCodec::Durability::TransientLocal : ZenohROSCodec::Durability::Volatile;
        qos.depth = profile.historyDepth;
        return qos;
    }

private:
    struct Entry
    {
        enum class State : uint8_t { Free, PendingDeclare, Declared, PendingUndeclare };
        State state = State::Free;
        uint64_t sequence = 0;
        ZenohROSIdentity::Gid gid{};
        char key[KEY_MAX] = {};
        char tokenKey[TOKEN_MAX] = {};
        z_owned_publisher_t pub;
        z_owned_liveliness_token_t token;
    };
    static void release(Entry& e)
    {
        e.state = Entry::State::Free;
        e.key[0] = '\0';
        e.tokenKey[0] = '\0';
        e.sequence = 0;
        z_internal_null(&e.pub);
        z_internal_null(&e.token);
    }

    Deps _deps;
    Live* _live = nullptr;
    Entry _entries[ZENOH_PICO_AUTOPUB_CAPACITY];
    uint64_t _nextEntityId = 2;
    uint64_t _nowMs = 0;
    uint8_t _cursor = 0;
    uint32_t _published = 0, _putFailures = 0, _declareFailures = 0, _putMaxUs = 0;
    uint32_t _undeclares = 0, _undeclareFailures = 0;
    int _lastUndeclareRslt = 0;
};

} // namespace RaftRuntime::ZenohPico

using namespace RaftRuntime::ZenohPico;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Construction and setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RaftROS::RaftROS(const char* pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig),
      _pico(new PicoState()),
      _autoPubBackend(new ZenohPicoAutoPubBackend()),
      _sysConfig(sysConfig)
{
}

RaftROS::~RaftROS()
{
    if (_pico && _pico->live)
    {
        _autoPubBackend->setLive(nullptr);
        delete _pico->live;
        _pico->live = nullptr;
    }
}

void RaftROS::setup()
{
    _isEnabled = configGetBool("enable", false);
    _domainId = configGetLong("domainId", 0);
    _nodeName = configGetString("nodeName", "raft_esp32");
    _nodeNamespace = configGetString("nodeNamespace", "/");
    _routerFromConfig = configGetString("routerHost", "").length() > 0;
#ifdef CONFIG_RAFTROS_ZENOH_ROUTER_HOST
    _routerHost = configGetString("routerHost", CONFIG_RAFTROS_ZENOH_ROUTER_HOST);
    _routerPort = configGetLong("routerPort", CONFIG_RAFTROS_ZENOH_ROUTER_PORT);
#else
    _routerHost = configGetString("routerHost", "");
    _routerPort = configGetLong("routerPort", 7447);
#endif
    if (!_isEnabled)
    {
        LOG_I(MODULE_PREFIX, "setup DISABLED");
        return;
    }
    struct in_addr parsed;
    if (inet_aton(_routerHost.c_str(), &parsed) == 0)
    {
        LOG_E(MODULE_PREFIX, "setup routerHost '%s' is not an IPv4 address - disabled", _routerHost.c_str());
        _isEnabled = false;
        return;
    }
    if (!buildNodeIdentity())
    {
        LOG_E(MODULE_PREFIX, "setup could not build node identity - disabled");
        _isEnabled = false;
        return;
    }

    _pico->sampleQueue = xQueueCreate(QUEUE_DEPTH, sizeof(SampleItem));
    _pico->requestQueue = xQueueCreate(QUEUE_DEPTH, sizeof(RequestItem));
    if (!_pico->sampleQueue || !_pico->requestQueue)
    {
        LOG_E(MODULE_PREFIX, "setup could not create queues - disabled");
        _isEnabled = false;
        return;
    }
    for (uint8_t i = 0; i < MAX_SUBSCRIPTIONS; ++i)
        _pico->subCtx[i] = {_pico.get(), i};
    for (uint8_t i = 0; i < MAX_SERVICES; ++i)
        _pico->svcCtx[i] = {_pico.get(), i};

    ZenohPicoAutoPubBackend::Deps deps;
    deps.domainId = _domainId;
    deps.sessionId = _sessionIdStr;
    deps.nodeNamespace = _nodeNamespace.c_str();
    deps.nodeName = _nodeName.c_str();
    _autoPubBackend->setup(deps);

    _chatterEnabled = configGetBool("chatterEnable", true);
    if (_chatterEnabled)
    {
        RaftRuntime::AutoPub::AutoPubEndpointDesc chatter;
        chatter.deviceId = {0, 0, 0};
        chatter.msgKind = RaftRuntime::AutoPub::AutoPubMsgKind::String;
        chatter.qosProfileId = RaftRuntime::AutoPub::AutoPubQoSProfileId::FallbackString;
        if (chatter.setNames("/chatter", RaftRuntime::AutoPub::AutoPubClassMap_typeName(
                                             RaftRuntime::AutoPub::AutoPubMsgKind::String)))
            _chatterSlot = _autoPubBackend->createPublisher(chatter);
        if (_chatterSlot == ZenohPicoAutoPubBackend::INVALID_SLOT)
            _chatterEnabled = false;
    }

    if (_autoPubSource.setup(*_autoPubBackend, getSysManager(), configGetString("qosProfiles", "{}").c_str()))
        LOG_I(MODULE_PREFIX, "setup auto-publish listener registered with DeviceManager");

    LOG_I(MODULE_PREFIX, "setup backend=zenoh-pico router=%s:%u domain=%u node=%s%s session=%s",
          _routerHost.c_str(), (unsigned)_routerPort, (unsigned)_domainId,
          _nodeNamespace.c_str(), _nodeName.c_str(), _sessionIdStr);
    setupParameters();
}

/// @brief Session identity: WiFi MAC plus boot randomness, as on the other
/// Zenoh build.  zenoh-pico takes it as 32 hex digits in byte order and
/// prints it reversed with leading zeros stripped - the text rmw_zenoh keys use.
bool RaftROS::buildNodeIdentity()
{
    uint8_t mac[6] = {};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    for (size_t i = 0; i < sizeof(mac); ++i)
        _identity[i] = mac[i];
    for (size_t i = sizeof(mac); i < _identity.size(); ++i)
        _identity[i] = (uint8_t)esp_random();
    for (size_t i = 0; i < _identity.size(); ++i)
        snprintf(_pico->zid + i * 2, 3, "%02x", _identity[i]);
    for (size_t i = 0; i < _identity.size(); ++i)
        snprintf(_sessionIdStr + i * 2, 3, "%02x", _identity[_identity.size() - 1 - i]);
    size_t first = 0;
    while (_sessionIdStr[first] == '0' && _sessionIdStr[first + 1] != '\0')
        ++first;
    if (first)
        memmove(_sessionIdStr, _sessionIdStr + first, strlen(_sessionIdStr + first) + 1);
    return ZenohROSCodec::formatNodeToken(_nodeToken, sizeof(_nodeToken), _domainId, _sessionIdStr,
                                          1, "/", _nodeNamespace.c_str(), _nodeName.c_str());
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void noteMax(uint32_t& maxUs, int64_t startUs)
{
    const uint32_t elapsed = (uint32_t)(esp_timer_get_time() - startUs);
    if (elapsed > maxUs)
        maxUs = elapsed;
}

void RaftROS::loop()
{
    if (!_isEnabled)
        return;
    const uint32_t nowMs = millis();
    const int64_t passStartUs = esp_timer_get_time();

    _autoPubBackend->setNow(nowMs);
    _autoPubSource.drainSamples();
    noteMax(_loopDrainMaxUs, passStartUs);

    switch (_connState)
    {
        case ConnState::DISCONNECTED:
            if (Raft::isTimeout(nowMs, _lastConnectAttemptMs, _reconnectDelayMs) && getLocalIP() != 0)
            {
                const int64_t startUs = esp_timer_get_time();
                startOpen(nowMs);
                noteMax(_loopConnMaxUs, startUs);
            }
            break;
        case ConnState::OPENING:
        {
            const int64_t startUs = esp_timer_get_time();
            checkOpen(nowMs);
            noteMax(_loopConnMaxUs, startUs);
            break;
        }
        case ConnState::READY:
        {
            if (z_session_is_closed(z_loan(_pico->live->session)))
            {
                closeSession("session closed");
                break;
            }
            const int64_t rxStartUs = esp_timer_get_time();
            drainSamples();
            drainRequests(nowMs);
            _services.service(nowMs, SERVICE_DISPATCH_BUDGET);
            noteMax(_loopRxMaxUs, rxStartUs);

            const int64_t txStartUs = esp_timer_get_time();
            // One outbound operation per pass, as on the other Zenoh build:
            // replies first (a client is waiting), then declarations
            if (!sendServiceReply())
                stepDeclarations();
            publishChatter(nowMs);
            noteMax(_loopTxMaxUs, txStartUs);
            applyPendingRouterHost();
            break;
        }
    }
    noteMax(_loopPassMaxUs, passStartUs);
}

uint32_t RaftROS::getLocalIP()
{
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ipInfo;
    if (!netif || esp_netif_get_ip_info(netif, &ipInfo) != ESP_OK)
        return 0;
    return ipInfo.ip.addr;
}

/// @brief Start a helper task that runs the blocking z_open
void RaftROS::startOpen(uint32_t nowMs)
{
    _lastConnectAttemptMs = nowMs;
    if (_pico->openState != PicoState::OPEN_IDLE || _pico->closing)
        return;
    snprintf(_pico->locator, sizeof(_pico->locator), "tcp/%s:%u", _routerHost.c_str(), (unsigned)_routerPort);
    _pico->openState = PicoState::OPEN_RUNNING;
    if (xTaskCreatePinnedToCore(openTask, "zpOpen", OPEN_TASK_STACK, _pico.get(), HELPER_PRIORITY, nullptr, HELPER_CORE) != pdPASS)
    {
        _pico->openState = PicoState::OPEN_IDLE;
        LOG_W(MODULE_PREFIX, "could not start the session-open task");
        return;
    }
    _connState = ConnState::OPENING;
}

/// @brief Take the open task's result, if it has one
void RaftROS::checkOpen(uint32_t nowMs)
{
    const int state = _pico->openState;
    if (state == PicoState::OPEN_RUNNING)
        return;
    _lastOpenResult = _pico->openResult;
    _lastOpenMs = _pico->openMs;
    _pico->openState = PicoState::OPEN_IDLE;
    if (state == PicoState::OPEN_DONE_OK)
    {
        _pico->live = _pico->openLive;
        _pico->openLive = nullptr;
        _autoPubBackend->setLive(_pico->live);
        _nodeTokenDeclared = false;
        for (auto& s : _subscriptions)
            s.declared = false;
        for (auto& s : _serviceSlots)
            s.declared = false;
        _connState = ConnState::READY;
        _reconnectDelayMs = RECONNECT_DELAY_MIN_MS;
        _lastSessionMs = nowMs;
        ++_sessionCount;
        LOG_I(MODULE_PREFIX, "session %u open to %s (z_open took %ums in its own task)%s",
              (unsigned)_sessionCount, _pico->locator, (unsigned)_lastOpenMs,
              _connectFailures ? " - router reachable again" : "");
        _connectFailures = 0;
        return;
    }
    ++_connectFailures;
    _connState = ConnState::DISCONNECTED;
    LOG_W(MODULE_PREFIX, "could not open a session to %s (z_open %d after %ums), retrying in %ums%s",
          _pico->locator, _lastOpenResult, (unsigned)_lastOpenMs, (unsigned)_reconnectDelayMs,
          _connectFailures >= 3 ? " - ROUTER UNREACHABLE: is rmw_zenohd running there, and is routerHost right for this network?" : "");
    _reconnectDelayMs = _reconnectDelayMs >= RECONNECT_DELAY_MAX_MS ? RECONNECT_DELAY_MAX_MS : _reconnectDelayMs * 2;
}

/// @brief Drop the session (off the loop) and go back to opening one
void RaftROS::closeSession(const char* reason)
{
    // Queries held for a reply belong to the dying session
    for (auto& f : _pico->inflight)
    {
        if (f.used)
        {
            z_drop(z_move(f.query));
            f.used = false;
        }
    }
    _autoPubBackend->setLive(nullptr);
    Live* dying = _pico->live;
    _pico->live = nullptr;
    if (dying)
    {
        _pico->closing = true;
        auto* job = new CloseJob{dying, _pico.get()};
        if (xTaskCreatePinnedToCore(closeTask, "zpClose", OPEN_TASK_STACK, job, HELPER_PRIORITY, nullptr, HELPER_CORE) != pdPASS)
        {
            delete job;
            delete dying;
            _pico->closing = false;
        }
    }
    _connState = ConnState::DISCONNECTED;
    _lastConnectAttemptMs = millis();
    LOG_W(MODULE_PREFIX, "session dropped (%s), reopening in %ums", reason, (unsigned)_reconnectDelayMs);
}

/// @brief Declare one thing per pass: the node token, then endpoints,
/// subscriptions and services.  zenoh-pico sends each declaration as it is
/// made, so this bounds the loop the way one message per pass does elsewhere.
bool RaftROS::stepDeclarations()
{
    const int64_t startUs = esp_timer_get_time();
    bool did = false;
    Live& live = *_pico->live;
    if (!_nodeTokenDeclared)
    {
        z_view_keyexpr_t key;
        _nodeTokenDeclared = true;
        if (z_view_keyexpr_from_str(&key, _nodeToken) == Z_OK &&
            z_liveliness_declare_token(z_loan(live.session), &live.nodeToken, z_loan(key), nullptr) == Z_OK)
        {
            live.nodeTokenValid = true;
            LOG_I(MODULE_PREFIX, "declared node %s%s", _nodeNamespace.c_str(), _nodeName.c_str());
        }
        else
            LOG_E(MODULE_PREFIX, "node token declaration failed - node will not appear in the ROS graph");
        did = true;
    }
    else if (_autoPubBackend->service(millis()))
        did = true;
    else
    {
        for (uint8_t i = 0; i < _subscriptionCount && !did; ++i)
        {
            Subscription& sub = _subscriptions[i];
            if (!sub.inUse || sub.declared)
                continue;
            sub.declared = true;
            did = true;
            const auto qosId = _autoPubSource.resolveSubscriptionQoS(sub.rosTopic);
            const ZenohROSCodec::Endpoint endpoint{sub.entityId, ZenohROSCodec::EndpointKind::Subscription,
                sub.rosTopic, sub.type, sub.typeHash, ZenohPicoAutoPubBackend::qosForProfile(qosId)};
            char token[TOKEN_MAX];
            const ZenohROSCodec::NodeIdentity node{_domainId, _sessionIdStr, 1, "/", _nodeNamespace.c_str(), _nodeName.c_str()};
            z_view_keyexpr_t key, tokenKey;
            z_owned_closure_sample_t closure;
            z_closure_sample(&closure, onSample, nullptr, &_pico->subCtx[i]);
            if (!ZenohROSCodec::formatEndpointToken(token, sizeof(token), node, endpoint) ||
                z_view_keyexpr_from_str(&key, sub.key) != Z_OK || z_view_keyexpr_from_str(&tokenKey, token) != Z_OK ||
                z_declare_subscriber(z_loan(live.session), &live.subs[i].sub, z_loan(key), z_move(closure), nullptr) != Z_OK)
            {
                LOG_W(MODULE_PREFIX, "subscription %s could not be declared", sub.rosTopic);
                continue;
            }
            if (z_liveliness_declare_token(z_loan(live.session), &live.subs[i].token, z_loan(tokenKey), nullptr) != Z_OK)
                z_internal_null(&live.subs[i].token);
            live.subs[i].valid = true;
            LOG_I(MODULE_PREFIX, "subscribed to %s (qos=%s)", sub.rosTopic, RaftRuntime::AutoPub::AutoPubQoSProfile_name(qosId));
        }
        for (uint8_t i = 0; i < MAX_SERVICES && !did; ++i)
        {
            ServiceSlot& svc = _serviceSlots[i];
            if (!svc.inUse || svc.declared)
                continue;
            svc.declared = true;
            did = true;
            const ZenohROSCodec::Endpoint endpoint{svc.entityId, ZenohROSCodec::EndpointKind::Service,
                _services.rosName(i), svc.wireType, svc.typeHash,
                {ZenohROSCodec::Reliability::Reliable, ZenohROSCodec::Durability::Volatile, 10}};
            const ZenohROSCodec::NodeIdentity node{_domainId, _sessionIdStr, 1, "/", _nodeNamespace.c_str(), _nodeName.c_str()};
            char token[TOKEN_MAX];
            z_view_keyexpr_t key, tokenKey;
            z_owned_closure_query_t closure;
            z_closure_query(&closure, onQuery, nullptr, &_pico->svcCtx[i]);
            if (!ZenohROSCodec::formatEndpointToken(token, sizeof(token), node, endpoint) ||
                z_view_keyexpr_from_str(&key, svc.key) != Z_OK || z_view_keyexpr_from_str(&tokenKey, token) != Z_OK)
            {
                z_drop(z_move(closure));
                LOG_W(MODULE_PREFIX, "service %s could not be declared", _services.rosName(i));
                continue;
            }
            z_queryable_options_t options;
            z_queryable_options_default(&options);
            options.complete = true;
            if (z_declare_queryable(z_loan(live.session), &live.svcs[i].queryable, z_loan(key), z_move(closure), &options) != Z_OK)
            {
                LOG_W(MODULE_PREFIX, "service %s queryable could not be declared", _services.rosName(i));
                continue;
            }
            if (z_liveliness_declare_token(z_loan(live.session), &live.svcs[i].token, z_loan(tokenKey), nullptr) != Z_OK)
                z_internal_null(&live.svcs[i].token);
            live.svcs[i].valid = true;
        }
    }
    if (did)
        noteMax(_loopDeclMaxUs, startUs);
    return did;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Samples, requests and replies
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::drainSamples()
{
    SampleItem item;
    // Bounded: the queue holds QUEUE_DEPTH, and a handler may log
    for (uint8_t n = 0; n < QUEUE_DEPTH && xQueueReceive(_pico->sampleQueue, &item, 0) == pdTRUE; ++n)
    {
        if (item.subscription >= _subscriptionCount)
        {
            ++_samplesDropped;
            continue;
        }
        Subscription& sub = _subscriptions[item.subscription];
        ++sub.received;
        char text[256];
        const auto decoded = RaftRuntime::AutoPub::AutoPubStringMessage_decode(item.payload, item.payloadLen, text, sizeof(text));
        if (!decoded.success)
        {
            ++_samplesDropped;
            continue;
        }
        ZenohROSCodec::Attachment attachment;
        static const uint8_t UNKNOWN_GID[ZenohROSCodec::GID_SIZE] = {};
        const bool haveGid = item.attachmentLen &&
            ZenohROSCodec::decodeAttachment(item.attachment, item.attachmentLen, attachment);
        const uint8_t* gid = haveGid ? attachment.publisherGid.data() : UNKNOWN_GID;
        const StringMessageHandler& handler = sub.handler ? sub.handler : _defaultHandler;
        if (handler)
            handler(gid + 12, gid, text, decoded.textLen);
    }
}

void RaftROS::drainRequests(uint32_t nowMs)
{
    // A query the registry never staged a reply for (it refuses at most one
    // request at a time when full) would be held for ever: release it, which
    // sends its final, well after any reply could still come
    for (auto& f : _pico->inflight)
    {
        if (f.used && Raft::isTimeout(nowMs, f.sinceMs, 15000))
        {
            z_drop(z_move(f.query));
            f.used = false;
            ++_requestsDropped;
        }
    }
    RequestItem item;
    for (uint8_t n = 0; n < QUEUE_DEPTH && xQueueReceive(_pico->requestQueue, &item, 0) == pdTRUE; ++n)
    {
        // A slot to hold the query until the registry has a reply for it
        PicoState::Inflight* slot = nullptr;
        for (auto& f : _pico->inflight)
            if (!f.used) { slot = &f; break; }
        if (!slot || item.service >= MAX_SERVICES || !_serviceSlots[item.service].inUse)
        {
            ++_requestsDropped;
            z_drop(z_move(item.query));      // the client gets its final now
            continue;
        }
        slot->used = true;
        slot->sinceMs = nowMs;
        slot->requestId = _nextRequestId++;
        z_take(&slot->query, z_move(item.query));
        // The registry answers a refusal through next() as well, so the query
        // is kept either way and released when its reply has gone
        _services.accept(item.service, slot->requestId, item.payload, item.payloadLen,
                         item.attachmentLen ? item.attachment : nullptr, item.attachmentLen, 0, nowMs);
    }
}

/// @brief Send what the registry has staged.  A reply and its final go
/// together: the reply, then dropping the query, which sends the final.
bool RaftROS::sendServiceReply()
{
    using RaftRuntime::AutoPub::AutoPubServiceSend;
    const AutoPubServiceSend send = _services.next();
    if (send.kind == AutoPubServiceSend::Kind::None)
        return false;
    const int64_t startUs = esp_timer_get_time();
    PicoState::Inflight* slot = nullptr;
    for (auto& f : _pico->inflight)
        if (f.used && f.requestId == send.requestId) { slot = &f; break; }

    if (slot)
    {
        if (send.kind == AutoPubServiceSend::Kind::Response)
        {
            z_view_keyexpr_t key;
            z_owned_bytes_t payload, att;
            z_query_reply_options_t options;
            z_query_reply_options_default(&options);
            if (z_view_keyexpr_from_str(&key, _serviceSlots[send.slot].key) == Z_OK &&
                toBytes(payload, send.payload, send.payloadLen))
            {
                if (toBytes(att, send.attachment, RaftRuntime::AutoPub::AUTOPUB_SERVICE_ATTACHMENT_SIZE))
                    options.attachment = z_move(att);
                z_query_reply(z_loan(slot->query), z_loan(key), z_move(payload), &options);
            }
        }
        else if (send.kind == AutoPubServiceSend::Kind::Error)
        {
            z_owned_bytes_t reason;
            z_query_reply_err_options_t options;
            z_query_reply_err_options_default(&options);
            if (toBytes(reason, reinterpret_cast<const uint8_t*>(send.reason), strlen(send.reason)))
                z_query_reply_err(z_loan(slot->query), z_move(reason), &options);
        }
        z_drop(z_move(slot->query));          // RESPONSE_FINAL
        slot->used = false;
    }
    _services.sent();
    if (send.kind != AutoPubServiceSend::Kind::Final)
        _services.sent();                       // the final went with it
    noteMax(_loopReplyMaxUs, startUs);
    return true;
}

void RaftROS::publishChatter(uint32_t nowMs)
{
    if (!_chatterEnabled || !Raft::isTimeout(nowMs, _lastChatterSendMs, _chatterPeriodMs))
        return;
    char message[64];
    snprintf(message, sizeof(message), "Hello from %s [%u]", _nodeName.c_str(), (unsigned)_chatterMsgIndex);
    uint8_t payload[128];
    CDREncoder encoder;
    encoder.reset(payload, sizeof(payload));
    if (!encoder.writeEncapsulationHeader() || !encoder.writeString(message))
        return;
    if (_autoPubBackend->publish(_chatterSlot, payload, encoder.getPos()) ==
            RaftRuntime::AutoPub::AutoPubPublishResult::Accepted)
    {
        ++_chatterMsgIndex;
        _lastChatterSendMs = nowMs;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Subscriptions and services (registration - the same rules as the other Zenoh build)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int RaftROS::addStringSubscription(const char* topic, const char* type, StringMessageHandler handler)
{
    using namespace RaftRuntime::AutoPub;
    if (!topic || !*topic || !type || !*type)
        return -1;
    const char* rosTopic = topic;
    if (strncmp(topic, "rt/", 3) == 0)
        rosTopic = topic + 2;
    char rosTopicBuf[AUTOPUB_TOPIC_MAX_LEN];
    if (rosTopic[0] != '/')
    {
        if (snprintf(rosTopicBuf, sizeof(rosTopicBuf), "/%s", rosTopic) >= (int)sizeof(rosTopicBuf))
            return -1;
        rosTopic = rosTopicBuf;
    }
    for (uint8_t i = 0; i < _subscriptionCount; ++i)
    {
        if (strcmp(_subscriptions[i].rosTopic, rosTopic) == 0)
        {
            if (handler)
                _subscriptions[i].handler = std::move(handler);
            return i;
        }
    }
    if (_subscriptionCount >= MAX_SUBSCRIPTIONS)
        return -1;
    const char* typeHash = AutoPubClassMap_typeHash(AutoPubClassMap_kindForTypeName(type));
    if (!typeHash)
        return -1;
    Subscription& sub = _subscriptions[_subscriptionCount];
    if (snprintf(sub.rosTopic, sizeof(sub.rosTopic), "%s", rosTopic) >= (int)sizeof(sub.rosTopic) ||
        snprintf(sub.type, sizeof(sub.type), "%s", type) >= (int)sizeof(sub.type) ||
        !ZenohROSCodec::formatTopicKey(sub.key, sizeof(sub.key), _domainId, sub.rosTopic, sub.type, typeHash))
        return -1;
    sub.entityId = _nextSubscriptionEntityId++;
    sub.typeHash = typeHash;
    sub.handler = std::move(handler);
    sub.inUse = true;
    sub.declared = false;
    return _subscriptionCount++;
}

int RaftROS::addService(const char* name, const char* type, ServiceHandler handler)
{
    using namespace RaftRuntime::AutoPub;
    if (!name || !*name || !type)
        return -1;
    AutoPubServiceKind kind = AutoPubServiceCodec_kindForWireType(type);
    const char* typeHash = AutoPubClassMap_serviceTypeHash(type);
    if (kind == AutoPubServiceKind::Unknown && typeHash)
        kind = AutoPubServiceKind::Raw;
    if (kind == AutoPubServiceKind::Unknown || !typeHash)
    {
        LOG_W(MODULE_PREFIX, "addService '%s': type '%s' is not one this build can serve", name, type);
        return -1;
    }
    char rosName[AUTOPUB_SERVICE_NAME_MAX];
    if (snprintf(rosName, sizeof(rosName), "%s%s", name[0] == '/' ? "" : "/", name) >= (int)sizeof(rosName))
        return -1;
    const uint8_t slot = _services.add(rosName, kind, std::move(handler));
    if (slot == decltype(_services)::INVALID_SLOT)
        return -1;
    ServiceSlot& svc = _serviceSlots[slot];
    svc.typeHash = typeHash;
    svc.entityId = _nextServiceEntityId++;
    if (strlen(type) >= sizeof(svc.wireType) ||
        !ZenohROSCodec::formatTopicKey(svc.key, sizeof(svc.key), _domainId, rosName, type, typeHash))
    {
        _services.remove(slot);
        return -1;
    }
    strcpy(svc.wireType, type);
    svc.inUse = true;
    svc.declared = false;
    LOG_I(MODULE_PREFIX, "serving %s (%s)", rosName, type);
    return slot;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Parameters (as on the other Zenoh build)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::setupParameters()
{
    using namespace RaftRuntime::AutoPub;
    declareParameter("use_sim_time", false, "Simulated time is not supported on this node", true);
    declareParameter("chatterEnable", _chatterEnabled, "Publish /chatter", false,
        [this](const ParamValue& v, const char*&) { _chatterEnabled = v.boolValue; return true; });
    declareParameter("chatterPeriodMs", (int64_t)_chatterPeriodMs, "Period of /chatter in ms (100-60000)", false,
        [this](const ParamValue& v, const char*& reason)
        {
            if (v.integerValue < 100 || v.integerValue > 60000)
            {
                reason = "chatterPeriodMs must be 100-60000";
                return false;
            }
            _chatterPeriodMs = (uint32_t)v.integerValue;
            return true;
        });
    declareParameter("routerHost", _routerHost.c_str(),
        "Zenoh router IPv4 address; persisted, applied on the next connection", false,
        [this](const ParamValue& v, const char*& reason)
        {
            struct in_addr addr;
            if (!inet_aton(v.stringValue, &addr))
            {
                reason = "routerHost must be an IPv4 address (DNS would block the loop)";
                return false;
            }
            if (!persistRouterHost(v.stringValue))
            {
                reason = "could not persist routerHost to the settings overlay";
                return false;
            }
            strncpy(_routerHostPending, v.stringValue, sizeof(_routerHostPending) - 1);
            return true;
        });

    struct ParamService { const char* name; const char* type; std::function<uint32_t(const uint8_t*, uint32_t, uint8_t*, uint32_t)> fn; };
    auto& store = _params;
    const ParamService services[] = {
        {"list_parameters", "rcl_interfaces::srv::dds_::ListParameters_",
            [&store](const uint8_t* r, uint32_t n, uint8_t* o, uint32_t c) { return store.list(r, n, o, c); }},
        {"get_parameters", "rcl_interfaces::srv::dds_::GetParameters_",
            [&store](const uint8_t* r, uint32_t n, uint8_t* o, uint32_t c) { return store.get(r, n, o, c); }},
        {"get_parameter_types", "rcl_interfaces::srv::dds_::GetParameterTypes_",
            [&store](const uint8_t* r, uint32_t n, uint8_t* o, uint32_t c) { return store.getTypes(r, n, o, c); }},
        {"set_parameters", "rcl_interfaces::srv::dds_::SetParameters_",
            [&store](const uint8_t* r, uint32_t n, uint8_t* o, uint32_t c) { return store.set(r, n, o, c); }},
        {"set_parameters_atomically", "rcl_interfaces::srv::dds_::SetParametersAtomically_",
            [&store](const uint8_t* r, uint32_t n, uint8_t* o, uint32_t c) { return store.setAtomically(r, n, o, c); }},
        {"describe_parameters", "rcl_interfaces::srv::dds_::DescribeParameters_",
            [&store](const uint8_t* r, uint32_t n, uint8_t* o, uint32_t c) { return store.describe(r, n, o, c); }},
    };
    String node = _nodeNamespace;
    if (!node.endsWith("/"))
        node += "/";
    node += _nodeName;
    for (const ParamService& svc : services)
    {
        const String name = node + "/" + svc.name;
        auto fn = svc.fn;
        addService(name.c_str(), svc.type,
            [this, fn](const ServiceRequest& request, ServiceReply& reply)
            {
                const uint32_t n = fn(request.fields.raw, request.fields.rawLen, _paramReplyBuf, sizeof(_paramReplyBuf));
                if (n == 0)
                {
                    reply.reason = "bad request";
                    return ServiceOutcome::Refused;
                }
                reply.fields.raw = _paramReplyBuf;
                reply.fields.rawLen = n;
                return ServiceOutcome::Replied;
            });
    }
}

/// @brief Serialise one element of a RaftJson document (copied from the other
/// Zenoh build, where the reason it is done leaf by leaf is explained)
static void serialiseJsonElement(const RaftJson& json, const String& path, String& out, int depth = 0)
{
    int arrayLen = 0;
    const RaftJsonIF::RaftJsonType type = json.getType(path.c_str(), arrayLen);
    if (depth > 8)
    {
        out += "null";
        return;
    }
    switch (type)
    {
        case RaftJsonIF::RAFT_JSON_OBJECT:
        {
            std::vector<String> keys;
            json.getKeys(path.c_str(), keys);
            out += "{";
            bool first = true;
            for (const String& key : keys)
            {
                if (!first)
                    out += ",";
                first = false;
                out += "\"" + key + "\":";
                serialiseJsonElement(json, path.length() ? path + "/" + key : key, out, depth + 1);
            }
            out += "}";
            return;
        }
        case RaftJsonIF::RAFT_JSON_ARRAY:
            out += "[";
            for (int index = 0; index < arrayLen; ++index)
            {
                if (index)
                    out += ",";
                serialiseJsonElement(json, path + "[" + String(index) + "]", out, depth + 1);
            }
            out += "]";
            return;
        case RaftJsonIF::RAFT_JSON_STRING:
        {
            String value = json.getString(path.c_str(), "");
            value.replace("\\", "\\\\");
            value.replace("\"", "\\\"");
            out += "\"" + value + "\"";
            return;
        }
        case RaftJsonIF::RAFT_JSON_NUMBER:
        case RaftJsonIF::RAFT_JSON_BOOLEAN:
            out += json.getString(path.c_str(), "null");
            return;
        default:
            out += "null";
            return;
    }
}

bool RaftROS::persistRouterHost(const char* host)
{
    const char* overlayDoc = _sysConfig.getJsonDoc();
    RaftJson overlay((overlayDoc && overlayDoc[0] == '{') ? overlayDoc : "{}");
    std::vector<String> keys;
    overlay.getKeys("", keys);
    String doc = "{";
    for (const String& key : keys)
    {
        if (key == "RaftROS")
            continue;
        doc += "\"" + key + "\":";
        serialiseJsonElement(overlay, key, doc);
        doc += ",";
    }
    doc += "\"RaftROS\":{";
    std::vector<String> ourKeys;
    overlay.getKeys("RaftROS", ourKeys);
    for (const String& key : ourKeys)
    {
        if (key == "routerHost")
            continue;
        doc += "\"" + key + "\":";
        serialiseJsonElement(overlay, "RaftROS/" + key, doc);
        doc += ",";
    }
    doc += String("\"routerHost\":\"") + host + "\"}}";
    return _sysConfig.setJsonDoc(doc.c_str());
}

void RaftROS::applyPendingRouterHost()
{
    if (_routerHostPending[0] == '\0' || _services.inflightCount() != 0)
        return;
    _routerHost = _routerHostPending;
    _routerFromConfig = true;
    _routerHostPending[0] = '\0';
    _reconnectDelayMs = RECONNECT_DELAY_MIN_MS;
    closeSession("router address changed");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// API / status - the same fields as the other Zenoh build where they mean the same
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RaftROS::addRestAPIEndpoints(RestAPIEndpointManager& endpointManager)
{
    endpointManager.addEndpoint("rosstat", RestAPIEndpoint::EndpointType::ENDPOINT_CALLBACK,
                                RestAPIEndpoint::EndpointMethod::ENDPOINT_GET,
                                std::bind(&RaftROS::apiStatus, this, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3),
                                "Get RaftROS status");
}

String RaftROS::getStatusJSON() const
{
    const char* stStr = _connState == ConnState::READY ? "ready" :
                        _connState == ConnState::OPENING ? "opening" : "disconnected";
    const auto svcStats = _services.stats();
    char buf[900];
    snprintf(buf, sizeof(buf),
             R"({"rslt":"ok","backend":"zenoh-pico","en":%s,"domId":%d,"node":"%s","ns":"%s","router":"%s:%u",)"
             R"("conn":"%s","sessions":%u,"devices":%u,"pubs":%u,"pending":%u,"samples":%u,"putFails":%u,"declFails":%u,)"
             R"("undecl":%u,"undeclFails":%u,"undeclLastRslt":%d,)"
             R"("subs":%u,"rxDropped":%u,"rxQueueFull":%u,"rxOversized":%u,"stackFreeB":%u,)"
             R"("routerSource":"%s","connectFails":%u,"lastOpenMs":%u,"lastOpenResult":%d,"lastSessionAgoS":%d,)"
             R"("heapFreeB":%u,"heapMinB":%u,"heapLargestB":%u,)"
             R"("loopMaxUs":%u,"loopMaxDrainUs":%u,"loopMaxConnUs":%u,"loopMaxRxUs":%u,"loopMaxTxUs":%u,)"
             R"("loopMaxDeclUs":%u,"loopMaxPutUs":%u,"loopMaxReplyUs":%u,)"
             R"("services":%u,"svcAccepted":%u,"svcCompleted":%u,"svcDeferred":%u,"svcTimedOut":%u,)"
             R"("svcRefused":%u,"svcDropped":%u,"svcQueueFull":%u,"svcOversized":%u,"params":%u})",
             _isEnabled ? "true" : "false", (int)_domainId, _nodeName.c_str(), _nodeNamespace.c_str(),
             _routerHost.c_str(), (unsigned)_routerPort, stStr, (unsigned)_sessionCount,
             (unsigned)_autoPubSource.attachedCount(), (unsigned)_autoPubBackend->inUseCount(),
             (unsigned)_autoPubBackend->pendingCount(), (unsigned)_autoPubBackend->published(),
             (unsigned)_autoPubBackend->putFailures(), (unsigned)_autoPubBackend->declareFailures(),
             (unsigned)_autoPubBackend->undeclares(), (unsigned)_autoPubBackend->undeclareFailures(),
             _autoPubBackend->lastUndeclareRslt(),
             (unsigned)_subscriptionCount, (unsigned)_samplesDropped,
             (unsigned)_pico->samplesQueueFull.load(), (unsigned)_pico->samplesOversized.load(),
             (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)),
             _routerFromConfig ? "config" : "default", (unsigned)_connectFailures,
             (unsigned)_lastOpenMs, _lastOpenResult,
             _lastSessionMs ? (int)((millis() - _lastSessionMs) / 1000) : -1,
             (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size(),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
             (unsigned)_loopPassMaxUs, (unsigned)_loopDrainMaxUs, (unsigned)_loopConnMaxUs,
             (unsigned)_loopRxMaxUs, (unsigned)_loopTxMaxUs,
             (unsigned)_loopDeclMaxUs, (unsigned)_autoPubBackend->putMaxUs(), (unsigned)_loopReplyMaxUs,
             (unsigned)_services.inUseCount(), (unsigned)svcStats.accepted, (unsigned)svcStats.completed,
             (unsigned)svcStats.deferred, (unsigned)svcStats.timedOut,
             (unsigned)(svcStats.refusedBusy + svcStats.refusedBad + svcStats.refusedByHandler),
             (unsigned)_requestsDropped, (unsigned)_pico->requestsQueueFull.load(),
             (unsigned)_pico->requestsOversized.load(), (unsigned)_params.count());
    return buf;
}

RaftRetCode RaftROS::apiStatus(const String&, String& respStr, const APISourceInfo&)
{
    respStr = getStatusJSON();
    return RaftRetCode::RAFT_OK;
}
