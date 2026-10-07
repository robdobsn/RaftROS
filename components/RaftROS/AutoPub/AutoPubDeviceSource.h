/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubDeviceSource - turns DeviceManager bus devices into published ROS 2 topics
//
// This is the whole auto-publish pipeline above the transport: it listens for
// devices coming and going, decides what each one publishes, decodes its poll
// records on the bus task, and emits serialised samples on the loop task.  The
// only thing it does not know is how a sample reaches the network - that is the
// backend it is templated on, which both RTPS and Zenoh implement with the same
// createPublisher / destroyPublisher / publish contract.
//
// Threading (the reason this is shaped the way it is):
//   * Status callbacks and drainSamples() run on the SysMod loop task.
//   * Data callbacks run on bus worker tasks, which have a small stack and must
//     never do network I/O - a blocked bus task stalls polling for every device
//     on that bus.
//   * So a data callback only decodes into a generation-safe mailbox, and the
//     loop task serialises and publishes.  The handle in the callback info is
//     what makes a callback that races with (or outlives) detach safe: the pool
//     rejects a stale handle before the device context is ever reached.
//
// Backends differ in what else has to happen around an endpoint's life - RTPS
// announces each writer to each discovered participant and disposes it at each
// peer, Zenoh declares one token and lets the router fan it out - so the SysMod
// supplies those as hooks rather than this code knowing about either.
//
// Note on dependencies: this is the one AutoPub header that includes RaftCore.
// It is the DeviceManager adapter; the mapping, QoS, CDR and pool headers below
// it stay RaftCore-free so the host tests can exercise them directly.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "AutoPub/AutoPubAttachPlan.h"
#include "AutoPub/AutoPubCDRSerializer.h"
#include "AutoPub/AutoPubClassMap.h"
#include "AutoPub/AutoPubPublisherPool.h"
#include "AutoPub/AutoPubQoSProfile.h"
#include "AutoPub/AutoPubSampleRunner.h"

#include "BusAddrStatus.h"
#include "DeviceManager.h"
#include "DevicePollingInfo.h"
#include "DeviceTypeRecords.h"
#include "RaftBusDevicesIF.h"
#include "RaftDevice.h"
#include "RaftDeviceConsts.h"
#include "RaftJson.h"
#include "RaftThreading.h"
#include "SysManagerIF.h"
#include "esp_timer.h"

#include <atomic>
#include <cstring>
#include <functional>
#include <vector>

namespace RaftRuntime::AutoPub
{

/// @brief Raft mutex as the publisher pool's lock type
struct AutoPubRaftMutexLock
{
    RaftMutex mutex;
    AutoPubRaftMutexLock() { RaftMutex_init(mutex); }
    ~AutoPubRaftMutexLock() { RaftMutex_destroy(mutex); }
    AutoPubRaftMutexLock(const AutoPubRaftMutexLock&) = delete;
    AutoPubRaftMutexLock& operator=(const AutoPubRaftMutexLock&) = delete;
    bool lock(uint32_t timeoutMs) { return RaftMutex_lock(mutex, timeoutMs); }
    void unlock() { RaftMutex_unlock(mutex); }
};

/// @brief DeviceManager -> ROS 2 auto-publishing, over any backend
/// @tparam Backend the compile-selected auto-publish backend
/// @tparam CAPACITY endpoints the backend can hold (its registry size)
template <typename Backend, uint8_t CAPACITY>
class AutoPubDeviceSource
{
public:
    static constexpr uint8_t INVALID_SLOT = 0xFF;

    /// @brief 64 bytes covers every generated poll record (largest is 36);
    /// attach rejects a larger one rather than truncating it
    /// @brief Largest decoded poll record a device can hand over: a multizone
    /// ToF frame (VL53L5CX: 64 distances and 64 statuses, 200 B) is the
    /// biggest in the catalogue
    static constexpr uint32_t MAILBOX_RECORD_SIZE = 256;

    /// @brief A bus task must not wait on the loop task for long
    static constexpr uint32_t PRODUCER_LOCK_TIMEOUT_MS = 2;

    /// @brief Sized for the largest message the serialiser emits: a 64-zone
    /// depth Image (~320 B) or a 64-element labelled Float64MultiArray (~560 B)
    static constexpr uint32_t CDR_BUF_SIZE = 1024;

    /// @brief Called when a device's endpoints are created or about to go.
    /// `secondarySlot` and `tertiarySlot` are INVALID_SLOT unless the device
    /// is a composite with that many endpoints.
    using EndpointHook = std::function<void(const AutoPubDeviceId& deviceId,
                                            uint8_t primarySlot, uint8_t secondarySlot,
                                            uint8_t tertiarySlot)>;

    ~AutoPubDeviceSource()
    {
        for (auto*& pCtx : _ctxs)
        {
            delete pCtx;
            pCtx = nullptr;
        }
    }

    /// @brief Bind to the backend and start listening to DeviceManager
    /// @param qosProfilesJson the SysMod's `qosProfiles` config block
    /// @return true if a DeviceManager was found and the listener registered
    bool setup(Backend& backend, SysManagerIF* pSysManager, const char* qosProfilesJson)
    {
        _pBackend = &backend;
        _pSysManager = pSysManager;
        parseQoSOverrides(qosProfilesJson);

        DeviceManager* pDevMan = pSysManager ? pSysManager->getDeviceManager() : nullptr;
        if (!pDevMan)
            return false;
        pDevMan->registerForDeviceStatusChange(
            [this](RaftDevice& device, const BusAddrStatus& addrStatus) {
                this->onDeviceStatusChange(device, addrStatus);
            });
        _statusCBRegistered = true;
        return true;
    }

    /// @brief Hooks for whatever the transport needs around an endpoint's life.
    /// `detaching` runs while the endpoints still exist (RTPS needs their live
    /// identity to dispose them); `detached` runs once they are gone.
    void setEndpointHooks(EndpointHook attached, EndpointHook detaching, EndpointHook detached)
    {
        _onAttached = std::move(attached);
        _onDetaching = std::move(detaching);
        _onDetached = std::move(detached);
    }

    /// @brief Serialise and publish every device's latest sample.  Call once
    /// per loop pass, in every connection state: with no peers nothing goes out
    /// but the writer's sequence still advances once per sample, as the RTPS
    /// writer semantics require.
    void drainSamples();

    // Diagnostics
    uint8_t attachedCount() const
    {
        uint8_t count = 0;
        for (const auto* pCtx : _ctxs)
            count += pCtx ? 1 : 0;
        return count;
    }
    bool isListening() const { return _statusCBRegistered; }
    uint32_t drainGapMaxUs() const { return _drainGapMaxUs; }
    uint32_t drainGapsOver150ms() const { return _drainGapsOver150ms; }
    void resetDrainDiagnostics() { _drainGapMaxUs = 0; }
    uint8_t mailboxInUseCount() const { return _pool.inUseCount(); }
    auto poolCounters() const { return _pool.counters(); }

    /// @brief QoS profile for a subscription on a ROS topic, resolved the same
    /// way a publisher's is: a `qosProfiles` alias override for the topic's
    /// last segment wins, otherwise FallbackString (RELIABLE, VOLATILE, depth
    /// 10 - what both transports announced for readers before this existed).
    /// Resolved when the reader is announced or declared rather than when the
    /// application subscribes, so the order SysMods set up in does not matter.
    AutoPubQoSProfileId resolveSubscriptionQoS(const char* rosTopic) const
    {
        return resolveQoSProfileId(AutoPubAttachPlan_topicAlias(rosTopic), nullptr, 0, nullptr);
    }

private:
    /// @brief One attached device: what it publishes, how to decode it, and the
    /// mailbox handle its bus callbacks carry
    struct DeviceCtx
    {
        RaftDeviceID deviceID;
        AutoPubDeviceId planDeviceId;
        uint16_t deviceTypeIndex = DEVICE_TYPE_INDEX_INVALID;
        uint8_t slot = INVALID_SLOT;
        AutoPubPublisherHandle handle;

        // Cached decode metadata
        DeviceTypeRecordDecodeFn decodeFn = nullptr;
        const AttrFieldDesc* pFieldDescs = nullptr;
        uint16_t fieldCount = 0;
        uint16_t structSize = 0;
        uint16_t pollDataSizeBytes = 0;
        RaftBusDeviceDecodeState decodeState;

        /// @brief Pre-allocated because the bus task's stack is small.
        /// Touched only under the pool lock.
        uint8_t* pDecodeBuf = nullptr;
        uint32_t decodeBufSize = 0;
        uint16_t maxDecodeRecords = 0;

        /// @brief Cached at attach so the hot path skips the class-map lookup
        AutoPubMsgKind msgKind = AutoPubMsgKind::Unknown;

        /// @brief Composite extra endpoints (e.g. an AHT20 publishes
        /// Temperature on the primary slot and RelativeHumidity on the
        /// secondary; an SCD40 adds CO2 on the tertiary).  They share the
        /// decoded record but have their own publisher and topic.
        uint8_t secondarySlot = INVALID_SLOT;
        AutoPubMsgKind secondaryMsgKind = AutoPubMsgKind::Unknown;
        uint8_t tertiarySlot = INVALID_SLOT;
        AutoPubMsgKind tertiaryMsgKind = AutoPubMsgKind::Unknown;

        /// @brief Data callbacks received (incremented under the pool lock;
        /// read on the loop task only after the handle is released)
        uint32_t sampleCount = 0;

        /// @brief Gaps between consecutive data callbacks
        /// (<50, <150, <250, <400, >=400 ms), written by the bus task
        static constexpr uint8_t CB_GAP_BUCKETS = 5;
        int64_t lastCallbackUs = 0;
        std::atomic<uint32_t> cbGapHist[CB_GAP_BUCKETS] = {};

        ~DeviceCtx() { delete[] pDecodeBuf; }
    };

    // Device lifecycle
    void onDeviceStatusChange(RaftDevice& device, const BusAddrStatus& addrStatus);
    bool attachDevice(RaftDevice& device, const BusAddrStatus& addrStatus);
    void detachDevice(RaftDevice& device);
    void onDeviceData(std::vector<uint8_t> data, const void* pCallbackInfo);

    /// @brief Find an attached device's slot, or -1
    int findSlot(const RaftDeviceID& deviceID) const
    {
        for (uint8_t slot = 0; slot < CAPACITY; ++slot)
        {
            if (_ctxs[slot] && _ctxs[slot]->deviceID == deviceID)
                return slot;
        }
        return -1;
    }

    // QoS overrides from SysTypes: per-device alias beats per-class, which
    // beats the built-in default for the device's classes
    struct QoSOverride
    {
        String key;
        AutoPubQoSProfileId id = AutoPubQoSProfileId::FallbackString;
    };
    void parseQoSOverrides(const char* qosProfilesJson);
    AutoPubQoSProfileId resolveQoSProfileId(const char* pTopicAlias,
                                            const char* const* pClasArray, size_t clasCount,
                                            const char* pDeviceTypeName) const;

    Backend* _pBackend = nullptr;
    SysManagerIF* _pSysManager = nullptr;
    bool _statusCBRegistered = false;

    /// @brief Owning, indexed by backend slot; nullptr means free
    DeviceCtx* _ctxs[CAPACITY] = {};

    AutoPubPublisherPool<AutoPubRaftMutexLock, CAPACITY, MAILBOX_RECORD_SIZE> _pool;

    /// @brief Serialisation happens on the loop task, one sample at a time, so
    /// every device shares these
    static constexpr uint8_t MAX_ENDPOINTS_PER_DEVICE = 3;
    uint8_t _cdrBufs[MAX_ENDPOINTS_PER_DEVICE][CDR_BUF_SIZE] = {};

    std::vector<QoSOverride> _aliasOverrides;
    std::vector<QoSOverride> _classOverrides;

    EndpointHook _onAttached;
    EndpointHook _onDetaching;
    EndpointHook _onDetached;

    // Drain-pass diagnostics
    int64_t _lastDrainUs = 0;
    uint32_t _drainGapMaxUs = 0;
    uint32_t _drainGapsOver150ms = 0;
};

} // namespace RaftRuntime::AutoPub

#include "AutoPub/AutoPubDeviceSource.hpp"
