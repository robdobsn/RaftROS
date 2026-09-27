/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubDeviceSource implementation - see AutoPubDeviceSource.h for the design
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "Logger.h"

// Diagnostic logging for the device pipeline.  Off by default: console writes
// block the calling task even with nothing reading the console, and the
// per-sample line alone was measured costing ~5% of auto-published samples on
// an ESP32-S3.  Attach, detach and every warning always log; these switches
// add the noise that is only useful while bringing something up.
// #define AUTOPUB_DEBUG_STATUS_CB     // every DeviceManager status callback
// #define AUTOPUB_DEBUG_SAMPLES       // one line per 100 published samples, per device

namespace RaftRuntime::AutoPub
{

static const char* AUTOPUB_SOURCE_PREFIX = "RaftROS";

// The CDR serialiser reads the decoded poll struct through a mirror of
// RaftCore's AttrFieldDesc layout, so it can stay out of RaftCore's include
// graph.  Check the layouts are still identical on this build.
static_assert(sizeof(AutoPubAttrFieldDesc) == sizeof(AttrFieldDesc),
              "AutoPubAttrFieldDesc must match AttrFieldDesc size");
static_assert(offsetof(AutoPubAttrFieldDesc, offset) == offsetof(AttrFieldDesc, offset),
              "AutoPubAttrFieldDesc offset mismatch");
static_assert(offsetof(AutoPubAttrFieldDesc, divisor) == offsetof(AttrFieldDesc, divisor),
              "AutoPubAttrFieldDesc divisor mismatch");
static_assert((uint8_t)AutoPubAttrType::Float == (uint8_t)AttrType::Float,
              "AutoPubAttrType enum ordering must match AttrType");
static_assert((uint8_t)AutoPubAttrType::Bool == (uint8_t)AttrType::Bool,
              "AutoPubAttrType enum ordering must match AttrType");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device status
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Backend, uint8_t CAPACITY>
void AutoPubDeviceSource<Backend, CAPACITY>::onDeviceStatusChange(
        RaftDevice& device, const BusAddrStatus& addrStatus)
{
#ifdef AUTOPUB_DEBUG_STATUS_CB
    LOG_I(AUTOPUB_SOURCE_PREFIX,
          "autoPubStatusCb devID=%s typeIdx=%u online=%d isChange=%d isNewlyId=%d",
          device.getDeviceID().toString().c_str(),
          (unsigned)addrStatus.deviceTypeIndex,
          (int)addrStatus.onlineState,
          (int)addrStatus.isChange,
          (int)addrStatus.isNewlyIdentified);
#endif

    // Act on either an online/offline transition (isChange) or on the
    // first-identification event for an already-online device
    // (isNewlyIdentified).  I2C devices are reported ONLINE as soon as the bus
    // scanner sees an ACK at the address; identification (and therefore a valid
    // deviceTypeIndex) happens asynchronously in a follow-up status change with
    // isNewlyIdentified=true but no online-state change.  Auto-publish needs
    // the typeIdx, so we cannot commit on isChange alone.
    if (!addrStatus.isChange && !addrStatus.isNewlyIdentified)
        return;

    switch (addrStatus.onlineState)
    {
        case DeviceOnlineState::ONLINE:
            // Skip attach until the bus has finished identification
            if (addrStatus.deviceTypeIndex == DEVICE_TYPE_INDEX_INVALID)
                return;
            attachDevice(device, addrStatus);
            break;
        case DeviceOnlineState::OFFLINE:
        case DeviceOnlineState::PENDING_DELETION:
            detachDevice(device);
            break;
        case DeviceOnlineState::INITIAL:
        default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// QoS overrides
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Backend, uint8_t CAPACITY>
void AutoPubDeviceSource<Backend, CAPACITY>::parseQoSOverrides(const char* qosProfilesJson)
{
    _aliasOverrides.clear();
    _classOverrides.clear();

    // Each top-level key is either the literal "classDefaults" (a nested
    // CLAS->profileName object) or a device alias mapped to a profile name,
    // given as a bare string or as {profile:"<name>"}.
    RaftJson blockJson(qosProfilesJson && *qosProfilesJson ? qosProfilesJson : "{}");
    std::vector<String> topKeys;
    blockJson.getKeys("", topKeys);
    for (const String& key : topKeys)
    {
        if (key == "classDefaults")
        {
            const String classRaw = blockJson.getString(key.c_str(), "{}");
            RaftJson classJson(classRaw);
            std::vector<String> classKeys;
            classJson.getKeys("", classKeys);
            for (const String& clasCode : classKeys)
            {
                const String profileName = classJson.getString(clasCode.c_str(), "");
                AutoPubQoSProfileId id = AutoPubQoSProfileId::FallbackString;
                if (!AutoPubQoSProfile_parseName(profileName.c_str(), id))
                {
                    LOG_W(AUTOPUB_SOURCE_PREFIX,
                          "qosProfiles.classDefaults.%s: unknown profile '%s' - ignored",
                          clasCode.c_str(), profileName.c_str());
                    continue;
                }
                _classOverrides.push_back({clasCode, id});
            }
            continue;
        }

        const String rawVal = blockJson.getString(key.c_str(), "");
        String profileName;
        if (rawVal.length() >= 2 && rawVal.charAt(0) == '{')
        {
            RaftJson obj(rawVal);
            profileName = obj.getString("profile", "");
        }
        else
        {
            profileName = rawVal;
        }
        AutoPubQoSProfileId id = AutoPubQoSProfileId::FallbackString;
        if (!AutoPubQoSProfile_parseName(profileName.c_str(), id))
        {
            LOG_W(AUTOPUB_SOURCE_PREFIX, "qosProfiles.%s: unknown profile '%s' - ignored",
                  key.c_str(), profileName.c_str());
            continue;
        }
        _aliasOverrides.push_back({key, id});
    }

    if (!_aliasOverrides.empty() || !_classOverrides.empty())
    {
        LOG_I(AUTOPUB_SOURCE_PREFIX, "autoPubQoS overrides parsed: aliases=%u classDefaults=%u",
              (unsigned)_aliasOverrides.size(), (unsigned)_classOverrides.size());
    }
}

template <typename Backend, uint8_t CAPACITY>
AutoPubQoSProfileId AutoPubDeviceSource<Backend, CAPACITY>::resolveQoSProfileId(
        const char* pTopicAlias, const char* const* pClasArray, size_t clasCount,
        const char* pDeviceTypeName) const
{
    // 1. Per-device alias
    if (pTopicAlias && *pTopicAlias)
    {
        for (const auto& override : _aliasOverrides)
        {
            if (override.key == pTopicAlias)
                return override.id;
        }
    }

    // 2. Per-class override - first class code that matches wins
    if (pClasArray)
    {
        for (size_t index = 0; index < clasCount; index++)
        {
            const char* clasCode = pClasArray[index];
            if (!clasCode || !*clasCode)
                continue;
            for (const auto& override : _classOverrides)
            {
                if (override.key == clasCode)
                    return override.id;
            }
        }
    }

    // 3. Built-in default from the class list
    return AutoPubQoSProfile_defaultForClasses(pClasArray, clasCount, pDeviceTypeName);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Attach
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Backend, uint8_t CAPACITY>
bool AutoPubDeviceSource<Backend, CAPACITY>::attachDevice(
        RaftDevice& device, const BusAddrStatus& addrStatus)
{
    if (!_pBackend)
        return false;
    const RaftDeviceID devID = device.getDeviceID();
    if (!devID.isValid() || addrStatus.deviceTypeIndex == DEVICE_TYPE_INDEX_INVALID)
    {
        LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubAttach invalid devID=%s typeIdx=%u - skipping",
              devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex);
        return false;
    }

    // Already attached?  DeviceManager re-emits online on reconnect after
    // offline, by which point we have detached.
    if (findSlot(devID) >= 0)
    {
        LOG_I(AUTOPUB_SOURCE_PREFIX, "autoPubAttach already attached devID=%s - skipping",
              devID.toString().c_str());
        return false;
    }

    // Look up the ROS 2 message mapping from the device's class tags.  Falls
    // back to String/"raw" for unknown classes; actuators are excluded.
    DeviceTypeRecord devTypeRec;
    const bool haveTypeRec = deviceTypeRecords.getDeviceInfo(addrStatus.deviceTypeIndex, devTypeRec);
    const char* pDeviceTypeName = haveTypeRec ? devTypeRec.deviceType : nullptr;

    // Parse the "clas" array out of devInfoJson.  Runs once per attach, so the
    // RaftJson parse cost is negligible.
    std::vector<String> clasStrs;
    if (haveTypeRec && devTypeRec.devInfoJson)
    {
        RaftJson devInfo(devTypeRec.devInfoJson, false);
        devInfo.getArrayElems("clas", clasStrs);
    }
    std::vector<const char*> clasPtrs;
    clasPtrs.reserve(clasStrs.size());
    for (const auto& clasStr : clasStrs)
        clasPtrs.push_back(clasStr.c_str());

    // Decide what this device publishes: class mapping -> ROS topic/type ->
    // semantic QoS.  The SysTypes QoS overrides are applied here because they
    // are SysMod configuration, not a property of the device.
    const AutoPubDeviceId planDeviceId{(uint8_t)devID.getBusNum(), (uint32_t)devID.getAddress(), 0};
    const auto plan = AutoPubAttachPlan_build(
        planDeviceId,
        clasPtrs.empty() ? nullptr : clasPtrs.data(), clasPtrs.size(), pDeviceTypeName,
        [this](const char* alias, const char* const* clasArray, size_t clasCount,
               const char* deviceTypeName) {
            return resolveQoSProfileId(alias, clasArray, clasCount, deviceTypeName);
        });

    if (plan.excluded)
    {
        LOG_I(AUTOPUB_SOURCE_PREFIX, "autoPubAttach devID=%s type=%s is an actuator - excluded from auto-publish",
              devID.toString().c_str(), pDeviceTypeName ? pDeviceTypeName : "?");
        return false;
    }
    if (plan.endpointCount == 0)
    {
        LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubAttach devID=%s type=%s - no publishable endpoint (name too long?)",
              devID.toString().c_str(), pDeviceTypeName ? pDeviceTypeName : "?");
        return false;
    }

    // Create the primary publisher on the selected backend
    const uint8_t slot = _pBackend->createPublisher(plan.endpoints[0]);
    if (slot == Backend::INVALID_SLOT || slot >= CAPACITY)
    {
        LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubAttach backend full - cannot attach devID=%s typeIdx=%u",
              devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex);
        return false;
    }

    // Build the per-device context: cache decode metadata and pre-allocate the
    // decode buffer (the bus callback runs on a task with a small stack).
    DeviceCtx* pCtx = new DeviceCtx();
    pCtx->deviceID = devID;
    pCtx->planDeviceId = planDeviceId;
    pCtx->deviceTypeIndex = addrStatus.deviceTypeIndex;
    pCtx->slot = slot;
    pCtx->msgKind = plan.endpoints[0].msgKind;

    if (haveTypeRec && devTypeRec.pollResultDecodeFn && devTypeRec.pollFieldDescs)
    {
        pCtx->decodeFn = devTypeRec.pollResultDecodeFn;
        pCtx->pFieldDescs = devTypeRec.pollFieldDescs;
        pCtx->fieldCount = devTypeRec.pollFieldCount;
        pCtx->structSize = devTypeRec.pollStructSize;
        pCtx->pollDataSizeBytes = devTypeRec.pollDataSizeBytes;

        // Up to 8 records per decode for FIFO-backed devices keeps the heap
        // allocation under 1 kB for typical struct sizes (IMU = 24 B).
        static constexpr uint16_t MAX_DECODE_RECORDS = 8;
        pCtx->maxDecodeRecords = MAX_DECODE_RECORDS;
        pCtx->decodeBufSize = (uint32_t)pCtx->structSize * MAX_DECODE_RECORDS;
        if (pCtx->decodeBufSize > 0)
            pCtx->pDecodeBuf = new uint8_t[pCtx->decodeBufSize];
    }
    else
    {
        LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubAttach decode unavailable devID=%s typeIdx=%u - callbacks will be rawlen-only",
              devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex);
    }

    // Claim a generation-safe mailbox handle for the bus data callback.
    // Without one the endpoint is still announced but publishes nothing.
    if (pCtx->pDecodeBuf)
    {
        pCtx->handle = _pool.acquire(pCtx, pCtx->structSize);
        if (!pCtx->handle.isValid())
        {
            LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubAttach no mailbox devID=%s structSize=%u (max %u, devices %u/%u) - endpoint will not publish",
                  devID.toString().c_str(), (unsigned)pCtx->structSize,
                  (unsigned)MAILBOX_RECORD_SIZE,
                  (unsigned)_pool.inUseCount(), (unsigned)_pool.capacity());
        }
    }

    _ctxs[slot] = pCtx;

    LOG_I(AUTOPUB_SOURCE_PREFIX, "autoPubAttach devID=%s typeIdx=%u slot=%u topic=%s type=%s qos=%s fields=%u structSize=%u",
          devID.toString().c_str(), (unsigned)addrStatus.deviceTypeIndex, (unsigned)slot,
          plan.endpoints[0].topic, plan.endpoints[0].type,
          AutoPubQoSProfile_name(plan.endpoints[0].qosProfileId),
          (unsigned)pCtx->fieldCount, (unsigned)pCtx->structSize);

    // Composite second endpoint: shares the decoded record and field
    // descriptors with the primary but gets its own publisher (distinct topic
    // and wire identity).  Failure is non-fatal - the primary keeps publishing.
    if (plan.endpointCount > 1)
    {
        const uint8_t secSlot = _pBackend->createPublisher(plan.endpoints[1]);
        if (secSlot == Backend::INVALID_SLOT || secSlot >= CAPACITY)
        {
            LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubAttach composite backend full - only primary attached devID=%s",
                  devID.toString().c_str());
        }
        else
        {
            pCtx->secondarySlot = secSlot;
            pCtx->secondaryMsgKind = plan.endpoints[1].msgKind;
            LOG_I(AUTOPUB_SOURCE_PREFIX, "autoPubAttach secondary devID=%s slot=%u topic=%s type=%s qos=%s",
                  devID.toString().c_str(), (unsigned)secSlot,
                  plan.endpoints[1].topic, plan.endpoints[1].type,
                  AutoPubQoSProfile_name(plan.endpoints[1].qosProfileId));
        }
    }

    // Install the per-device data callback.  DeviceManager forwards this to the
    // underlying bus, and callbacks run on the bus polling task.  The callback
    // info is the encoded pool handle, never the context pointer, so a callback
    // that races with (or outlives) detach cannot reach freed state.
    DeviceManager* pDevMan = _pSysManager ? _pSysManager->getDeviceManager() : nullptr;
    if (pDevMan && pCtx->handle.isValid())
    {
        pDevMan->registerForDeviceData(
            devID,
            [this](uint16_t /*deviceTypeIdx*/, std::vector<uint8_t> data, const void* pCallbackInfo) {
                this->onDeviceData(std::move(data), pCallbackInfo);
            },
            /*minTimeBetweenReportsMs=*/0,
            /*pCallbackInfo=*/pCtx->handle.toCallbackInfo(),
            /*unregister=*/false);
    }

    if (_onAttached)
        _onAttached(planDeviceId, slot, pCtx->secondarySlot);
    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Detach
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Backend, uint8_t CAPACITY>
void AutoPubDeviceSource<Backend, CAPACITY>::detachDevice(RaftDevice& device)
{
    if (!_pBackend)
        return;
    const RaftDeviceID devID = device.getDeviceID();
    const int slot = findSlot(devID);
    if (slot < 0)
        return;

    DeviceCtx* pCtx = _ctxs[slot];
    _ctxs[slot] = nullptr;

    // Drop the DeviceManager data registration.  This only removes the pending
    // request: a registration already forwarded to the bus is not withdrawn,
    // and a callback may be in flight on the bus task right now.  Releasing the
    // pool handle is what makes freeing the context safe - it waits for any
    // producer holding the lock, and every later callback carrying the old
    // handle is dropped as stale without touching the context.
    DeviceManager* pDevMan = _pSysManager ? _pSysManager->getDeviceManager() : nullptr;
    if (pDevMan && pCtx && pCtx->handle.isValid())
    {
        pDevMan->registerForDeviceData(devID, nullptr, 0, pCtx->handle.toCallbackInfo(),
                                       /*unregister=*/true);
    }
    if (pCtx)
        _pool.release(pCtx->handle);

    // The transport may need the endpoints' live identity to withdraw them
    // (RTPS disposes each writer at each peer), so this runs before they go.
    const uint8_t secondarySlot = pCtx ? pCtx->secondarySlot : INVALID_SLOT;
    const AutoPubDeviceId planDeviceId = pCtx ? pCtx->planDeviceId : AutoPubDeviceId{};
    if (_onDetaching)
        _onDetaching(planDeviceId, (uint8_t)slot, secondarySlot);

    _pBackend->destroyPublisher((uint8_t)slot);
    if (secondarySlot != INVALID_SLOT)
        _pBackend->destroyPublisher(secondarySlot);

    LOG_I(AUTOPUB_SOURCE_PREFIX, "autoPubDetach devID=%s slot=%d secSlot=%d samples=%u",
          devID.toString().c_str(), slot, (int)(int8_t)secondarySlot,
          pCtx ? (unsigned)pCtx->sampleCount : 0u);
    delete pCtx;

    if (_onDetached)
        _onDetached(planDeviceId, (uint8_t)slot, secondarySlot);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bus data callback - runs on the bus worker task.  Decodes under the pool lock
// and stores only the latest record; no serialisation and no network I/O.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Backend, uint8_t CAPACITY>
void AutoPubDeviceSource<Backend, CAPACITY>::onDeviceData(
        std::vector<uint8_t> data, const void* pCallbackInfo)
{
    // The callback info is an encoded handle; a stale one (device detached,
    // slot reused) is rejected by the pool before the context is ever reached.
    const AutoPubPublisherHandle handle = AutoPubPublisherHandle::fromCallbackInfo(pCallbackInfo);
    _pool.produce(handle,
        [&](void* pUser, uint8_t* pRecord, uint32_t recordSize) -> uint32_t {
            DeviceCtx* pCtx = static_cast<DeviceCtx*>(pUser);
            pCtx->sampleCount++;

            // Diagnostics: gap since this device's previous data callback
            const int64_t nowUs = esp_timer_get_time();
            if (pCtx->lastCallbackUs != 0)
            {
                const int64_t gapMs = (nowUs - pCtx->lastCallbackUs) / 1000;
                const uint8_t bucket = gapMs < 50 ? 0 : gapMs < 150 ? 1 : gapMs < 250 ? 2 : gapMs < 400 ? 3 : 4;
                pCtx->cbGapHist[bucket].fetch_add(1, std::memory_order_relaxed);
            }
            pCtx->lastCallbackUs = nowUs;
            if (!(pCtx->decodeFn && pCtx->pDecodeBuf && pCtx->structSize > 0 &&
                  pCtx->structSize == recordSize))
                return 0;

            // Pad up to the fixed record stride the generated decoder expects
            const uint32_t expectedRecordSize = pCtx->pollDataSizeBytes
                                              + DevicePollingInfo::POLL_RESULT_TIMESTAMP_SIZE;
            if (data.size() < expectedRecordSize)
                data.resize(expectedRecordSize, 0);

            const uint32_t numRecords = pCtx->decodeFn(data.data(), data.size(),
                                                       pCtx->pDecodeBuf, pCtx->decodeBufSize,
                                                       pCtx->maxDecodeRecords, pCtx->decodeState);
            // Keep only the latest record (FIFO latest-only); check the
            // decoder's count before forming the pointer.
            if (numRecords == 0 || numRecords > pCtx->decodeBufSize / pCtx->structSize)
                return 0;
            std::memcpy(pRecord, pCtx->pDecodeBuf + (numRecords - 1) * pCtx->structSize,
                        pCtx->structSize);
            return pCtx->structSize;
        },
        PRODUCER_LOCK_TIMEOUT_MS);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Mailbox drain - runs on the loop task, which also owns attach and detach
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Backend, uint8_t CAPACITY>
void AutoPubDeviceSource<Backend, CAPACITY>::drainSamples()
{
    if (!_pBackend)
        return;

    // Diagnostics: gap between drain passes (an overwrite means a sample was
    // replaced before the loop got back here)
    const int64_t nowUs = esp_timer_get_time();
    if (_lastDrainUs != 0)
    {
        const uint32_t gapUs = (uint32_t)(nowUs - _lastDrainUs);
        if (gapUs > _drainGapMaxUs)
            _drainGapMaxUs = gapUs;
        if (gapUs > 150000)
            _drainGapsOver150ms++;
    }
    _lastDrainUs = nowUs;

    _pool.drain([&](const AutoPubDrainedSample& sample) {
        // Safe: release() and the delete of the context also run on this task
        const DeviceCtx* pCtx = static_cast<const DeviceCtx*>(sample.pUser);

        AutoPubDecodedBatch batch;
        batch.data = sample.record;
        batch.capacity = sample.length;
        batch.recordSize = pCtx->structSize;
        batch.recordCount = 1;
        batch.fields = reinterpret_cast<const AutoPubAttrFieldDesc*>(pCtx->pFieldDescs);
        batch.fieldCount = pCtx->fieldCount;
        const AutoPubSampleOutput outputs[] = {
            {pCtx->msgKind, _cdrBufs[0], CDR_BUF_SIZE},
            {pCtx->secondaryMsgKind,
             pCtx->secondarySlot == INVALID_SLOT ? nullptr : _cdrBufs[1],
             CDR_BUF_SIZE}
        };
        AutoPubSampleResult results[2];
        uint64_t emissionSeq[2] = {0, 0};
        uint32_t emissionPeers[2] = {0, 0};
        auto publish = [&](uint8_t outputIndex, const uint8_t* payload, uint32_t length, uint32_t) {
            const uint8_t slot = outputIndex == 0 ? pCtx->slot : pCtx->secondarySlot;
            return _pBackend->publish(slot, payload, length,
                                      &emissionSeq[outputIndex], &emissionPeers[outputIndex]);
        };

        const bool batchOK = AutoPubSampleRunner::run(batch, outputs, 2, results, publish);

        // Rate-limit the per-sample log on the mailbox's stored-record count
        if (sample.produced > 3 && (sample.produced % 100) != 0)
            return;
        if (!batchOK)
        {
            LOG_W(AUTOPUB_SOURCE_PREFIX, "autoPubData devID=%s invalid mailbox record len=%u stride=%u fields=%u (sample #%u)",
                  pCtx->deviceID.toString().c_str(), (unsigned)sample.length,
                  (unsigned)pCtx->structSize, (unsigned)pCtx->fieldCount,
                  (unsigned)sample.produced);
        }
#ifdef AUTOPUB_DEBUG_SAMPLES
        else if (pCtx->secondarySlot == INVALID_SLOT)
        {
            LOG_I(AUTOPUB_SOURCE_PREFIX,
                  "autoPubData devID=%s typeIdx=%u slot=%u cdrBytes=%u serOK=%d seq=%u peers=%u pub=%u overwritten=%u cbGaps<50/150/250/400/+=%u/%u/%u/%u/%u (sample #%u)",
                  pCtx->deviceID.toString().c_str(), (unsigned)pCtx->deviceTypeIndex,
                  (unsigned)pCtx->slot,
                  (unsigned)results[0].bytesWritten, (int)results[0].serialized,
                  (unsigned)emissionSeq[0], (unsigned)emissionPeers[0],
                  (unsigned)results[0].publishResult,
                  (unsigned)sample.overwritten,
                  (unsigned)pCtx->cbGapHist[0].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[1].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[2].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[3].load(std::memory_order_relaxed),
                  (unsigned)pCtx->cbGapHist[4].load(std::memory_order_relaxed),
                  (unsigned)sample.produced);
        }
        else
        {
            LOG_I(AUTOPUB_SOURCE_PREFIX,
                  "autoPubData devID=%s typeIdx=%u slot=%u/%u priCDR=%u serOK=%d seq=%u peers=%u pub=%u | secCDR=%u secOK=%d secSeq=%u secPeers=%u pub=%u overwritten=%u (sample #%u)",
                  pCtx->deviceID.toString().c_str(), (unsigned)pCtx->deviceTypeIndex,
                  (unsigned)pCtx->slot, (unsigned)pCtx->secondarySlot,
                  (unsigned)results[0].bytesWritten, (int)results[0].serialized,
                  (unsigned)emissionSeq[0], (unsigned)emissionPeers[0],
                  (unsigned)results[0].publishResult,
                  (unsigned)results[1].bytesWritten, (int)results[1].serialized,
                  (unsigned)emissionSeq[1], (unsigned)emissionPeers[1],
                  (unsigned)results[1].publishResult,
                  (unsigned)sample.overwritten, (unsigned)sample.produced);
        }
#endif
    });
}

} // namespace RaftRuntime::AutoPub
