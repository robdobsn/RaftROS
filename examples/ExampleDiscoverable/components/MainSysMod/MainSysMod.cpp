////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "MainSysMod.h"
#include "RaftUtils.h"
#include "SysManager.h"
#include "RaftROS.h"
#include "esp_system.h"
#include "DeviceManager.h"
#include "DeviceTypeRecords.h"
#include "RaftBusSystem.h"
#include "DeviceTypeRecord.h"
#include <algorithm>

MainSysMod::MainSysMod(const char *pModuleName, RaftJsonIF& sysConfig)
    : RaftSysMod(pModuleName, sysConfig)
{
    // This code is executed when the system module is created
    // ...
}

MainSysMod::~MainSysMod()
{
    // This code is executed when the system module is destroyed
    // ...
}

void MainSysMod::setup()
{
    // Hook the RaftROS /chatter_in handler so application code receives decoded
    // std_msgs/String messages. This runs after SysMods have been created, so
    // the RaftROS instance is available by name lookup.
    if (getSysManager())
    {
        RaftSysMod* pRos = getSysManager()->getSysMod("RaftROS");
        RaftROS* pRaftROS = static_cast<RaftROS*>(pRos);
        if (pRaftROS)
        {
            // Register per-topic subscription for /chatter_in using the new
            // addStringSubscription API.  The handler is correlated to the
            // remote writer via the SEDP publication map so that additional
            // subscriptions (added later) each receive only their own messages.
            pRaftROS->addStringSubscription(
                "rt/chatter_in",
                "std_msgs::msg::dds_::String_",
                [this](const uint8_t* writerEID, const uint8_t* srcGuid,
                       const char* text, uint32_t textLen)
                {
                    LOG_I("MainSysMod",
                          "chatter_in #%u writerEID=%02X%02X%02X%02X src=%02X%02X%02X%02X... \"%s\" (%u chars)",
                          (unsigned)++_rxCount,
                          writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                          srcGuid[0], srcGuid[1], srcGuid[2], srcGuid[3],
                          text, (unsigned)textLen);
                });
            LOG_I(MODULE_PREFIX, "Registered /chatter_in string message handler (per-topic slot)");

            // Second subscription on a different topic, routed to its own slot.
            // Test from ROS 2 host with:
            //   ros2 topic pub /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"
            pRaftROS->addStringSubscription(
                "rt/chatter_in2",
                "std_msgs::msg::dds_::String_",
                [this](const uint8_t* writerEID, const uint8_t* srcGuid,
                       const char* text, uint32_t textLen)
                {
                    LOG_I("MainSysMod",
                          "chatter_in2 #%u writerEID=%02X%02X%02X%02X src=%02X%02X%02X%02X... \"%s\" (%u chars)",
                          (unsigned)++_rxCount2,
                          writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                          srcGuid[0], srcGuid[1], srcGuid[2], srcGuid[3],
                          text, (unsigned)textLen);
                });
            LOG_I(MODULE_PREFIX, "Registered /chatter_in2 string message handler (per-topic slot)");

            // Services (Zenoh build).  A handler runs on the main loop and
            // answers at once from state it already holds - nothing here waits.
            // "~/" names are relative to the node, whatever it is called.
            //   ros2 service call /<node>/devices std_srvs/srv/Trigger
            pRaftROS->addService("~/devices", "std_srvs::srv::dds_::Trigger_",
                [this, pRaftROS](const RaftROS::ServiceRequest&, RaftROS::ServiceReply& reply)
                {
                    snprintf(_serviceMsg, sizeof(_serviceMsg),
                             "%u device(s) attached, chatter %s, chatter_in rx %u/%u, heap free %u B",
                             (unsigned)pRaftROS->attachedDeviceCount(),
                             pRaftROS->isChatterEnabled() ? "on" : "off",
                             (unsigned)_rxCount, (unsigned)_rxCount2,
                             (unsigned)esp_get_free_heap_size());
                    reply.fields.success = true;
                    reply.fields.message = _serviceMsg;
                    return RaftROS::ServiceOutcome::Replied;
                });
            //   ros2 service call /<node>/read std_srvs/srv/Trigger
            // Deferred: answered by loop() once every attached device has a
            // reading newer than the call (see serviceReadRequest)
            pRaftROS->addService("~/read", "std_srvs::srv::dds_::Trigger_",
                [this](const RaftROS::ServiceRequest& request, RaftROS::ServiceReply& reply)
                {
                    if (_readRequestPending)
                    {
                        reply.reason = "a read is already in progress";
                        return RaftROS::ServiceOutcome::Refused;
                    }
                    if (attachedCount() == 0)
                    {
                        reply.fields.success = true;
                        reply.fields.message = "no devices attached";
                        return RaftROS::ServiceOutcome::Replied;
                    }
                    {
                        std::lock_guard<std::mutex> lock(_dataMutex);
                        for (AttachedDevice& device : _attachedDevices)
                            device.samplesAtRequest = device.samples;
                    }
                    _readToken = request.token;
                    _readRequestMs = millis();
                    _readRequestPending = true;
                    return RaftROS::ServiceOutcome::Deferred;
                });
            // A parameter of the application's own, with a callback that
            // refuses values out of range; read where it is used
            //   ros2 param set /<node> readTimeoutMs 1000
            pRaftROS->declareParameter("readTimeoutMs", (int64_t)READ_TIMEOUT_DEFAULT_MS,
                "How long ~/read waits for a fresh reading from every device, in ms", false,
                [](const RaftROS::ParamValue& value, const char*& reason)
                {
                    if (value.integerValue < 100 || value.integerValue > 4500)
                    {
                        reason = "readTimeoutMs must be 100-4500 (RaftROS times a deferred request out at 5 s)";
                        return false;
                    }
                    return true;
                });
            //   ros2 service call /<node>/ping std_srvs/srv/Empty
            pRaftROS->addService("~/ping", "std_srvs::srv::dds_::Empty_",
                [](const RaftROS::ServiceRequest&, RaftROS::ServiceReply&)
                {
                    return RaftROS::ServiceOutcome::Replied;
                });
            //   ros2 service call /<node>/chatter_enable std_srvs/srv/SetBool "{data: false}"
            pRaftROS->addService("~/chatter_enable", "std_srvs::srv::dds_::SetBool_",
                [pRaftROS](const RaftROS::ServiceRequest& request, RaftROS::ServiceReply& reply)
                {
                    pRaftROS->setChatterEnabled(request.fields.data);
                    reply.fields.success = true;
                    reply.fields.message = request.fields.data ? "chatter on" : "chatter off";
                    return RaftROS::ServiceOutcome::Replied;
                });
        }
        else
        {
            LOG_W(MODULE_PREFIX, "RaftROS SysMod not found - cannot register handler");
        }

        // The devices ~/read reports: every identified device, of any type
        // (status callbacks are a list, so this does not displace anyone else's)
        DeviceManager* pDevMan = getSysManager()->getDeviceManager();
        if (pDevMan)
        {
            pDevMan->registerForDeviceStatusChange(
                [this](RaftDevice& device, const BusAddrStatus& addrStatus)
                {
                    onDeviceStatusChange(device, addrStatus);
                });
        }
    }
}

/// @brief Keep the table of identified devices current, and subscribe to
/// each one's data.  Runs on the loop task.
void MainSysMod::onDeviceStatusChange(RaftDevice& device, const BusAddrStatus& addrStatus)
{
    DeviceManager* pDevMan = getSysManager() ? getSysManager()->getDeviceManager() : nullptr;
    if (!pDevMan)
        return;
    const RaftDeviceID deviceID = device.getDeviceID();
    AttachedDevice* pKnown = nullptr;
    AttachedDevice* pFree = nullptr;
    for (AttachedDevice& slot : _attachedDevices)
    {
        if (slot.used && slot.deviceID == deviceID)
            pKnown = &slot;
        else if (!slot.used && !pFree)
            pFree = &slot;
    }
    if (addrStatus.onlineState == DeviceOnlineState::ONLINE &&
        addrStatus.deviceTypeIndex != DEVICE_TYPE_INDEX_INVALID)
    {
        if (pKnown || !pFree)
            return;
        {
            std::lock_guard<std::mutex> lock(_dataMutex);
            pFree->used = true;
            pFree->key = _nextDeviceKey++;
            pFree->deviceID = deviceID;
            pFree->deviceTypeIndex = addrStatus.deviceTypeIndex;
            pFree->samples = pFree->samplesAtRequest = 0;
            pFree->latest.clear();
            pFree->decodeState = RaftBusDeviceDecodeState();
        }
        pDevMan->registerForDeviceData(deviceID,
            [this](uint16_t /*deviceTypeIdx*/, std::vector<uint8_t> data, const void* pCallbackInfo)
            {
                onDeviceData(std::move(data), pCallbackInfo);
            },
            0, reinterpret_cast<const void*>(pFree->key));
    }
    else if (pKnown && addrStatus.isChange)
    {
        pDevMan->registerForDeviceData(deviceID, nullptr, 0, reinterpret_cast<const void*>(pKnown->key), true);
        std::lock_guard<std::mutex> lock(_dataMutex);
        pKnown->used = false;
        pKnown->latest.clear();
    }
}

/// @brief A device's poll data.  Runs on the bus task: keep the latest and
/// count it, nothing more.  A callback for a device that has gone finds no
/// slot with its key and is dropped.
void MainSysMod::onDeviceData(std::vector<uint8_t>&& data, const void* pCallbackInfo)
{
    const uintptr_t key = reinterpret_cast<uintptr_t>(pCallbackInfo);
    std::lock_guard<std::mutex> lock(_dataMutex);
    for (AttachedDevice& slot : _attachedDevices)
    {
        if (slot.used && slot.key == key)
        {
            slot.latest = std::move(data);
            ++slot.samples;
            return;
        }
    }
}

uint8_t MainSysMod::attachedCount() const
{
    uint8_t count = 0;
    for (const AttachedDevice& slot : _attachedDevices)
        count += slot.used ? 1 : 0;
    return count;
}

/// @brief Append "<type> <bus>_<addr>: name=value ..." for a device's latest
/// sample, decoded with its type's decoder and listed from its field table.
/// Values are scaled by the table's divisor and addend, as RaftROS does for
/// the device's topic, so the two agree.  Loop task.
void MainSysMod::appendReading(AttachedDevice& device, bool fresh, String& out)
{
    DeviceTypeRecord devTypeRec;
    if (!deviceTypeRecords.getDeviceInfo(device.deviceTypeIndex, devTypeRec))
        return;
    out += String(out.length() ? "; " : "") + (devTypeRec.deviceType ? devTypeRec.deviceType : "?") +
           " " + device.deviceID.toString() + ":";
    std::vector<uint8_t> data;
    if (fresh)
    {
        std::lock_guard<std::mutex> lock(_dataMutex);
        data = device.latest;
    }
    if (!fresh || data.empty() || !devTypeRec.pollResultDecodeFn || !devTypeRec.pollFieldDescs ||
        devTypeRec.pollStructSize == 0)
    {
        out += " no new reading";
        return;
    }
    // A sample may hold several records (a FIFO read); the last is the newest
    const uint32_t maxRecords = std::max<uint32_t>(1, std::min<uint32_t>(16, 4096 / devTypeRec.pollStructSize));
    if (_decodeBuf.size() < maxRecords * devTypeRec.pollStructSize)
        _decodeBuf.resize(maxRecords * devTypeRec.pollStructSize);
    const uint32_t numRecords = devTypeRec.pollResultDecodeFn(data.data(), data.size(), _decodeBuf.data(),
                                    _decodeBuf.size(), maxRecords, device.decodeState);
    if (numRecords == 0)
    {
        out += " undecodable sample";
        return;
    }
    const uint8_t* pRecord = _decodeBuf.data() + (numRecords - 1) * devTypeRec.pollStructSize;
    for (uint16_t index = 0; index < devTypeRec.pollFieldCount; ++index)
    {
        const AttrFieldDesc& field = devTypeRec.pollFieldDescs[index];
        const uint8_t* pField = pRecord + field.offset;
        double value = 0;
        switch (field.type)
        {
            case AttrType::Float:  { float v;    memcpy(&v, pField, sizeof(v)); value = v; break; }
            case AttrType::Int32:  { int32_t v;  memcpy(&v, pField, sizeof(v)); value = v; break; }
            case AttrType::Uint32: { uint32_t v; memcpy(&v, pField, sizeof(v)); value = v; break; }
            case AttrType::Int16:  { int16_t v;  memcpy(&v, pField, sizeof(v)); value = v; break; }
            case AttrType::Uint16: { uint16_t v; memcpy(&v, pField, sizeof(v)); value = v; break; }
            case AttrType::Int8:   value = (int8_t)*pField; break;
            case AttrType::Uint8:
            case AttrType::Bool:   value = *pField; break;
        }
        if (field.divisor != 0.0f && field.divisor != 1.0f)
            value /= field.divisor;
        value += field.addend;
        char text[48];
        // An array attribute (a multizone frame, say) shows its first element and its size
        if (field.count > 1)
            snprintf(text, sizeof(text), " %s[%u]=%.4g..", field.name, (unsigned)field.count, value);
        else
            snprintf(text, sizeof(text), " %s=%.4g", field.name, value);
        out += text;
    }
}

/// @brief Complete a parked ~/read once every attached device has delivered
/// a sample since the call, or at readTimeoutMs with what has arrived.  Loop
/// task only - completeService requires it.
void MainSysMod::serviceReadRequest()
{
    if (!_readRequestPending)
        return;
    RaftROS* pRaftROS = getRaftROS();
    if (!pRaftROS)
    {
        _readRequestPending = false;
        return;
    }
    bool allFresh = true;
    {
        std::lock_guard<std::mutex> lock(_dataMutex);
        for (const AttachedDevice& device : _attachedDevices)
            if (device.used && device.samples == device.samplesAtRequest)
                allFresh = false;
    }
    const RaftROS::ParamValue* pTimeout = pRaftROS->parameter("readTimeoutMs");
    const uint32_t timeoutMs = pTimeout ? (uint32_t)pTimeout->integerValue : (uint32_t)READ_TIMEOUT_DEFAULT_MS;
    if (!allFresh && !Raft::isTimeout(millis(), _readRequestMs, timeoutMs))
        return;

    _readReply = "";
    bool allReported = true;
    for (AttachedDevice& device : _attachedDevices)
    {
        if (!device.used)
            continue;
        bool fresh = false;
        {
            std::lock_guard<std::mutex> lock(_dataMutex);
            fresh = device.samples != device.samplesAtRequest;
        }
        allReported = allReported && fresh;
        appendReading(device, fresh, _readReply);
    }
    if (_readReply.length() == 0)
        _readReply = "no devices attached";
    static const unsigned MAX_REPLY_CHARS = 450;
    if (_readReply.length() > MAX_REPLY_CHARS)
        _readReply = _readReply.substring(0, MAX_REPLY_CHARS - 2) + "..";
    RaftROS::ServiceReply reply;
    reply.fields.success = allReported;
    reply.fields.message = _readReply.c_str();
    pRaftROS->completeService(_readToken, reply);   // false if it already timed out
    _readRequestPending = false;
}

/// @brief The RaftROS SysMod, or nullptr if it is not running
RaftROS* MainSysMod::getRaftROS()
{
    RaftSysMod* pRos = getSysManager() ? getSysManager()->getSysMod("RaftROS") : nullptr;
    return static_cast<RaftROS*>(pRos);
}

void MainSysMod::loop()
{
    serviceReadRequest();

    // Check for loop rate
    if (Raft::isTimeout(millis(), _lastLoopMs, 1000))
    {
        // Update last loop time
        _lastLoopMs = millis();

        // Put some code here that will be executed once per second
        // ...
    }
}

