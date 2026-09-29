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
#include "DevicePollRecords_generated.h"

static constexpr const char* RANGE_DEVICE_TYPE = "VL6180";

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
            //   ros2 service call /raft_esp32/devices std_srvs/srv/Trigger
            pRaftROS->addService("/raft_esp32/devices", "std_srvs::srv::dds_::Trigger_",
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
            //   ros2 service call /raft_esp32/range std_srvs/srv/Trigger
            // Deferred: answered by loop() from the next poll result; if none
            // comes (sensor unplugged) the SysMod answers an error at the timeout
            pRaftROS->addService("/raft_esp32/range", "std_srvs::srv::dds_::Trigger_",
                [this](const RaftROS::ServiceRequest& request, RaftROS::ServiceReply& reply)
                {
                    if (_rangeRequestPending)
                    {
                        reply.reason = "a range read is already in progress";
                        return RaftROS::ServiceOutcome::Refused;
                    }
                    _rangeToken = request.token;
                    _rangeSeqAtRequest = _rangeSampleSeq.load();
                    _rangeRequestMs = millis();
                    _rangeRequestPending = true;
                    return RaftROS::ServiceOutcome::Deferred;
                });
            // A parameter of the application's own: read where it is used
            // (serviceRangeRequest), so no callback is needed
            //   ros2 param set /raft_esp32 rangeOffsetMm 4.5
            pRaftROS->declareParameter("rangeOffsetMm", 0.0, "Added to every /raft_esp32/range reading, in mm");
            //   ros2 service call /raft_esp32/ping std_srvs/srv/Empty
            pRaftROS->addService("/raft_esp32/ping", "std_srvs::srv::dds_::Empty_",
                [](const RaftROS::ServiceRequest&, RaftROS::ServiceReply&)
                {
                    return RaftROS::ServiceOutcome::Replied;
                });
            //   ros2 service call /raft_esp32/chatter_enable std_srvs/srv/SetBool "{data: false}"
            pRaftROS->addService("/raft_esp32/chatter_enable", "std_srvs::srv::dds_::SetBool_",
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

        // The range service's view of the sensor: which device it is (status
        // callbacks) and when a poll result has landed (data callbacks, bus task)
        DeviceManager* pDevMan = getSysManager()->getDeviceManager();
        if (pDevMan)
        {
            pDevMan->registerForDeviceStatusChange(
                [this](RaftDevice& device, const BusAddrStatus& addrStatus)
                {
                    DeviceTypeRecord devTypeRec;
                    if (!deviceTypeRecords.getDeviceInfo(addrStatus.deviceTypeIndex, devTypeRec) ||
                        !devTypeRec.deviceType || strcmp(devTypeRec.deviceType, RANGE_DEVICE_TYPE) != 0)
                        return;
                    if (addrStatus.onlineState == DeviceOnlineState::ONLINE)
                    {
                        _rangeDeviceID = device.getDeviceID();
                        _rangeAttached = true;
                    }
                    else if (addrStatus.isChange)
                        _rangeAttached = false;
                });
            pDevMan->registerForDeviceData(RANGE_DEVICE_TYPE,
                [this](uint16_t, std::vector<uint8_t>, const void*) { _rangeSampleSeq.fetch_add(1); },
                /*minTimeBetweenReportsMs=*/0);
        }
    }
}

/// @brief Complete a parked /raft_esp32/range request once a poll result has
/// arrived since it was made.  Loop task only - completeService requires it.
void MainSysMod::serviceRangeRequest()
{
    if (!_rangeRequestPending)
        return;
    RaftSysMod* pRos = getSysManager() ? getSysManager()->getSysMod("RaftROS") : nullptr;
    RaftROS* pRaftROS = static_cast<RaftROS*>(pRos);
    if (!pRaftROS)
    {
        _rangeRequestPending = false;
        return;
    }
    if (_rangeSampleSeq.load() == _rangeSeqAtRequest)
    {
        // Nothing new from the bus yet.  RaftROS answers the client with an
        // error at the timeout; drop our side once that has passed.
        if (Raft::isTimeout(millis(), _rangeRequestMs, 10000))
            _rangeRequestPending = false;
        return;
    }
    RaftROS::ServiceReply reply;
    poll_VL6180 poll = {};
    RaftBus* pBus = _rangeAttached ? raftBusSystem.getBusByNumber(_rangeDeviceID.getBusNum()) : nullptr;
    RaftBusDevicesIF* pDevicesIF = pBus ? pBus->getBusDevicesIF() : nullptr;
    if (pDevicesIF && pDevicesIF->getLatestDecodedPollResponse(_rangeDeviceID.getAddress(),
                                                               &poll, sizeof(poll), _rangeDecodeState))
    {
        // The rangeOffsetMm parameter (ros2 param set /raft_esp32 rangeOffsetMm 4.5)
        const RaftROS::ParamValue* pOffset = pRaftROS->parameter("rangeOffsetMm");
        const double offsetMm = pOffset ? pOffset->doubleValue : 0.0;
        snprintf(_rangeMsg, sizeof(_rangeMsg), "range %.1f mm (offset %.1f) valid=%d, read %u ms after the call",
                 (double)poll.dist + offsetMm, offsetMm, (int)poll.valid, (unsigned)(millis() - _rangeRequestMs));
        reply.fields.success = true;
    }
    else
    {
        snprintf(_rangeMsg, sizeof(_rangeMsg), "poll result arrived but could not be decoded");
        reply.fields.success = false;
    }
    reply.fields.message = _rangeMsg;
    pRaftROS->completeService(_rangeToken, reply);   // false if it already timed out
    _rangeRequestPending = false;
}

void MainSysMod::loop()
{
    serviceRangeRequest();

    // Check for loop rate
    if (Raft::isTimeout(millis(), _lastLoopMs, 1000))
    {
        // Update last loop time
        _lastLoopMs = millis();

        // Put some code here that will be executed once per second
        // ...
    }
}

