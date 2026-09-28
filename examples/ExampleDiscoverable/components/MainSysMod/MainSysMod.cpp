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
    }
}

void MainSysMod::loop()
{
    // Check for loop rate
    if (Raft::isTimeout(millis(), _lastLoopMs, 1000))
    {
        // Update last loop time
        _lastLoopMs = millis();

        // Put some code here that will be executed once per second
        // ...
    }
}

