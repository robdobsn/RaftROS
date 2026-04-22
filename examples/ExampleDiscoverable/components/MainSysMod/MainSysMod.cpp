////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.cpp
//
////////////////////////////////////////////////////////////////////////////////

#include "MainSysMod.h"
#include "RaftUtils.h"
#include "SysManager.h"
#include "RaftROS.h"

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
    // The following code is an example of how to use the config object to
    // get a parameter from SysType (JSON) file for this system module
    // Replace this with your own setup code
    String configValue = config.getString("exampleGroup/exampleKey", "This Should Not Happen!");
    LOG_I(MODULE_PREFIX, "%s", configValue.c_str());

    // Hook the RaftROS /chatter_in handler so application code receives decoded
    // std_msgs/String messages. This runs after SysMods have been created, so
    // the RaftROS instance is available by name lookup.
    if (getSysManager())
    {
        RaftSysMod* pRos = getSysManager()->getSysMod("RaftROS");
        RaftROS* pRaftROS = static_cast<RaftROS*>(pRos);
        if (pRaftROS)
        {
            pRaftROS->setStringMessageHandler(
                [](const uint8_t* writerEID, const uint8_t* srcGuid,
                   const char* text, uint32_t textLen)
                {
                    LOG_I("MainSysMod",
                          "chatter_in received writerEID=%02X%02X%02X%02X src=%02X%02X%02X%02X... \"%s\" (%u chars)",
                          writerEID[0], writerEID[1], writerEID[2], writerEID[3],
                          srcGuid[0], srcGuid[1], srcGuid[2], srcGuid[3],
                          text, (unsigned)textLen);
                });
            LOG_I(MODULE_PREFIX, "Registered /chatter_in string message handler");
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

