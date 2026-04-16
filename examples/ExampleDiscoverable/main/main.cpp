/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main entry point
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RaftCoreApp.h"
#include "RegisterSysMods.h"
#include "RegisterWebServer.h"
#include "MainSysMod.h"
#include "RaftROS.h"


// Create the app
RaftCoreApp raftCoreApp;

// Entry point
extern "C" void app_main(void)
{
    
    // Register SysMods from RaftSysMods library
    RegisterSysMods::registerSysMods(raftCoreApp.getSysManager());

    // Register WebServer from RaftWebServer library
    RegisterSysMods::registerWebServer(raftCoreApp.getSysManager());

    // Register RaftROS SysMod for ROS 2 node discovery
    raftCoreApp.registerSysMod("RaftROS", RaftROS::create, true);

    // Register sysmod
    raftCoreApp.registerSysMod("MainSysMod", MainSysMod::create, true);

    // Loop forever
    while (1)
    {
        // Loop the app
        raftCoreApp.loop();
    }
}
