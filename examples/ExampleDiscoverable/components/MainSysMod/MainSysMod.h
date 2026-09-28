////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.h
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftArduino.h"
#include "RaftSysMod.h"

class MainSysMod : public RaftSysMod
{
public:
    MainSysMod(const char *pModuleName, RaftJsonIF& sysConfig);
    virtual ~MainSysMod();

    // Create function (for use by SysManager factory)
    static RaftSysMod* create(const char* pModuleName, RaftJsonIF& sysConfig)
    {
        return new MainSysMod(pModuleName, sysConfig);
    }

protected:

    // Setup
    virtual void setup() override final;

    // Loop (called frequently)
    virtual void loop() override final;

private:
    // Debug
    static constexpr const char *MODULE_PREFIX = "MainSysMod";

    // Example of how to control loop rate
    uint32_t _lastLoopMs = 0;

    // Receive counter — incremented on every chatter_in message so dropped
    // messages (gaps in the sequence) can be spotted in the serial log.
    uint32_t _rxCount = 0;

    // Separate receive counter for the second subscription (rt/chatter_in2).
    uint32_t _rxCount2 = 0;

    // Reply text for the /raft_esp32/devices service; encoded before the
    // handler returns, so one buffer serves every call
    char _serviceMsg[160] = {};
};
