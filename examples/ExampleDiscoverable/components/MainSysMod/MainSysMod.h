////////////////////////////////////////////////////////////////////////////////
//
// MainSysMod.h
//
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "RaftArduino.h"
#include "RaftSysMod.h"
#include "RaftDeviceConsts.h"
#include "RaftBusDevicesIF.h"

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

    // /raft_esp32/range: a deferred service.  The handler cannot read the
    // sensor itself (that is a bus transaction on the bus task), so it parks
    // the request and the loop completes it from the first poll result newer
    // than the call - a reading taken after the call, not a cached one.
    // No data callback: the bus holds one per device and the auto-publisher
    // owns it; the loop peeks the latest decoded result instead.
    RaftDeviceID _rangeDeviceID;                  ///< The VL6180, once identified
    bool _rangeAttached = false;
    bool _rangeRequestPending = false;
    uint32_t _rangeToken = 0;
    uint32_t _rangePollTimeAtRequest = 0;         ///< timeMs of the latest poll when the call arrived
    uint32_t _rangeRequestMs = 0;
    bool peekRange(struct poll_VL6180& poll);
    RaftBusDeviceDecodeState _rangeDecodeState;
    char _rangeMsg[96] = {};
    void serviceRangeRequest();
};
