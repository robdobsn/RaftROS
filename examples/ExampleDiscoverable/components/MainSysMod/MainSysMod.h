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
#include <mutex>
#include <vector>

class RaftROS;

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

    // Reply text for the ~/devices service; encoded before the handler
    // returns, so one buffer serves every call
    char _serviceMsg[160] = {};

    // ~/read: a deferred service that answers with a fresh reading from every
    // attached device, whatever it is.  The handler cannot read a device
    // itself (that is a bus transaction on the bus task), so it notes how many
    // samples each device has delivered and parks the request; the loop
    // completes it once every device has delivered another (or at the
    // readTimeoutMs parameter, reporting any that had not).  Samples arrive
    // through DeviceManager's per-device data callbacks, which fan out to
    // every subscriber, so auto-publishing is not disturbed; each is decoded
    // with the device type record's own decoder and field table, so nothing
    // here knows about any particular device.
    struct AttachedDevice
    {
        bool used = false;
        uintptr_t key = 0;                  ///< callback info identifying this registration
        RaftDeviceID deviceID;
        uint16_t deviceTypeIndex = 0;
        uint32_t samples = 0;               ///< samples delivered (bus task writes, under _dataMutex)
        std::vector<uint8_t> latest;        ///< latest raw poll data (bus task writes, under _dataMutex)
        uint32_t samplesAtRequest = 0;
        RaftBusDeviceDecodeState decodeState;
    };
    static const uint8_t MAX_ATTACHED_DEVICES = 8;
    AttachedDevice _attachedDevices[MAX_ATTACHED_DEVICES];
    uintptr_t _nextDeviceKey = 1;
    std::mutex _dataMutex;
    std::vector<uint8_t> _decodeBuf;        ///< decoded poll structs, reused
    bool _readRequestPending = false;
    uint32_t _readToken = 0;
    uint32_t _readRequestMs = 0;
    String _readReply;
    static const int64_t READ_TIMEOUT_DEFAULT_MS = 2000;
    void onDeviceStatusChange(RaftDevice& device, const BusAddrStatus& addrStatus);
    void onDeviceData(std::vector<uint8_t>&& data, const void* pCallbackInfo);
    uint8_t attachedCount() const;
    void appendReading(AttachedDevice& device, bool fresh, String& out);
    void serviceReadRequest();
    RaftROS* getRaftROS();
};
