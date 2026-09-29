# ExampleDiscoverable — RaftROS Phase 1–4 Demonstrator

A minimal Raft ESP32 application that brings up the RaftROS SysMod so the ESP32 becomes a native ROS 2 node. Generated originally by `raft new` and extended with the RaftROS SysMod.

## What it does

When connected to WiFi and booted:

- Registers as DDS participant `/raft_esp32` on domain 0.
- Responds to ROS 2 SPDP/SEDP discovery so `ros2 node list` and `ros2 topic list` see it as a first-class node.
- Publishes `std_msgs/msg/String` on `/chatter` at 1 Hz with a payload like `Hello from raft_esp32 [N]` (RELIABLE + VOLATILE QoS).
- Handles incoming HEARTBEAT / ACKNACK and retransmits lost SEDP and user-data DATA submessages.
- **Auto-publishes every connected bus device** as a typed ROS 2 topic on
  `/raft/<slug>_<bus>_<addrHex>` (Phase 4). For example an MPU6050 at
  `bus=1, addr=0x68` appears as `/raft/imu_1_68` with type
  `sensor_msgs/msg/Imu`. See "Auto-publishing bus devices" below.
- Does all of the above over **either** transport: native RTPS/DDS (the
  default) or Zenoh via an `rmw_zenoh` router. The choice is made at build
  time - see "Choosing the transport" below.

## Build and flash

The simplest way to build this application is to [use the raft command line interface](https://github.com/robdobsn/RaftCLI):

```bash
raft run
```

Once flashed, configure WiFi via the serial console the first time:

```
w/<SSID>/<password>
```

## Verify from a ROS 2 host

On a Linux host on the same network with ROS 2 Humble installed:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 bash -lc '
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0

  ros2 node list --no-daemon
  # expect: /raft_esp32

  ros2 topic list --no-daemon
  # expect to include: /chatter

  ros2 topic info /chatter --no-daemon -v
  # expect: one publisher, RELIABLE + VOLATILE, from node raft_esp32

  ros2 topic echo /chatter std_msgs/msg/String --no-daemon
  # expect: "data: Hello from raft_esp32 [N]" once per second
'
```

The `env -u PYTHONPATH PYTHONNOUSERSITE=1` and `--no-daemon` parts avoid a common user-site numpy collision and a ROS 2 daemon XMLRPC timeout when running against a non-local DDS participant.

## Choosing the transport

ROS 2 can be reached two ways, and one is compiled into an image - there is no
runtime switch, and the RTPS runtime and the Zenoh session are never both
linked.

| | RTPS / DDS | Zenoh (default) |
| --- | --- | --- |
| How it reaches ROS 2 | directly, over multicast and unicast UDP | one TCP session to a router |
| Needs | nothing beyond the network | a reachable `rmw_zenohd`, and ROS 2 peers running `rmw_zenoh` |
| App image | ~1268 kB | ~1251 kB |
| Device auto-publish | yes | yes |
| `/chatter` publisher and string subscriptions | yes | yes |

Zenoh is the default. To build the RTPS image instead, in
`systypes/SysTypeMain/sdkconfig.defaults`:

```
CONFIG_RAFTROS_BACKEND_RTPS=y
```

Delete `build/SysTypeMain/sdkconfig` after changing `sdkconfig.defaults`, or
the old selection is kept.

### Telling the device where the router is

The router address is layered, each level overriding the one before:

1. **Built-in default** - `CONFIG_RAFTROS_ZENOH_ROUTER_HOST` in menuconfig
   (`192.168.86.192:7447` as shipped).
2. **SysTypes** - `"routerHost"` / `"routerPort"` in the `RaftROS` block of
   `systypes/SysTypeMain/SysTypes.json`.
3. **Runtime, no rebuild** - post a settings overlay from any host on the
   network; it persists in NVS and the device reboots into it:

   ```bash
   curl -X POST http://<device-ip>/api/postsettings/reboot \
        -d '{"RaftROS":{"routerHost":"192.168.1.50"}}'
   curl http://<device-ip>/api/getsettings/nv      # see what is set
   curl http://<device-ip>/api/clearsettings       # back to SysTypes/default
   ```

`routerHost` must be an IPv4 address: resolving a name would block the main
loop for as long as the DNS query takes, so the SysMod refuses a hostname and
says so in the log.

**If the router isn't there, the device says so.** After three failed connection
attempts, and every 30 s while it stays unreachable, the log carries a
`ROUTER UNREACHABLE` warning that names the address, says whether the host
answered (no router running there) or not (wrong address), says whether the
address is the built-in default that was set for another network, and gives the
`curl` line to change it. `GET /api/rosstat` reports the same:
`"routerSource":"default"|"config"`, `"routerReachable"`, `"connectFails"`,
`"lastSessionAgoS"`.

## Demonstrating the Zenoh build

Run a router on a machine the device can reach, then use the ordinary ROS 2
tools. This is verified end to end against `rmw_zenoh_cpp` 0.2.10 on ROS 2
Jazzy - node, both publishers, both subscriptions and per-topic routing:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd            # the router (start it detached from any ssh
                                             #  session, or it dies with the session)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 node list                               # expect /raft_esp32
ros2 topic echo /chatter                     # expect one message a second
ros2 topic echo /raft/range_1_29             # expect Range samples
ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello'}"
```

The application code is the same either way: `MainSysMod` calls
`addStringSubscription("rt/chatter_in", ...)` and gets the same handler
arguments on both builds, so nothing in the example is conditional on the
transport.

### Services

The Zenoh build also serves two ROS 2 services (the RTPS build compiles the
same application code; its `addService` logs a warning and returns -1):

```bash
ros2 service list -t                         # expect both, with their types
ros2 service call /raft_esp32/devices std_srvs/srv/Trigger
#   success=True, message='1 device(s) attached, chatter on, chatter_in rx 0/0, heap free 173356 B'
ros2 service call /raft_esp32/chatter_enable std_srvs/srv/SetBool "{data: false}"
ros2 topic echo --once /chatter              # nothing arrives until ...
ros2 service call /raft_esp32/chatter_enable std_srvs/srv/SetBool "{data: true}"
```

A handler runs on the main loop and answers from state it already holds
(`MainSysMod.cpp`):

```cpp
pRaftROS->addService("/raft_esp32/chatter_enable", "std_srvs::srv::dds_::SetBool_",
    [pRaftROS](const RaftROS::ServiceRequest& request, RaftROS::ServiceReply& reply)
    {
        pRaftROS->setChatterEnabled(request.fields.data);
        reply.fields.success = true;
        reply.fields.message = request.fields.data ? "chatter on" : "chatter off";
        return RaftROS::ServiceOutcome::Replied;
    });
```

A handler that needs the bus cannot wait for it - bus transactions happen on
the bus task, and a handler runs on the main loop. It returns `Deferred` and
the request is completed from a later loop pass with
`completeService(token, reply)`. The third service shows this:

```bash
ros2 service call /raft_esp32/range std_srvs/srv/Trigger
#   success=True, message='range 255.0 mm valid=1, read 82 ms after the call'
```

The handler parks the request; `MainSysMod::loop()` completes it from the
first VL6180 poll result that lands after the call (a data callback on the
bus task bumps a counter; the loop decodes the latest result with
`getLatestDecodedPollResponse`). The reading is taken after the call, not
served from a cache. A second call while one is parked is refused; with the
sensor unplugged no poll result comes and RaftROS answers the client with an
error at the timeout (5 s) rather than leaving it hanging.

The types this build can serve are `std_srvs` `Trigger`, `SetBool` and
`Empty`; up to four services, four requests in flight. `GET /api/rosstat`
counts `svcAccepted`, `svcCompleted`, `svcDeferred`, `svcTimedOut`,
`svcRefused` and `svcUnknownKey`.

With no router to hand, `tools/` has two stand-ins that need no ROS install:

```bash
# Speaks the wire protocol directly: shows the session, every liveliness token
# the device declares, and decoded samples.  Use --interest to exercise the
# device's interest replies.
python3 tools/zenoh_router_stub.py --interest

# Uses the real Zenoh library, so it also shows that Zenoh itself accepts what
# the firmware sends.  Needs: pip install eclipse-zenoh==1.8.0
python3 tools/zenoh_subscriber_demo.py
```

Unplugging a sensor withdraws its topic from the graph within a scan period;
plugging it back in announces it again as a fresh endpoint, on the same
session. Both stand-ins print the device's node and per-endpoint tokens
(topic, type and ROS type hash) and decode the samples by the type named in
their key:

```
SAMPLE #1 "Hello from raft_esp32 [35]"
    0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c74...
SAMPLE #1 range=0.0170 m
    0/raft/range_1_29/sensor_msgs::msg::dds_::Range_/RIHS01_b42b6256...
```
 A device whose session drops re-declares
everything on the new session and carries on from the next sequence number, so
restarting either tool is a fair test of reconnection.

The SysMod's own view is on `GET /api/rosstat` - session state, counters, and
the system's stack headroom and free/minimum heap, so a long soak can be read
from any host with `curl` once a minute:

```json
{"backend":"zenoh","conn":"ready","sessions":1,"devices":1,"pubs":2,"samples":204,"subs":2,
 "rxDropped":0,"routerSource":"default","routerReachable":true,"connectFails":0,"lastSessionAgoS":41,
 "stackFreeB":5480,"heapFreeB":177012,"heapMinB":168400,"loopMaxUs":2357,
 "services":2,"svcAccepted":56,"svcCompleted":56,"svcDeferred":0,"svcTimedOut":0,"svcRefused":0,"svcUnknownKey":0}
```

### Logging

A default build logs events only - setup, device attach/detach, connection,
node and subscription announcements, and every warning. The bring-up detail is
behind per-file switches: `RAFTROS_VERBOSE_LOGGING` near the top of
`Zenoh/RaftROSZenoh.cpp` or `RTPS/RaftROSRTPS.cpp` (full key expressions and
tokens, per-peer announce traffic, the 5 s health line), and
`AUTOPUB_DEBUG_STATUS_CB` / `AUTOPUB_DEBUG_SAMPLES` in
`AutoPub/AutoPubDeviceSource.hpp` (every DeviceManager callback; a per-100-
samples counter line per device). Console writes block the main loop, so leave
them off for a demo; the counters are all on `GET /api/rosstat`.

On a board whose console is USB-Serial-JTAG (the ProS3, the Feather), a log
line costs about 10 ms of main-loop time whenever no host is reading the
port, whatever its length - and under 1 ms when a terminal is attached. So
the SysMod never writes more than one line in a loop pass, and the loop
figures on `rosstat` (`loopMaxUs` and the per-phase `loopMaxDrainUs`,
`loopMaxConnUs`, `loopMaxRxUs`, `loopMaxTxUs`) should be read with no
terminal open: that is the number an unattended device sees.

### Parameters

The Zenoh build is a full ROS 2 parameter server: the six `rcl_interfaces`
services are declared for the node, so the standard tools work.

```bash
ros2 param list /raft_esp32
ros2 param dump /raft_esp32
#   chatterEnable: true
#   chatterPeriodMs: 1000
#   rangeOffsetMm: 0.0
#   routerHost: 192.168.86.192
#   use_sim_time: false
ros2 param set /raft_esp32 chatterPeriodMs 250    # /chatter now at 4 Hz
ros2 param set /raft_esp32 chatterEnable false
ros2 param describe /raft_esp32 routerHost
ros2 param set /raft_esp32 routerHost 192.168.86.50
```

| Parameter | Type | Effect |
| --- | --- | --- |
| `chatterEnable` | bool | Starts and stops `/chatter` at once |
| `chatterPeriodMs` | int | Period of `/chatter`, 100-60000 ms |
| `routerHost` | string | Zenoh router IPv4 address. **Persisted** in the same settings overlay `/api/postsettings` writes; the node reconnects to it once the reply has gone out |
| `rangeOffsetMm` | double | The example's own: added to `/raft_esp32/range` readings |
| `use_sim_time` | bool | Read-only `false` (every ROS 2 node has it) |

A refused set reports why, in the words a stock ROS 2 node uses
(`Wrong parameter type, expected 'Type.INTEGER' got 'Type.STRING'`,
`Trying to set a read-only parameter: use_sim_time.`) or the parameter
owner's own (`chatterPeriodMs must be 100-60000`). Only `chatterEnable` and
`chatterPeriodMs` of the SysMod's settings are live; the rest take effect
through `/api/postsettings` and a reboot as before.

An application declares its own parameters on the SysMod and either reads
them where they are used or takes a callback that can refuse a value:

```cpp
pRaftROS->declareParameter("rangeOffsetMm", 0.0, "Added to every /raft_esp32/range reading, in mm");
...
const RaftROS::ParamValue* pOffset = pRaftROS->parameter("rangeOffsetMm");
```

Scalars only (bool, int64, double, string up to 63 characters); 16
parameters per node. A set arrives on the main loop; a callback must not
block (a `routerHost` set writes NVS, the one flash operation, ~12 ms).

## Configuration

The RaftROS SysMod and the overall system configuration live in `systypes/SysTypeMain/SysTypes.json`. The relevant RaftROS fields are the domain ID, the ROS 2 node name (`raft_esp32` by default), the SPDP announce interval and participant lease duration (RTPS), and the router address (Zenoh).

### QoS for subscriptions

The `qosProfiles` block applies to subscriptions as well as published devices.
An alias override keyed on the topic's last segment picks the profile a reader
is announced with; without one, readers use `fallback_string` (RELIABLE,
VOLATILE, depth 10). It can be set at runtime like the router address:

```bash
curl -X POST http://<device-ip>/api/postsettings/reboot \
     -d '{"RaftROS":{"qosProfiles":{"chatter_in":"event"}}}'
ros2 topic info -v /chatter_in     # Reliability: RELIABLE  Durability: TRANSIENT_LOCAL
```

## Auto-publishing bus devices (Phase 4)

With `RaftROS.enable = 1` and `DevMan.enable = 1`, every device that
`DeviceManager` brings online (I2C, BLE, ...) is automatically mirrored as a
ROS 2 publisher. No per-device code, no per-topic config needed for the
common case.

### Topic + type rules

- Topic name: `/raft/<slug>_<bus>_<addrHex>`
  (e.g. `/raft/imu_1_6a`, `/raft/temperature_1_38`).
- Message type is chosen by first-match on the device's `clas[]` tags and
  device type name (see `RTPSAutoPubClassMap.h`):
  - `{ACC, GYRO}` → `sensor_msgs/msg/Imu` (single writer).
  - `{TEMP, RH}` → **two writers**: `Temperature` + `RelativeHumidity`.
  - `{PRES, TEMP}` → **two writers**: `FluidPressure` + `Temperature`.
  - Single-class rules for TEMP, RH, PRES, LGHT, PROX, DIST, ANG, ROT,
    ACC, TCH, BTN, FRCE, HRM, SOIL, GAME.
  - Actuators (SRVO, PUMP, PIX) are excluded from publishing.
  - Unknown classes fall back to `std_msgs/msg/String` with a JSON body of
    every decoded field, topic slug `raw`.
- Unit scaling follows REP-103 (g→m/s², °/s→rad/s, mm→m, hPa→Pa,
  %→0..1). `Header.stamp` is taken from the poll record's `timeMs` field,
  so subscribers see sample-time not emit-time.

### QoS profiles

Four built-in profiles (see `RTPSAutoPubQoSProfile.h`):

| Profile           | Reliability | Durability       | Depth | Default for                                      |
|-------------------|-------------|------------------|-------|--------------------------------------------------|
| `fast_sensor`     | BEST_EFFORT | VOLATILE         | 10    | ACC, GYRO, IMU, PROX, LGHT, DIST, ANG, HRM, FRCE |
| `slow_sensor`     | RELIABLE    | VOLATILE         | 5     | TEMP, RH, PRES, SOIL, BTHM                       |
| `event`           | RELIABLE    | TRANSIENT_LOCAL  | 20    | BTN, TCH, ROT, GAME                              |
| `fallback_string` | RELIABLE    | VOLATILE         | 10    | any unmapped class                               |

Override in SysTypes (resolution order: per-device alias → per-class
override → built-in default):

```jsonc
"RaftROS": {
  "enable": 1,
  "qosProfiles": {
    "imu_1_6a":         "slow_sensor",            // per-device alias (topic tail)
    "temperature_1_38": "event",
    "classDefaults":    { "ACC": "slow_sensor" }  // per-class override
  }
}
```

### Verify from a ROS 2 host

```bash
# Lists every device topic plus /chatter and ros_discovery_info.
ros2 topic list --no-daemon

# Example: an MPU6050 on I2C bus 1, address 0x6A.
ros2 topic info /raft/imu_1_6a --no-daemon -v
ros2 topic echo /raft/imu_1_6a sensor_msgs/msg/Imu --no-daemon
```

When the device is removed (or stops responding long enough for
DeviceManager to mark it `PENDING_DELETION`) RaftROS sends an SEDP dispose
so the topic disappears from `ros2 topic list` within a heartbeat interval
— no stale writers left behind.

## Further reading

- [RaftROS overview and design](../../devdocs/RaftROS-overview.md)
- [RaftROS development status](../../devdocs/RaftROS-development-status.md)
- [RaftROS next stages plan](../../devdocs/RaftROS-next-stages-implementation-plan.md)
- [Raft command line documentation](https://github.com/robdobsn/RaftCLI)
- [Raft Framework documentation](https://github.com/robdobsn/RaftCore/wiki)
