# ExampleDiscoverable - RaftROS demonstrator

A complete Raft ESP32 application that makes the board a native ROS 2 node
using the [RaftROS](../../README.md) SysMod. It is both the demo and the
reference for how an application uses the library: everything it does is in
`main/main.cpp` (registering the SysMod), `systypes/SysTypeMain/SysTypes.json`
(configuring it) and `components/MainSysMod/MainSysMod.cpp` (subscriptions,
services and parameters from application code).

Tested on an Adafruit ESP32-S3 TFT Feather with a VL6180 range sensor on I2C, against ROS 2 Jazzy.

## What it does

Once it is on WiFi:

- Appears as ROS 2 node `/raft_esp32` on domain 0.
- Publishes `std_msgs/msg/String` on `/chatter` (1 Hz by default, RELIABLE +
  VOLATILE) with a payload like `Hello from raft_esp32 [N]`.
- Subscribes to `/chatter_in` and `/chatter_in2` and logs what arrives.
- **Auto-publishes every bus device** `DeviceManager` finds as a typed ROS 2
  topic `/raft/<slug>_<bus>_<addrHex>` - a VL6180 at I2C 0x29 on bus 1 appears
  as `/raft/range_1_29` (`sensor_msgs/msg/Range`). Unplugging withdraws the
  topic; plugging back in announces it again.
- On the Zenoh build (the default), also serves four ROS 2 **services** and
  the ROS 2 **parameter** services - see "Services" and "Parameters" below.

## Choosing the transport

Exactly one transport is compiled into an image; there is no runtime switch.

| | Zenoh (default) | RTPS / DDS |
| --- | --- | --- |
| How it reaches ROS 2 | one TCP session to an `rmw_zenoh` router | directly, over multicast and unicast UDP |
| Needs on the ROS side | a reachable `rmw_zenohd`; ROS 2 tools run with `RMW_IMPLEMENTATION=rmw_zenoh_cpp` | nothing beyond the network (FastDDS or CycloneDDS) |
| Node, `/chatter`, string subscriptions | yes | yes |
| Device auto-publish, hot-plug | yes | yes |
| QoS profiles (`qosProfiles`) for publishers and subscriptions | yes | yes |
| **ROS 2 services** (`std_srvs` Trigger / SetBool / Empty, deferred replies) | **yes** | no - `addService` warns and returns -1 |
| **ROS 2 parameters** (`ros2 param list/get/set/describe/dump`) | **yes** | no - `declareParameter` warns, `parameter()` returns null |
| **Change settings from ROS** (`chatterPeriodMs`, `routerHost` persisted to NVS) | **yes** | no - use `/api/postsettings` |
| Reconnect after the router or session is lost | yes (re-declares everything) | n/a (discovery is continuous) |
| Worst main-loop pass under load, no terminal attached | ~13 ms | ~15 ms |
| App image (28% of the app partition free) | ~1.28 MB | ~1.27 MB |

The application code is the same on both: the RTPS build compiles the
services and parameters calls as stubs, so nothing in `MainSysMod` is
conditional on the transport.

To build the RTPS image, add to `systypes/SysTypeMain/sdkconfig.defaults`:

```
CONFIG_RAFTROS_BACKEND_RTPS=y
```

and delete `build/SysTypeMain/sdkconfig` (otherwise the previous selection is
kept).

## Build and flash

With the [Raft CLI](https://github.com/robdobsn/RaftCLI), from this directory:

```bash
raft run                           # build, flash and open the monitor
# or, separately:
raft build .
raft flash -p <PORT> .
```

The build fetches RaftCore, RaftSysMods, RaftWebServer and RaftI2C (`@main`,
see `systypes/Common/features.cmake`) and picks up RaftROS itself from this
repository. The first time, set the WiFi credentials from the serial console:

```
w/<SSID>/<password>
```

The board in `SysTypes.json` has its I2C bus on SDA 42 / SCL 41 (the Feather's
STEMMA QT connector) and `main.cpp` power-cycles the connector on GPIO 21;
change both for another board.

## Try it (Zenoh build)

On the ROS 2 host (Jazzy, `sudo apt install ros-jazzy-rmw-zenoh-cpp`):

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd            # the router; keep it running
                                             #  (detach it from any ssh session)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp      # in every shell that runs ros2
ros2 node list                               # /raft_esp32
ros2 topic echo /chatter                     # one message a second
ros2 topic echo /raft/range_1_29             # Range samples (with a VL6180 attached)
ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello'}"
ros2 service call /raft_esp32/devices std_srvs/srv/Trigger
ros2 param dump /raft_esp32
```

The device must be told where the router is - see the next section; if it
cannot reach it, its log says so and how to fix it.

### Telling the device where the router is (Zenoh)

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

## Try it (RTPS build)

On a ROS 2 host on the same subnet (no router needed):

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # or rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY                     # Jazzy: a leftover value restricts discovery
ros2 node list --no-daemon                   # /raft_esp32
ros2 topic echo /chatter std_msgs/msg/String --no-daemon
ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range --no-daemon --qos-reliability best_effort
```

With CycloneDDS pass `--no-daemon` and the message type, and expect the first
query to miss the node for a few seconds (each CLI call is a new
participant). Under WSL2, discovery also needs mirrored networking and a
firewall rule - see "Host Setup Notes" in the [top-level README](../../README.md#host-setup-notes-read-first-if-discovery-isnt-working).

## Services (Zenoh build)

The example serves four ROS 2 services:

| Service | Type | Does |
| --- | --- | --- |
| `/raft_esp32/devices` | `std_srvs/srv/Trigger` | Reports attached devices, chatter state, heap |
| `/raft_esp32/chatter_enable` | `std_srvs/srv/SetBool` | Starts / stops `/chatter` |
| `/raft_esp32/range` | `std_srvs/srv/Trigger` | A fresh VL6180 reading - a *deferred* reply (below) |
| `/raft_esp32/ping` | `std_srvs/srv/Empty` | Nothing - a round-trip check |

```bash
ros2 service list -t                         # expect the four, plus the parameter services
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
first VL6180 poll result newer than the call, peeking the latest decoded
result with `getLatestDecodedPollResponse` and comparing its `timeMs`. The
reading is taken after the call, not served from a cache.

A second call while one is parked is refused; with the sensor unplugged no
poll result comes and RaftROS answers the client with an error at the
timeout (5 s) rather than leaving it hanging. (`ros2 service call` logs that
error and then keeps waiting - give scripts their own timeout.)

Peeking is the better pattern for "the latest reading when I need it": it
costs nothing when nobody asks. An application can also subscribe to a
device's data with `DeviceManager::registerForDeviceData`; RaftCore fans each
sample out to every subscriber, so this does not disturb auto-publishing
(RaftCore `78781c0` or later - before that a second registration silently
replaced the auto-publisher's and stopped the topic).

Serveable types are `std_srvs` `Trigger`, `SetBool` and `Empty` (plus the
parameter services, which RaftROS adds itself); twelve service slots, six of
them taken by parameters, and four requests in flight. `GET /api/rosstat`
counts `svcAccepted`, `svcCompleted`, `svcDeferred`, `svcTimedOut`,
`svcRefused` and `svcUnknownKey`.

## Without a ROS install

With no router or ROS to hand, `tools/` has two stand-ins:

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

A device whose session drops re-declares everything on the new session and
carries on from the next sequence number, so restarting either tool is a fair
test of reconnection.

## Status and diagnostics

`GET http://<device-ip>/api/rosstat` is the SysMod's own view - session,
counters, heap, stack and loop timing - readable from any host with `curl`
(this is the Zenoh build after a 22-hour run):

```json
{"backend":"zenoh","conn":"ready","sessions":1,"devices":1,"pubs":2,"samples":482021,"subs":2,
 "rxDropped":0,"routerSource":"default","routerReachable":true,"connectFails":0,
 "stackFreeB":5564,"heapFreeB":147280,"heapMinB":114328,"loopMaxUs":17711,
 "services":10,"svcAccepted":3744,"svcCompleted":3744,"svcDeferred":720,"svcTimedOut":0,
 "svcRefused":0,"svcUnknownKey":0,"params":5}
```

The RTPS build reports its own set (`disc` peers, `loopMaxUs`, `rxDeferrals`,
send timing, heap, `logSuppressed`).

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

On a board whose console is USB-Serial-JTAG (the ESP32-S3 Feather and most S3 boards), a log
line costs about 10 ms of main-loop time whenever no host is reading the
port, whatever its length - and under 1 ms when a terminal is attached. So
the SysMod never writes more than one line in a loop pass, and the loop
figures on `rosstat` (`loopMaxUs` and the per-phase `loopMaxDrainUs`,
`loopMaxConnUs`, `loopMaxRxUs`, `loopMaxTxUs`) should be read with no
terminal open: that is the number an unattended device sees.

## Parameters (Zenoh build)

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

The SysMod is configured in the `RaftROS` block of
`systypes/SysTypeMain/SysTypes.json`; any key can also be overridden at run
time with `POST /api/postsettings/reboot` (persisted in NVS).

| Key | Default | Meaning |
| --- | --- | --- |
| `enable` | 0 | Must be 1 for the SysMod to run |
| `domainId` | 0 | ROS 2 domain |
| `nodeName` / `nodeNamespace` | `raft_esp32` / `/` | The node's name |
| `chatterEnable` | true | Publish `/chatter` |
| `qosProfiles` | built-ins | Per-topic and per-class QoS (below) |
| `routerHost` / `routerPort` | Kconfig (`CONFIG_RAFTROS_ZENOH_ROUTER_HOST`) / 7447 | Zenoh router, IPv4 only |
| `spdpIntervalMs` / `leaseDurationSec` | 5000 / 120 | RTPS discovery announce period and lease |

`DevMan` in the same file declares the buses (`buslist`); auto-publishing
needs `DevMan.enable` (on by default in this example).

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

## Auto-publishing bus devices

With `RaftROS.enable = 1` and `DevMan.enable = 1`, every device that
`DeviceManager` brings online (I2C, BLE, ...) is automatically mirrored as a
ROS 2 publisher. No per-device code, no per-topic config needed for the
common case.

### Topic + type rules

- Topic name: `/raft/<slug>_<bus>_<addrHex>`
  (e.g. `/raft/imu_1_6a`, `/raft/temperature_1_38`).
- Message type is chosen by first-match on the device's `clas[]` tags and
  device type name (see `AutoPub/AutoPubClassMap.h`):
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

Four built-in profiles (see `AutoPub/AutoPubQoSProfile.h`):

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
ros2 topic list                               # every device topic plus /chatter
ros2 topic info -v /raft/imu_1_6a             # an IMU on I2C bus 1, address 0x6A
ros2 topic echo /raft/imu_1_6a sensor_msgs/msg/Imu
```

When a device is removed (or stops responding long enough for DeviceManager
to mark it `PENDING_DELETION`) RaftROS withdraws its topic - the liveliness
token on Zenoh, an SEDP dispose on RTPS - so it leaves `ros2 topic list`
within seconds, with no stale publishers left behind.

## Further reading

- [RaftROS README](../../README.md) - using the library in your own application
- [Measured results](../../devdocs/RaftROS-zenoh-milestone-results.md) - loop budget, memory, soaks
- [Development status](../../devdocs/RaftROS-development-status.md) - the project log
- [Services](../../devdocs/RaftROS-services-implementation-plan.md) and
  [parameters](../../devdocs/RaftROS-parameters-implementation-plan.md) plans
- [RaftROS overview and design](../../devdocs/RaftROS-overview.md)
- [Raft command line documentation](https://github.com/robdobsn/RaftCLI)
- [Raft Framework documentation](https://github.com/robdobsn/RaftCore/wiki)
