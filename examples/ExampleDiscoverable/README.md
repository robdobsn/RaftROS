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

| | RTPS / DDS (default) | Zenoh |
| --- | --- | --- |
| How it reaches ROS 2 | directly, over multicast and unicast UDP | one TCP session to a router |
| Needs | nothing beyond the network | a reachable `rmw_zenohd`, and ROS 2 peers running `rmw_zenoh` |
| App image | ~1268 kB | ~1251 kB |
| Device auto-publish | yes | yes |
| `/chatter` and string subscriptions | yes | not yet |

Select Zenoh in `systypes/SysTypeMain/sdkconfig.defaults`:

```
CONFIG_RAFTROS_BACKEND_ZENOH=y
```

and point the device at the router in `systypes/SysTypeMain/SysTypes.json`:

```json
"RaftROS": {
    "routerHost": "192.168.86.192",
    "routerPort": 7447
}
```

`routerHost` must be an IPv4 address: resolving a name would block the main
loop for as long as the DNS query takes, so the SysMod refuses a hostname and
says so in the log. Delete `build/SysTypeMain/sdkconfig` after changing
`sdkconfig.defaults`, or the old selection is kept.

## Demonstrating the Zenoh build

Run a router on a machine the device can reach, then use the ordinary ROS 2
tools:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd            # the router
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 node list                               # expect /raft_esp32
ros2 topic echo /raft/range_1_29             # expect Range samples
```

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

Both print the device's node token, its per-endpoint token (topic, type and
ROS type hash), and the samples. A device whose session drops re-declares
everything on the new session and carries on from the next sequence number, so
restarting either tool is a fair test of reconnection.

The SysMod's own view is on `GET /api/rosstat`:

```json
{"backend":"zenoh","conn":"ready","sessions":1,"devices":1,"pubs":1,"samples":9}
```

## Configuration

The RaftROS SysMod and the overall system configuration live in `systypes/SysTypeMain/SysTypes.json`. The relevant RaftROS fields are the domain ID, the ROS 2 node name (`raft_esp32` by default), the SPDP announce interval and participant lease duration (RTPS), and the router address (Zenoh).

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
