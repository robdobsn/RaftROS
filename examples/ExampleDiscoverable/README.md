# ExampleDiscoverable — RaftROS Phase 1–4 Demonstrator

A minimal Raft ESP32 application that brings up the RaftROS SysMod so the ESP32 becomes a native ROS 2 node. Generated originally by `raft new` and extended with the RaftROS SysMod.

## What it does

When connected to WiFi and booted:

- Registers as DDS participant `/raft_esp32` on domain 0.
- Responds to ROS 2 SPDP/SEDP discovery so `ros2 node list` and `ros2 topic list` see it as a first-class node.
- Publishes `std_msgs/msg/String` on `/chatter` at 1 Hz with a payload like `Hello from raft_esp32 [N]` (RELIABLE + VOLATILE QoS).
- Handles incoming HEARTBEAT / ACKNACK and retransmits lost SEDP and user-data DATA submessages.
- **Auto-publishes every connected bus device** as a typed ROS 2 topic on
  `rt/raft/<slug>_<bus>_<addrHex>` (Phase 4). For example an MPU6050 at
  `bus=1, addr=0x68` appears as `rt/raft/imu_1_68` with type
  `sensor_msgs/msg/Imu`. See "Auto-publishing bus devices" below.

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

## Configuration

The RaftROS SysMod and the overall system configuration live in `systypes/SysTypeMain/SysTypes.json`. The relevant RaftROS fields are the domain ID, the ROS 2 node name (`raft_esp32` by default), the SPDP announce interval, and the participant lease duration.

## Auto-publishing bus devices (Phase 4)

With `RaftROS.enable = 1` and `DevMan.enable = 1`, every device that
`DeviceManager` brings online (I2C, BLE, ...) is automatically mirrored as a
ROS 2 publisher. No per-device code, no per-topic config needed for the
common case.

### Topic + type rules

- Topic name: `rt/raft/<slug>_<bus>_<addrHex>`
  (e.g. `rt/raft/imu_1_6a`, `rt/raft/temperature_1_38`).
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
ros2 topic info /rt/raft/imu_1_6a --no-daemon -v
ros2 topic echo /rt/raft/imu_1_6a sensor_msgs/msg/Imu --no-daemon
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
