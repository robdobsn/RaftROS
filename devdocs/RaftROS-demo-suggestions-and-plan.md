# RaftROS / ExampleDiscoverable Demo Suggestions and Implementation Plan

**Date:** 2026-04-26  
**Status update:** 2026-04-27 — first demo path implemented in
`examples/DemoSimple`; Foxglove Bridge instructions added to `README.md`.
**Scope:** Demonstrations for `ExampleDiscoverable` showing hot-plug I2C devices appearing as native ROS 2 publishers.

## Zenoh Demo Extension (Planned 2026-09-17)

The existing demo and its recorded results use **RTPS/FastDDS**. The
[Zenoh implementation plan](RaftROS-zenoh-implementation-plan.md) adds a
separately built firmware profile, not a runtime switch in the current image.
Reuse the same sensors, ROS-visible topic names, dashboard and visualizations
after its interoperability gates pass.

1. Label the firmware/backend and pinned host versions. Build RTPS or Zenoh
  independently; do not demonstrate both from a supposedly smaller binary.
2. For Zenoh, run all host processes with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`,
  matching domain and the verified reachable session endpoints. Stop the
  old ros2cli daemon before changing RMW and restart it in the new environment
  if CLI introspection is used. FastDDS configuration does not select Zenoh.
3. Demonstrate the routerless topology validated in Z1. A separate
  `rmw_zenohd` control demo must be labeled router-assisted, not proof of the
  strict standalone goal. No DDS bridge or per-device host adapter is used.
4. Show the named ROS node and publisher endpoint metadata as well as live
  typed values. Start subscribers both before and after the firmware.
5. Hot-plug a range sensor and a composite sensor; verify publisher counts,
  fields/units and all endpoints on detach/replug. A dashboard subscription
  can keep a topic in the graph, so "topic still listed" is not a detach
  failure. Use publisher presence and sample age for removal/stale display.
6. Demonstrate WiFi recovery, a second board with a distinct namespace, and
  a late-started dashboard without stale publishers or duplicate dispatch.
7. Start Foxglove Bridge in the same Zenoh RMW/session environment. Foxglove
  Studio's WebSocket connection is unchanged; validate the bridge's pinned
  ROS/RMW build instead of assuming the existing FastDDS result transfers.

The current `run_dashboard.sh` already honors an explicit RMW override but
still applies FastDDS-specific environment setup. Z6 should scope those
settings to RTPS, audit discovery/removal behavior, and document the tested
Zenoh session configuration in the example READMEs. No Zenoh demo command is
claimed working by this planning update. The WSL/FastDDS graph caveat below
is not a waiver for Zenoh graph correctness; validate native Linux first.

## Demo Goal

Show that an ESP32 running `ExampleDiscoverable` can participate directly in ROS 2 discovery and automatically expose attached I2C devices as typed ROS 2 topics, without a micro-ROS agent or per-device host configuration.

The strongest demonstration is the dynamic behavior:

1. Start the ESP32 firmware.
2. Show the board as a named ROS 2 node (a DDS participant for the RTPS build).
3. Plug in sensors at runtime.
4. Show new ROS 2 topics appear.
5. Show typed values updating live.
6. Unplug sensors.
7. Show the corresponding publishers disappear; topics may remain while
  monitoring subscriptions exist.

Target devices:

| Device | I2C address | Class | Expected topic | ROS 2 type |
| --- | ---: | --- | --- | --- |
| LSM6DS | `0x6a` | `ACC`, `GYRO` | `/raft/imu_1_6a` | `sensor_msgs/msg/Imu` |
| VL6180 | `0x29` | `DIST` | `/raft/range_1_29` | `sensor_msgs/msg/Range` |
| VEML7700 | `0x10` | `LGHT` | `/raft/illuminance_1_10` | `sensor_msgs/msg/Illuminance` |
| AS5600 | `0x36` | `ANG` | `/raft/angle_1_36` | `std_msgs/msg/Float32` |

The exact topic namespace may change if `RaftROS` topic naming is configured differently, but the demo should preserve the same core shape: class-derived topic names, address-derived identity, and standard ROS 2 message types.

## Recommended Demo

Use a host-side dynamic ROS dashboard plus Foxglove Studio.

This is the best fit because it demonstrates what is novel about RaftROS:

- The ESP32 is a native DDS / ROS 2 participant.
- Device topics appear and disappear at runtime.
- Subscribers do not need to know the attached sensor set in advance.
- Standard ROS 2 message types are used for sensor values.
- The host side remains generic; no per-device demo code is needed.

### Demo Surface 1: Dynamic Terminal Dashboard

Create a small `rclpy` utility that watches the ROS graph, detects `/raft/...` topics, dynamically subscribes using the discovered type, and prints the latest value from each active topic.

Implemented as:

```text
examples/DemoSimple/raftros_dynamic_dashboard.py
examples/DemoSimple/run_dashboard.sh
examples/DemoSimple/README.md
```

This avoids the main weakness of plain `ros2 topic echo`: `echo` normally requires the user to know both the topic and type ahead of time. The dashboard should discover both dynamically.

Example output:

```text
RaftROS live devices

topic                  type                       age    latest
/raft/range_1_29       sensor_msgs/msg/Range      0.1s   range=0.184 m frame=range_1_29
/raft/imu_1_6a         sensor_msgs/msg/Imu        0.0s   accel=(0.02,-0.01,9.80) gyro=(0.00,0.01,0.00)
/raft/illuminance_1_10 sensor_msgs/msg/Illuminance 0.2s  illuminance=312.0 lux
/raft/angle_1_36       std_msgs/msg/Float32       0.1s   data=147.3 deg
```

When a device is unplugged, use publisher presence and sample age to remove
or mark the row stale; do not wait solely for the topic name to vanish while
the dashboard itself holds a subscription. On replug, live values resume.

### Demo Surface 2: Foxglove Studio

Use Foxglove as the polished visual companion:

- Plot `/raft/range_1_29/range` for the distance sensor.
- Plot `/raft/illuminance_1_10/illuminance` for ambient light.
- Plot `/raft/angle_1_36/data` for the magnetic encoder.
- Inspect `/raft/imu_1_6a` as a typed `sensor_msgs/Imu` stream.

Foxglove is more useful than Gazebo for the first demo because it visualizes live ROS data directly without needing a simulated robot model or physics world.

## Other Demo Options Considered

### Plain ROS 2 CLI

The CLI is useful for validation and should remain part of the demo script:

```bash
ros2 topic list
ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range --qos-reliability best_effort
```

However, it is not the best primary demo surface because:

- The user must know the topic and type ahead of time.
- `ros2` CLI / daemon behavior can be fragile with transient participants.
- Sensor streams use BEST_EFFORT QoS by default, so default CLI QoS can silently fail to match.

Use CLI commands as a backup and diagnostic layer, not as the main story.

### RViz2

RViz2 is useful for proving compatibility with standard ROS visualization tools, especially for `sensor_msgs/Range` and `sensor_msgs/Imu`.

It is less suitable as the primary demo because scalar topics such as light level and magnetic angle are not as naturally represented. RViz2 can be a later add-on once the dashboard and Foxglove flow is solid.

### Gazebo

Gazebo is not recommended for the first demo.

Gazebo would be useful if the goal were to drive a simulated robot or joint from live sensor data, for example:

- AS5600 controls a simulated joint angle.
- VL6180 drives a simulated proximity display.
- LSM6DS influences a model pose.

That would be a good second-stage demo, but it adds a simulation layer that distracts from the core RaftROS capability: live physical I2C devices becoming native ROS 2 publishers.

## Concrete Implementation Plan

### Slice 1: Create Dynamic Dashboard Utility

Added a script at:

```text
examples/DemoSimple/raftros_dynamic_dashboard.py
```

Responsibilities:

1. Initialise `rclpy` with a stable ROS environment.
2. Periodically call `node.get_topic_names_and_types()`.
3. Filter topics to the Raft namespace, initially `/raft/`.
4. For each new topic, resolve its message type dynamically.
5. Create a subscription with QoS compatible with RaftROS sensor streams.
6. Store the most recent message and timestamp per topic.
7. Remove dashboard rows for topics that disappear from the graph.
8. Render a compact terminal table at 2-5 Hz.

Use BEST_EFFORT / VOLATILE / KEEP_LAST QoS by default because that matches the fast sensor auto-publishing profile. Add an option to force RELIABLE for slow or diagnostic topics if needed.

CLI:

```bash
examples/DemoSimple/run_dashboard.sh \
  --namespace /raft \
  --qos best_effort \
  --refresh-hz 4
```

### Slice 2: Dynamic Type Resolution

Implement a helper that converts ROS 2 type strings into Python classes:

```text
sensor_msgs/msg/Range       -> sensor_msgs.msg.Range
sensor_msgs/msg/Imu         -> sensor_msgs.msg.Imu
sensor_msgs/msg/Illuminance -> sensor_msgs.msg.Illuminance
std_msgs/msg/Float32        -> std_msgs.msg.Float32
```

The utility should handle any importable ROS 2 message type, not just the four planned devices. Unknown or unavailable types should remain visible in the dashboard with a clear `unsupported type` status.

### Slice 3: Per-Type Value Summaries

Add concise formatters for common message types:

- `sensor_msgs/msg/Range`: `range`, `min_range`, `max_range`, `frame_id`.
- `sensor_msgs/msg/Imu`: linear acceleration and angular velocity.
- `sensor_msgs/msg/Illuminance`: `illuminance`.
- `sensor_msgs/msg/Temperature`: `temperature`.
- `sensor_msgs/msg/RelativeHumidity`: `relative_humidity`.
- `sensor_msgs/msg/FluidPressure`: `fluid_pressure`.
- `std_msgs/msg/Float32`: `data`.
- `std_msgs/msg/Int32`: `data`.
- `std_msgs/msg/Bool`: `data`.
- `std_msgs/msg/String`: first line or truncated JSON.

Fallback formatter:

```text
repr(message)
```

Keep the dashboard generic so future RaftROS-supported devices appear automatically.

### Slice 4: Demo Runner Script

Added a wrapper script that prepares the environment and starts the dashboard:

```text
examples/DemoSimple/run_dashboard.sh
```

It sources ROS, selects Fast DDS over UDPv4, clears Fast DDS profile overrides,
uses a local `examples/DemoSimple/logs` directory, and handles
`ROS_LOCALHOST_ONLY` for Humble vs newer distros.

Equivalent manual command:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset ROS_LOCALHOST_ONLY
python3 -u examples/DemoSimple/raftros_dynamic_dashboard.py --namespace /raft
```

If using ROS 2 Humble, keep `ROS_LOCALHOST_ONLY=0` where needed. If using Jazzy, leave `ROS_LOCALHOST_ONLY` unset.

### Slice 5: Foxglove Layout

Foxglove Bridge setup is documented in the main `README.md` for:

- `foxglove_bridge` running in WSL with Foxglove Studio on Windows.
- `foxglove_bridge` and Foxglove Studio both running natively on Linux.

Future optional polish: create a saved Foxglove layout or a short panel setup guide:

1. Connect Foxglove to the ROS 2 data source.
2. Add a topic list / raw messages panel.
3. Add plots for range, illuminance, and angle.
4. Add an IMU inspection panel for LSM6DS.
5. Save the layout under `devdocs` or `examples/ExampleDiscoverable/docs` if the exported layout is text-based and stable.

If a saved layout is not portable enough, document the manual setup instead.

### Slice 6: Demo Script / Checklist

Document the presenter flow:

1. Flash and boot `ExampleDiscoverable`.
2. Configure WiFi if needed.
3. Start the dashboard.
4. Confirm the base node and `/chatter` topic.
5. Plug VL6180 and show the range row appear.
6. Move a hand in front of the sensor and show range values changing.
7. Plug VEML7700 and show illuminance changing when covered or exposed to light.
8. Plug AS5600 and rotate the magnet to show angle updates.
9. Plug LSM6DS and move the board to show IMU values.
10. Unplug one device and show the row/topic disappear.
11. Replug it and show the same topic return.

### Slice 7: Validation

Validation should include:

- Dashboard starts with no Raft topics and does not crash.
- New topics are subscribed to without restarting the script.
- Known device types render useful summaries.
- Unknown but importable message types render via fallback.
- Unplugged device topics are removed from the dashboard.
- Replugged devices reappear under the same name.
- The script works when `ros2 topic echo` is unreliable due to daemon behavior.

For direct payload debugging, keep using:

```bash
python3 -u scripts/typed_deserialize_probe.py /raft/range_1_29 sensor_msgs/msg/Range
```

## Risks and Mitigations

### `_NODE_NAME_UNKNOWN_` In ROS CLI

On the current WSL2/Jazzy/FastDDS host, `ros2 topic info -v` can show RaftROS
publishers as `Node name: _NODE_NAME_UNKNOWN_` even when the data path works.
The current assumption is that the remaining symptom is specific to ros2cli/rmw
graph attribution on this WSL setup until it is reproduced on native Linux.

Mitigation: make `DemoSimple` and direct `rclpy` subscribers the primary demo
and validation path. Use Foxglove through `foxglove_bridge` for visualization.
Treat native Linux validation as the next discriminator before assuming a
remaining firmware protocol defect.

### ROS 2 CLI / Daemon Fragility

The demo should not depend on `ros2 topic list` or `ros2 topic echo` as the primary viewer. Use `rclpy` directly for the dashboard because it avoids several ros2cli daemon issues already observed in development.

### QoS Mismatch

Default ROS 2 subscriptions may not match RaftROS fast sensor streams. The dashboard should use BEST_EFFORT / VOLATILE by default and expose a command-line override.

### Topic Name Drift

The dashboard should not hard-code the four target topics. It should discover all topics under a namespace prefix and subscribe dynamically.

### Message Type Availability

The recommended types are standard ROS 2 packages, but minimal ROS installations may lack some packages. The utility should report unsupported imports rather than failing the whole dashboard.

### Hot-Unplug Timing

Topic removal timing depends on dispose handling and participant / endpoint discovery timing. The demo script should describe expected removal as "within a few seconds" rather than instant.

## Recommended First Implementation

`examples/DemoSimple/raftros_dynamic_dashboard.py` has been implemented. It provides the highest-value demo with the least extra infrastructure.

The main `README.md` now includes the demo guide and exact commands for:

- Flashing the firmware.
- Starting the dashboard.
- Opening Foxglove.
- Plugging each target device.
- Running fallback `ros2 topic echo` commands for individual topics.

