# DemoSimple — Dynamic RaftROS Device Dashboard

`DemoSimple` is a host-side ROS 2 demo for `ExampleDiscoverable`. It watches the ROS graph for RaftROS auto-published device topics, subscribes dynamically using the discovered message type, and prints a live terminal table of the latest sensor values.

It is intended to show hot-plug behavior:

1. Flash and boot `examples/ExampleDiscoverable`.
2. Start this dashboard on a ROS 2 host on the same network.
3. Plug in supported I2C sensors.
4. Watch topics and values appear without restarting the dashboard.
5. Unplug sensors and watch rows disappear after ROS discovery catches up.

## Quick Start

From the repository root:

```bash
examples/DemoSimple/run_dashboard.sh
```

Useful options:

```bash
examples/DemoSimple/run_dashboard.sh --namespace /raft
examples/DemoSimple/run_dashboard.sh --qos reliable
examples/DemoSimple/run_dashboard.sh --all-raft-topics --include-chatter
examples/DemoSimple/run_dashboard.sh --no-clear
```

The wrapper sources `/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash`, selects Fast DDS over UDPv4, clears Fast DDS profile overrides, and handles the `ROS_LOCALHOST_ONLY` difference between Humble and newer distros.

If you already have your ROS environment sourced:

```bash
python3 -u examples/DemoSimple/raftros_dynamic_dashboard.py
```

## Expected Devices

| Device | Expected topic | ROS 2 type | Dashboard summary |
| --- | --- | --- | --- |
| VL6180 | `/raft/range_1_29` | `sensor_msgs/msg/Range` | range, min, max, frame |
| LSM6DS | `/raft/imu_1_6a` | `sensor_msgs/msg/Imu` | acceleration and gyro |
| VEML7700 | `/raft/illuminance_1_10` | `sensor_msgs/msg/Illuminance` | lux |
| AS5600 | `/raft/angle_1_36` | `std_msgs/msg/Float32` | angle value |

The dashboard is generic and will also subscribe to other importable ROS 2 message types under the selected namespace. Unknown or unavailable message packages stay visible with an `unsupported type` status instead of crashing the demo.

## QoS

The default subscription QoS is:

- BEST_EFFORT reliability
- VOLATILE durability
- KEEP_LAST history, depth 10

That matches RaftROS fast sensor streams. Use `--qos reliable` only when you know the topic is published with compatible reliable QoS.

## Troubleshooting

If no rows appear:

- Confirm `ExampleDiscoverable` is booted, connected to the same network, and `RaftROS.enable` is set.
- Confirm your ROS domain matches the firmware domain, usually `ROS_DOMAIN_ID=0`.
- On WSL2, use mirrored networking and allow inbound UDP ports used by RTPS.
- Try a direct probe for a known topic:

```bash
python3 -u scripts/typed_deserialize_probe.py /raft/range_1_29 sensor_msgs/msg/Range
```

If rows appear but values stay at `waiting`, the topic was discovered but no compatible samples have arrived yet. Check QoS, network multicast/unicast reachability, and whether the sensor is actually online.

