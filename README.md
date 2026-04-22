# RaftROS

Native ROS 2 node functionality for ESP32 via the Raft framework.

RaftROS enables ESP32 firmware built on the Raft framework to function as a native ROS 2 node, participating directly in DDS discovery and data exchange without relying on micro-ROS or any external agent/bridge process.

## Status

**Phase 1 (Discovery), Phase 2 (Topic Publishing), and Phase 3 (Topic Subscribing with per-topic routing) are complete.** Verified end-to-end on 2026-04-22 against ROS 2 Humble + FastDDS 2.6.11 with an ESP32-S3 on WiFi:

- `ros2 node list` shows `/raft_esp32`
- `ros2 topic list` shows `/chatter` (published), `/chatter_in`, `/chatter_in2` (subscribed)
- `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints a sample per second
- `ros2 topic info /chatter --no-daemon -v` shows one RELIABLE + VOLATILE publisher from node `raft_esp32`
- `ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"` dispatches on-device to the correct per-slot handler

See `devdocs/RaftROS-development-status.md` for the full fix list and `devdocs/RaftROS-next-stages-implementation-plan.md` for what comes next.

## Features

- Clean-room RTPS 2.2 wire protocol implementation (MIT licensed)
- CDR serialization for standard ROS 2 message types
- SPDP/SEDP discovery — ESP32 appears as a first-class DDS participant
- Reliable QoS (HEARTBEAT + ACKNACK with retransmit) for both builtin endpoints and user topics
- VOLATILE- and TRANSIENT_LOCAL-aware HEARTBEAT `firstSN` semantics
- Shared runtime core with thin ESP32 and Linux platform wrappers
- Planned: automatic mapping of Raft device data to ROS 2 topics via DeviceTypeRecords, and automatic generation of ROS 2 services from device actions

## Quick Start

Build and flash the ExampleDiscoverable project (see `examples/ExampleDiscoverable/README.md`) and then, from a Linux host on the same network with ROS 2 Humble installed:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 bash -lc '
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  ros2 node list --no-daemon
  ros2 topic echo /chatter std_msgs/msg/String --no-daemon
'
```

## Repository Layout

- `components/RaftROS/` — the SysMod source and shared RTPS runtime.
- `components/RaftROS/RTPS/runtime/{discovery,reliability,announce,receive,schedule,wire,core}/` — shared runtime modules consumed by both ESP32 and Linux wrappers.
- `components/RaftROS/CDR/` — CDR encoder/decoder.
- `examples/ExampleDiscoverable/` — minimal ESP32 app that brings up RaftROS and publishes `/chatter` at 1 Hz.
- `linux_unit_tests/` — Linux-hosted unit tests (388+ cases) and a standalone linux RTPS publisher (`raftros_standalone.cpp`) used as a non-embedded reference implementation.
- `devdocs/` — design overview, development status, and implementation plan.

## Dependencies

- [RaftCore](https://github.com/robdobsn/RaftCore)
- [RaftSysMods](https://github.com/robdobsn/RaftSysMods) (StatePublisher, NetworkManager)

## License

MIT — see [LICENSE](LICENSE)
