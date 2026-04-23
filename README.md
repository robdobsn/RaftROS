# RaftROS

Native ROS 2 node functionality for ESP32 via the Raft framework.

RaftROS enables ESP32 firmware built on the Raft framework to function as a native ROS 2 node, participating directly in DDS discovery and data exchange without relying on micro-ROS or any external agent/bridge process.

## Status

**Phases 1–4 are complete.** Verified end-to-end against ROS 2 Humble +
FastDDS 2.6.11 with an ESP32-S3 on WiFi:

- **Phase 1 (Discovery)** — `ros2 node list` shows `/raft_esp32`.
- **Phase 2 (Topic Publishing)** — `ros2 topic echo /chatter` prints a
  sample per second; RELIABLE + VOLATILE QoS.
- **Phase 3 (Topic Subscribing)** — `ros2 topic pub --once /chatter_in2 ...`
  dispatches on-device to the correct per-slot handler.
- **Phase 4 (DeviceManager auto-publishing)** — every bus device detected
  by `DeviceManager` automatically becomes a typed ROS 2 topic
  (`rt/raft/<slug>_<bus>_<addrHex>`) with per-class message type, REP-103
  unit scaling, per-writer QoS, and clean dispose on detach. No per-device
  code. Composite devices (AHT20, BMP280, IMUs) publish multiple topics
  from a single bus sample.

**911 linux unit tests pass.** ESP32-S3 firmware fits in 25% of a
`0x1b0000` app partition.

See `devdocs/RaftROS-development-status.md` for the full fix list and
`devdocs/RaftROS-auto-publishing-design.md` for the Phase 4 design.

## Features

- Clean-room RTPS 2.2 wire protocol implementation (MIT licensed)
- CDR serialization for standard ROS 2 message types
- SPDP/SEDP discovery — ESP32 appears as a first-class DDS participant
- Reliable QoS (HEARTBEAT + ACKNACK with retransmit) for both builtin endpoints and user topics
- VOLATILE- and TRANSIENT_LOCAL-aware HEARTBEAT `firstSN` semantics
- Shared runtime core with thin ESP32 and Linux platform wrappers
- **Automatic ROS 2 publishing of every `DeviceManager`-detected bus device**
  — `clas[]`-driven type mapping, REP-103 SI unit scaling, composite
  multi-topic devices, per-class QoS profiles with SysTypes override.
- Planned: automatic subscription of actuator classes (SRVO/PUMP/PIX) for
  command-side wiring, and ROS 2 services from device actions.

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
