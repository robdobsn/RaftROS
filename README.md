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

## Host Setup Notes (READ FIRST if discovery isn't working)

Discovery silently fails on some host setups. These are the gotchas we hit
and the fixes — verified on Windows 11 + WSL2 Ubuntu 24.04 + ROS 2 Jazzy
(Fast DDS 3.x) and on native Linux + ROS 2 Humble (Fast DDS 2.6.x).

### 1. WSL2: mirrored networking is required

The default WSL2 NAT mode does NOT forward inbound multicast from the LAN
into WSL, so SPDP (`239.255.0.1:7400`) from the ESP32 never arrives.
Create/edit `%UserProfile%\.wslconfig` on Windows:

```ini
[wsl2]
networkingMode=mirrored
firewall=true
```

Then from an elevated PowerShell: `wsl --shutdown` and reopen WSL. In
mirrored mode WSL shares the Windows LAN IP, and multicast works.

### 2. Windows Firewall blocks inbound UDP to WSL

Even with mirrored networking, Windows Defender Firewall drops inbound
RTPS UDP to the WSL side by default. This is the most common "discovery
works one-way only" symptom (ESP32 sees the host; host never sees the
ESP32). Add a permissive rule from an elevated PowerShell:

```powershell
New-NetFirewallRule -DisplayName "ROS2 RTPS" -Direction Inbound `
  -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any
```

Alternative: set `firewall=false` in `.wslconfig` (less strict, fine for
a dev workstation). Verify with `sudo tcpdump -i any -nn udp port 7400`
inside WSL — if you see zero packets from the ESP32 IP but
`sudo nmap -sU -p 7400 <ESP32_IP>` works outbound, it's the firewall.

### 3. ROS 2 Jazzy: `ROS_LOCALHOST_ONLY` is deprecated — UNSET it

On Jazzy the semantics inverted; leaving `ROS_LOCALHOST_ONLY=0` in your
shell actually restricts discovery. Use:

```bash
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

On Humble, `ROS_LOCALHOST_ONLY=0` is still correct.

### 4. Same subnet / same `ROS_DOMAIN_ID`

ESP32 and host must be on the same L2 broadcast domain (guest WiFi
networks often isolate clients and will block SPDP). `ROS_DOMAIN_ID`
must match on both sides (RaftROS default is `0`).

### 5. Debug recipe when discovery fails

```bash
# In WSL / Linux host
sudo tcpdump -i any -w ~/raft_rtps.pcap 'udp and (port 7400 or portrange 7410-7500)'
sudo chown $USER ~/raft_rtps.pcap
# Open in Wireshark, filter on 'rtps'. Packets from the ESP32 IP confirm
# the firewall / mirroring path is OK; only host packets means Steps 1–2
# are not yet correct.
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
