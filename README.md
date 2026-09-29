# RaftROS

Native ROS 2 node functionality for ESP32 via the Raft framework.

RaftROS enables ESP32 firmware built on the Raft framework to function as a native ROS 2 node, participating directly in DDS discovery and data exchange without relying on micro-ROS or any external agent/bridge process.

## Status

A Raft SysMod that makes an ESP32 a native ROS 2 node over **either**
transport, chosen at build time:

- **Zenoh (the default since 2026-09-27)** - talks to an `rmw_zenoh` router
  (`ros2 run rmw_zenoh_cpp rmw_zenohd`); every ROS 2 tool works with
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- **RTPS/DDS** - a clean-room RTPS 2.2 participant that joins DDS discovery
  directly (FastDDS, CycloneDDS); opt in with `CONFIG_RAFTROS_BACKEND_RTPS=y`.

What works, verified on an ESP32-S3 against ROS 2 Jazzy
([measured results](devdocs/RaftROS-zenoh-milestone-results.md)):

| | Zenoh | RTPS |
| --- | --- | --- |
| Node in the graph, `/chatter`, string subscriptions | yes | yes |
| Every `DeviceManager` bus device auto-published as a typed topic | yes | yes |
| QoS profiles (`qosProfiles`) for publishers and subscriptions | yes | yes |
| Device hot-plug (topic withdrawn and re-announced) | yes | earlier phase |
| ROS 2 **services** (`std_srvs` Trigger / SetBool / Empty, deferred replies) | yes | no (by decision) |
| ROS 2 **parameters** (`ros2 param list/get/set/describe/dump`) | yes | no (by decision) |
| 12 h soak: one session, free heap flat | yes | - |

Worst main-loop pass, unattended: ~13 ms (Zenoh), ~15 ms (RTPS) against the
Raft 50 ms contract. Image ~1.27 MB (28% of the app partition free); free heap
~147 kB on the Zenoh build with services and parameters.

Host tests: the unit suite, the Zenoh session (1298), codec (2223) and
firmware-piece (141) suites all pass. The example -
[`examples/ExampleDiscoverable`](examples/ExampleDiscoverable/README.md) - is
the demo, with the exact `ros2` commands for every feature.

The project log is
[`devdocs/RaftROS-development-status.md`](devdocs/RaftROS-development-status.md).

### RTPS CLI Caveat (historical)

Applies to the RTPS build only. On the Windows 11 + WSL2 + ROS 2 Jazzy test setup of 2026-04, `ros2 topic info -v`
may show the ESP32 publisher with `Node name: _NODE_NAME_UNKNOWN_`, and
`ros2 node info /raft_esp32` may fail even while typed subscriptions receive
valid samples. This is currently treated as a host/CLI graph-attribution issue,
not a firmware data-path failure:

- `rclpy` subscribers receive and deserialize RaftROS samples correctly.
- `examples/DemoSimple/run_dashboard.sh` discovers `/raft/...` topics and
  displays live values.
- `foxglove_bridge` running in WSL works with Foxglove Studio on Windows when
  WSL networking/firewall setup allows ROS 2 discovery.

Assumption as of 2026-04-27: the remaining `_NODE_NAME_UNKNOWN_` symptom is
specific to ros2cli/rmw graph introspection on this WSL/Jazzy/FastDDS setup
until reproduced on native Linux. Native Linux avoids the WSL multicast and
firewall layer and should be used to confirm whether any protocol-side fix is
still required. See `devdocs/RaftROS-development-status.md` for the detailed
wire-level investigation and mitigations.

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
- **Zenoh transport** (default): `rmw_zenoh`-compatible session, liveliness
  tokens, interests, reconnect with a configurable router address
  (Kconfig < SysTypes < `/api/postsettings`) and an actionable warning when
  the router is unreachable.
- **ROS 2 services** (Zenoh): `std_srvs` Trigger / SetBool / Empty servers from
  application code; handlers never block the loop, and can defer a reply
  until a bus reading arrives.
- **ROS 2 parameters** (Zenoh): the six `rcl_interfaces` services; the node's
  own settings (`chatterEnable`, `chatterPeriodMs`, `routerHost` - persisted)
  and any the application declares.
- `GET /api/rosstat`: session, counters, heap, stack and per-phase loop maxima.
- Planned: automatic subscription of actuator classes (SRVO/PUMP/PIX) for
  command-side wiring; ROS 2 actions.

## Quick Start (RTPS build)

For the default Zenoh build, follow
[`examples/ExampleDiscoverable/README.md`](examples/ExampleDiscoverable/README.md#demonstrating-the-zenoh-build).

Build and flash the ExampleDiscoverable project (see `examples/ExampleDiscoverable/README.md`) and then, from a Linux host on the same network with ROS 2 Humble installed:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 bash -lc '
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  ros2 node list --no-daemon
  ros2 topic echo /chatter std_msgs/msg/String --no-daemon
'
```

## Demo: ExampleDiscoverable + DemoSimple + Foxglove

`examples/ExampleDiscoverable` is the ESP32 firmware demo. When flashed to an
ESP32 with WiFi configured, it appears as a native ROS 2 participant and
auto-publishes every supported I2C device detected by `DeviceManager`.

`examples/DemoSimple` is the host-side terminal demo. It watches the ROS graph
for RaftROS topics, dynamically subscribes using the discovered message type,
and prints live sensor values without needing to know which I2C devices are
attached ahead of time.

### 1. Flash And Boot ExampleDiscoverable

From `examples/ExampleDiscoverable`:

```bash
raft run
```

If WiFi has not been configured yet, use the serial console:

```text
w/<SSID>/<password>
```

Attach one or more supported I2C devices. Typical demo devices include:

| Device | Expected topic | ROS 2 type |
| --- | --- | --- |
| VL6180 distance sensor | `/raft/range_1_29` | `sensor_msgs/msg/Range` |
| VEML7700 ambient light sensor | `/raft/illuminance_1_10` | `sensor_msgs/msg/Illuminance` |
| AS5600 magnetic angle sensor | `/raft/angle_1_36` | `std_msgs/msg/Float32` |
| LSM6DS IMU | `/raft/imu_1_6a` | `sensor_msgs/msg/Imu` |

### 2. Run DemoSimple

From the repository root on a ROS 2 host:

```bash
examples/DemoSimple/run_dashboard.sh
```

The dashboard should show rows appearing and disappearing as I2C devices are
plugged and unplugged. For example, with a VL6180 attached:

```text
topic             type                   age    count  latest
/raft/range_1_29  sensor_msgs/msg/Range  0.1s      42  range=0.184 m min=0.000 max=2.000 frame=raft
```

Useful options:

```bash
examples/DemoSimple/run_dashboard.sh --namespace /raft
examples/DemoSimple/run_dashboard.sh --all-raft-topics --include-chatter
examples/DemoSimple/run_dashboard.sh --no-clear
```

The wrapper sources `/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash`, selects Fast
DDS over UDPv4, and uses a local log directory under `examples/DemoSimple/logs`.
If you already have a ROS environment sourced, you can run the Python script
directly:

```bash
python3 -u examples/DemoSimple/raftros_dynamic_dashboard.py
```

### 3. Run Foxglove Studio With A Bridge

Foxglove Studio connects to ROS 2 through `foxglove_bridge`. Install the bridge
in the ROS environment that can already see the RaftROS topics:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO:-jazzy}-foxglove-bridge
```

Start the bridge:

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset ROS_LOCALHOST_ONLY

ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Then open Foxglove Studio and connect to:

```text
ws://localhost:8765
```

Suggested Foxglove panels:

- Plot `/raft/range_1_29.range` for the VL6180.
- Plot `/raft/illuminance_1_10.illuminance` for the VEML7700.
- Plot `/raft/angle_1_36.data` for the AS5600.
- Inspect `/raft/imu_1_6a` as a `sensor_msgs/msg/Imu` stream.

#### Windows Foxglove + WSL Bridge

This setup works well: run ROS 2 and `foxglove_bridge` inside WSL, then run the
Foxglove Studio desktop app on Windows.

1. Install Foxglove Studio on Windows from <https://foxglove.dev/download>.
2. In WSL, confirm `examples/DemoSimple/run_dashboard.sh` can see the RaftROS
   topics.
3. In WSL, start `foxglove_bridge` with the command above.
4. In Windows Foxglove Studio, connect to `ws://localhost:8765`.

If `localhost` does not connect, get the WSL IP:

```bash
hostname -I
```

Then connect Foxglove Studio to:

```text
ws://<WSL_IP>:8765
```

For ESP32 discovery through WSL2, mirrored networking and permissive inbound
UDP firewall rules are usually required; see the host setup notes below.

#### Native Linux Foxglove

On native Linux, run both `foxglove_bridge` and Foxglove Studio on the same
machine. Install Foxglove Studio from <https://foxglove.dev/download>, start
the bridge with the command above, and connect to:

```text
ws://localhost:8765
```

Native Linux avoids the WSL multicast and firewall issues, but the machine
still needs to be on the same subnet and `ROS_DOMAIN_ID` as the ESP32.

### 4. QoS Note

RaftROS fast sensor streams use BEST_EFFORT / VOLATILE QoS by default. The
`DemoSimple` dashboard subscribes with compatible QoS. If Foxglove shows topics
but does not show samples, check whether `foxglove_bridge` is subscribing with
compatible QoS for the topic.

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
- `examples/DemoSimple/` — host-side dynamic ROS 2 dashboard for hot-plugged RaftROS device topics.
- `linux_unit_tests/` — Linux-hosted unit tests (388+ cases) and a standalone linux RTPS publisher (`raftros_standalone.cpp`) used as a non-embedded reference implementation.
- `devdocs/` — design overview, development status, and implementation plan.
  Start with `RaftROS-zenoh-milestone-results.md` (measured results: loop
  budget, memory, the 12 h soak, defects fixed) and
  `RaftROS-services-assessment.md` (what services would need, per transport),
  `RaftROS-services-implementation-plan.md` (Zenoh server-side services, done)
  and `RaftROS-parameters-implementation-plan.md` (ROS 2 parameters, done).

## Dependencies

- [RaftCore](https://github.com/robdobsn/RaftCore)
- [RaftSysMods](https://github.com/robdobsn/RaftSysMods) (StatePublisher, NetworkManager)

## License

MIT — see [LICENSE](LICENSE)
