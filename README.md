# RaftROS

Native ROS 2 node functionality for ESP32, as a [Raft](https://github.com/robdobsn/RaftCore)
SysMod.

RaftROS makes a Raft ESP32 application a first-class ROS 2 node - no
micro-ROS, no agent process. It talks to ROS 2 over **Zenoh** (through an
`rmw_zenoh` router; the default) or directly over **RTPS/DDS** (chosen at
build time), publishes every bus device that Raft's `DeviceManager` finds as
a typed ROS 2 topic, and lets application code subscribe to topics, serve
ROS 2 services and expose ROS 2 parameters.

**Start with the example:** [`examples/ExampleDiscoverable`](examples/ExampleDiscoverable/README.md)
is a complete application that uses every feature, with the exact `ros2`
commands to try each one. The rest of this page is about using RaftROS in
your own application.

## Status

Verified on an ESP32-S3 against ROS 2 Jazzy
([measured results](devdocs/RaftROS-zenoh-milestone-results.md)). The same
Zenoh firmware also passes the full functional, robustness, load and hot-plug
tests against **Kilted** and **Lyrical**
([distribution support](devdocs/RaftROS-ros-distro-support.md)):

| | Zenoh (default) | RTPS/DDS |
| --- | --- | --- |
| Node in the graph, `/chatter`, string subscriptions | yes | yes |
| Every `DeviceManager` bus device auto-published as a typed topic | yes | yes |
| QoS profiles (`qosProfiles`) for publishers and subscriptions | yes | yes |
| Device hot-plug (topic withdrawn and re-announced) | yes | yes |
| ROS 2 **services** (`std_srvs` Trigger / SetBool / Empty, deferred replies) | yes | no |
| ROS 2 **parameters** (`ros2 param list/get/set/describe/dump`) | yes | no |
| Needs on the ROS side | an `rmw_zenohd` router | nothing (FastDDS, CycloneDDS) |
| Worst main-loop pass, unattended | ~13 ms | ~15 ms |

Against the Raft SysMod contract of 50 ms worst case. Two 12-hour soaks on
Zenoh (the second with a service or parameter call every few seconds) ran
with one session throughout, no failed calls and free heap flat. Image
~1.28 MB (28% of a 1.7 MB app partition free); free heap ~147 kB with
services and parameters. Host tests: the unit suite and the Zenoh session
(1298), codec (2223) and firmware-piece (141) suites.

## Using RaftROS in your own application

### 1. Add the component

RaftROS is an ESP-IDF component (`components/RaftROS`). In a Raft project
(`raft new` from the [Raft CLI](https://github.com/robdobsn/RaftCLI)), make it
visible to the build and depend on it - as the example does:

```cmake
# systypes/Common/features.cmake
set(RAFT_COMPONENTS
    RaftCore@main          # RaftCore 6e5bb96 or later (AttrFieldDesc with element count)
    RaftSysMods@main
    RaftWebServer@main     # for /api/rosstat and settings
    RaftI2C@main           # if you have I2C devices
)
list(APPEND EXTRA_COMPONENT_DIRS "<path-to>/RaftROS")   # or add it as a git submodule
```

```cmake
# main/CMakeLists.txt
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "."
    REQUIRES RaftCore RaftSysMods RaftWebServer RaftI2C RaftROS)
```

### 2. Choose the transport

Kconfig option `RAFTROS_BACKEND` - Zenoh by default. For RTPS, put this in
your `sdkconfig.defaults` (and delete the build's `sdkconfig`):

```
CONFIG_RAFTROS_BACKEND_RTPS=y
```

For Zenoh the router's address is configuration, not part of the image:
`routerHost` in SysTypes, or a setting posted to the running device (see
step 4). `CONFIG_RAFTROS_ZENOH_ROUTER_HOST` (and `..._PORT`, default 7447)
exists for a build that wants a baked-in default; it is empty as shipped.

Two sdkconfig settings the example uses and you probably want:
`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1=y` (keeps the SysMod loop off the WiFi
core; without it WiFi reconnects stall the loop ~275 ms) and
`CONFIG_ESP_MAIN_TASK_STACK_SIZE=10000`.

### 3. Register the SysMod

```cpp
#include "RaftCoreApp.h"
#include "RegisterSysMods.h"
#include "RaftROS.h"
#include "BusI2C.h"

RaftCoreApp raftCoreApp;

extern "C" void app_main(void)
{
    RegisterSysMods::registerSysMods(raftCoreApp.getSysManager());
    RegisterSysMods::registerWebServer(raftCoreApp.getSysManager());
    raftBusSystem.registerBus("I2C", BusI2C::createFn);
    raftCoreApp.registerSysMod("RaftROS", RaftROS::create, true);
    // ... your own SysMods
    while (1)
        raftCoreApp.loop();
}
```

### 4. Configure it

In the `RaftROS` block of your SysTypes JSON (with a `DevMan` block declaring
your buses, for auto-publishing):

```json
"RaftROS": {
    "active": 1,
    "domainId": 0,
    "nodeName": "my_robot",
    "nodeNamespace": "/",
    "routerHost": "192.168.1.50",
    "qosProfiles": { "classDefaults": {} }
}
```

`active` says whether the node runs; the SysMod itself is always there (it
was registered with `alwaysEnable`), so a product can ship with `active` off
and no router address, and be switched on and pointed at a router from the
network, with nothing rebuilt or reflashed:

```bash
curl 'http://<device-ip>/api/ros/set?active=1&routerHost=192.168.1.50&persist=1'
curl http://<device-ip>/api/ros          # status: active, router, session state, counters
curl http://<device-ip>/api/ros/clear    # drop the persisted settings; SysTypes applies again
```

`ros/set` takes `active`, `routerHost` (IPv4), `routerPort` and `persist`;
the change is applied on the next loop pass, and with `persist=1` it is also
written to NVS for later boots. The other keys can be changed with
`POST /api/postsettings/reboot` as before. On Zenoh an active node with no
router address says so once in the log and waits; one that cannot reach its
router reports the cause and the fix. The full list of keys is in the
[example's Configuration section](examples/ExampleDiscoverable/README.md#configuration).

At this point, with no application code, the node appears in ROS 2, publishes
`/chatter` and auto-publishes every bus device.

### 5. Use it from application code

Get the SysMod from any other SysMod's `setup()`:

```cpp
RaftROS* pRos = static_cast<RaftROS*>(getSysManager()->getSysMod("RaftROS"));
```

**Subscribe to a topic** (both transports; `std_msgs/String`):

```cpp
pRos->addStringSubscription("rt/cmd", "std_msgs::msg::dds_::String_",
    [](const uint8_t* writerEID, const uint8_t* srcGuid, const char* text, uint32_t len) {
        LOG_I("App", "cmd: %s", text);
    });
```

**Serve a service** (Zenoh). A handler runs on the main loop and must not
block; one that needs a bus reading returns `Deferred` and finishes later with
`completeService(token, reply)`:

```cpp
pRos->addService("/my_robot/reset", "std_srvs::srv::dds_::Trigger_",
    [](const RaftROS::ServiceRequest& req, RaftROS::ServiceReply& reply) {
        reply.fields.success = true;
        reply.fields.message = "done";
        return RaftROS::ServiceOutcome::Replied;
    });
```

**Declare a parameter** (Zenoh) - `ros2 param set` reaches it; the optional
callback can refuse a value:

```cpp
pRos->declareParameter("gain", 1.5, "Controller gain", false,
    [](const RaftROS::ParamValue& v, const char*& reason) {
        if (v.doubleValue < 0) { reason = "gain must be >= 0"; return false; }
        return true;
    });
double gain = pRos->parameter("gain")->doubleValue;
```

On the RTPS build these compile to stubs that log a warning, so one
application builds for both transports.

**Read a device's latest value** without disturbing auto-publishing: peek it
with `RaftBusDevicesIF::getLatestDecodedPollResponse`, or subscribe with
`DeviceManager::registerForDeviceData` (RaftCore fans samples out to every
subscriber).

The example's [`MainSysMod.cpp`](examples/ExampleDiscoverable/components/MainSysMod/MainSysMod.cpp)
does all of these.

### 6. Talk to it from ROS 2

Zenoh:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd          # once, on a host the device can reach
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 node list && ros2 topic list && ros2 param dump /my_robot
```

RTPS: any ROS 2 host on the same subnet with FastDDS or CycloneDDS - see
[Host Setup Notes](#host-setup-notes-read-first-if-discovery-isnt-working)
if discovery doesn't work.

### Rules worth knowing

- **Nothing blocks the main loop.** Handlers, parameter callbacks and
  subscription callbacks run on it; keep them short.
- **Logging costs loop time.** With nothing reading a USB-Serial-JTAG console
  each log line stalls ~10 ms; RaftROS writes at most one per loop pass. Read
  loop figures from `GET /api/rosstat` with no terminal attached.
- **`routerHost` is an IPv4 address** - a DNS lookup would block the loop.
- `GET /api/rosstat` has the session, counters, heap, stack and per-phase loop
  maxima; it is the thing to watch in a long run.

## Features

- **Zenoh transport** (default): an `rmw_zenoh`-compatible session with
  liveliness tokens and interests, reconnect that re-declares everything, a
  layered router address and an actionable warning when it is unreachable.
- **RTPS transport**: a clean-room RTPS 2.2 participant (MIT licensed) -
  SPDP/SEDP discovery, reliable QoS with HEARTBEAT/ACKNACK retransmit,
  VOLATILE and TRANSIENT_LOCAL.
- **Automatic ROS 2 publishing of every `DeviceManager` bus device** -
  `clas[]`-driven message types, REP-103 SI units, composite devices on
  several topics, per-class QoS profiles with SysTypes override, hot-plug.
- **ROS 2 services** (Zenoh): `std_srvs` Trigger / SetBool / Empty servers;
  deferred replies; busy, bad-request and timeout errors instead of hangs.
- **ROS 2 parameters** (Zenoh): the six `rcl_interfaces` services; the
  node's own settings (`chatterEnable`, `chatterPeriodMs`, `routerHost` -
  persisted) and any the application declares.
- CDR serialisation for the standard ROS 2 message types.
- Planned: subscriptions for actuator classes (SRVO/MOTR/PUMP/PIX) wired to
  `DeviceManager`, so ROS 2 can drive them (they already publish their state
  as `JointState`); ROS 2 actions; parameter events.

## Demo: ExampleDiscoverable + DemoSimple + Foxglove

The DemoSimple dashboard and the Foxglove commands below are written for the
RTPS build (FastDDS); with the Zenoh build set
`RMW_IMPLEMENTATION=rmw_zenoh_cpp` instead and run a router.

[`examples/ExampleDiscoverable`](examples/ExampleDiscoverable/README.md) is the ESP32 firmware demo. When flashed to an
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

These apply to the RTPS build, which depends on UDP multicast; the Zenoh
build needs only a TCP connection to the router.

### RTPS CLI caveat (historical)

> **Resolved:** on native Linux (2026-09-27) `ros2 node info /raft_esp32` and
> `ros2 topic info -v` attribute the node correctly over both FastDDS and
> CycloneDDS. The note below is kept for WSL setups that still show it.

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

### Discovery gotchas

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

- `components/RaftROS/` - the SysMod (`RaftROS.h` selects the backend).
  - `Zenoh/` - the Zenoh backend: session, wire messages, ROS key and token codec.
  - `RTPS/` - the RTPS backend and its runtime (`runtime/{discovery,reliability,announce,receive,schedule,wire,core}`).
  - `AutoPub/` - shared by both: device auto-publishing, message mapping and QoS,
    the service registry and codecs, the parameter store.
  - `CDR/` - CDR encoder/decoder.
- `examples/ExampleDiscoverable/` - the reference application (above).
- `examples/DemoSimple/` - host-side dashboard for hot-plugged device topics.
- `tools/` - a Zenoh router stub and a Zenoh subscriber, for testing without ROS.
- `linux_unit_tests/` - host test suites (`make` targets: `all`, `zenoh-test`,
  `zenoh-session-test`, `zenoh-autopub-test`), with captured-traffic fixtures.
- `devdocs/` - start with [RaftROS-zenoh-milestone-results.md](devdocs/RaftROS-zenoh-milestone-results.md)
  (measured results) and [RaftROS-development-status.md](devdocs/RaftROS-development-status.md)
  (the project log); the [services](devdocs/RaftROS-services-implementation-plan.md)
  and [parameters](devdocs/RaftROS-parameters-implementation-plan.md) plans
  record the wire contracts.

## Dependencies

- [RaftCore](https://github.com/robdobsn/RaftCore) - `78781c0` or later (device-data fan-out, `RaftJson` escape fix, NVS init order)
- [RaftSysMods](https://github.com/robdobsn/RaftSysMods) - networking
- [RaftWebServer](https://github.com/robdobsn/RaftWebServer) - `/api/rosstat` and settings endpoints
- [RaftI2C](https://github.com/robdobsn/RaftI2C) - for I2C devices
- ESP-IDF 6.0 (tested with 6.0.2); ROS 2 Jazzy, Kilted or Lyrical on the host
  (`ros-<distro>-rmw-zenoh-cpp` for Zenoh; `docker/distros/` builds a test
  container for each)

## License

MIT — see [LICENSE](LICENSE)
