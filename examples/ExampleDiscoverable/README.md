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

- Appears as a ROS 2 node on domain 0, named after the device: with no
  `nodeName` configured, RaftROS uses Raft's friendly name - the one set with
  `/api/friendlyname`, or until then the SysType's `DefaultName` (`RaftROS`
  here) with the last six hex digits of the MAC - made ROS-legal, e.g.
  `/raftros_adbfb6`. Two boards running the same image are two nodes.
- Publishes `std_msgs/msg/String` on `/chatter` (1 Hz by default, RELIABLE +
  VOLATILE) with a payload like `Hello from raftros_adbfb6 [N]`.
- Subscribes to `/chatter_in` and `/chatter_in2` and logs what arrives.
- **Auto-publishes every bus device** `DeviceManager` finds as a typed ROS 2
  topic `/raft/<slug>_<bus>_<addrHex>` - a VL6180 at I2C 0x29 on bus 1 appears
  as `/raft/range_1_29` (`sensor_msgs/msg/Range`). Unplugging withdraws the
  topic; plugging back in announces it again.
- On the Zenoh build (the default), also serves four ROS 2 **services** and
  the ROS 2 **parameter** services - see "Services" and "Parameters" below.
  None of the application code names a device: every service and parameter
  works with whatever `DeviceManager` has identified.

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

The board is described entirely in `SysTypes.json`, in the bus entry under
`DevMan`: the I2C pins (SDA 42 / SCL 41, the Feather's STEMMA QT connector)
and, in its `pwr` block, the GPIO that switches the connector's power
(GPIO 21, `TFT_I2C_POWER`). RaftI2C's bus power controller owns that pin: it
holds the connector off briefly at boot so devices start from a clean reset,
then powers it, and can power-cycle it later - there is no board code in
`main.cpp`.

```jsonc
"pwr": {
  "voltageLevels": ["off", "3V3"],
  "slotGroups": [{ "name": "STEMMA QT", "startSlotNum": 0, "numSlots": 1, "defaultLevelIdx": 1,
                   "levelsExclOff": [{ "vPins": [21], "on": [1] }] }]
}
```

Slot 0 is the bus itself (with no multiplexer there is only that slot). A
`vPins` entry is a pin on an I/O expander when one owns it, otherwise a plain
GPIO; omit the `pwr` block for a connector that is always powered. Driving a
plain GPIO this way needs RaftI2C's `BusPowerController` to hand pins no
expander owns to `RaftBusSystem::virtualPinsSet` (which falls back to GPIO) -
a 2026-10-11 change made in `raftdevlibs/RaftI2C`; until it is in RaftI2C
`main`, a build that fetches RaftI2C rather than using `raftdevlibs` logs
`FAILED to set pins` and leaves the connector unpowered. Change the pins for
another board, or add a SysType for it - `SysTypeFeatherS3` is an example.

## Try it (Zenoh build)

On the ROS 2 host (Jazzy, `sudo apt install ros-jazzy-rmw-zenoh-cpp`):

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd            # the router; keep it running
                                             #  (detach it from any ssh session)
export RMW_IMPLEMENTATION=rmw_zenoh_cpp      # in every shell that runs ros2
ros2 node list                               # /raftros_<mac6>
ros2 topic echo /chatter                     # one message a second
ros2 topic echo /raft/range_1_29             # Range samples (with a VL6180 attached)
ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello'}"
ros2 service call /<node>/devices std_srvs/srv/Trigger
ros2 param dump /<node>
```

The device must be told where the router is - see the next section; if it
cannot reach it, its log says so and how to fix it.

### Telling the device where the router is (Zenoh)

The router address is configuration, not part of the image. It is layered,
each level overriding the one before:

1. **Build default** - `CONFIG_RAFTROS_ZENOH_ROUTER_HOST` in menuconfig,
   empty as shipped (a build can bake one in).
2. **SysTypes** - `"routerHost"` / `"routerPort"` in the `RaftROS` block of
   `systypes/SysTypeMain/SysTypes.json`.
3. **Runtime, no rebuild** - the SysMod's own API, from any host on the
   network. The change is applied at once (a live session is dropped and the
   next connect uses the new address); `persist=1` also writes it to NVS for
   later boots:

   ```bash
   curl 'http://<device-ip>/api/ros/set?routerHost=192.168.1.50&persist=1'
   curl 'http://<device-ip>/api/ros/set?active=0'          # stop the node (until reboot)
   curl 'http://<device-ip>/api/ros/set?active=1&persist=1'
   curl http://<device-ip>/api/ros                           # status
   curl http://<device-ip>/api/ros/clear                     # drop the persisted RaftROS settings
   ```

   `POST /api/postsettings/reboot` with `{"RaftROS":{...}}` still works for
   any key, and `/api/getsettings/nv` shows what is persisted.

`routerHost` must be an IPv4 address: resolving a name would block the main
loop for as long as the DNS query takes, so the SysMod refuses a hostname and
says so in the log.

**An active node with no address waits**, and says so once in the log with
the `curl` line to set one; `/api/ros` shows `"conn":"unconfigured"`.

**If the router isn't there, the device says so.** After three failed connection
attempts, and every 30 s while it stays unreachable, the log carries a
`ROUTER UNREACHABLE` warning that names the address, says whether the host
answered (no router running there) or not (wrong address), says whether the
address is a build default that was set for another network, and gives the
`curl` line to change it. `GET /api/ros` reports the same:
`"routerSource":"default"|"config"`, `"routerReachable"`, `"connectFails"`,
`"lastSessionAgoS"`.

## Zenoh over the USB cable (optional serial link)

The same node, with no WiFi: the Zenoh session runs over the z-serial link on
the ESP32-S3's USB-Serial-JTAG port, and `rmw_zenohd` listens on that port
(`/dev/ttyACMn`) as well as on TCP. One cable carries flashing and ROS.
Tested on an Unexpected Maker FeatherS3 with an APDS-9960 and an MPU6050 on
its QWIIC connectors, against ROS 2 Lyrical.

The serial link is off unless chosen. Two things decide whether it can be:

- **The image.** The chip needs a USB-Serial-JTAG port (ESP32-S3, C3, C6...)
  and serial support built in (`CONFIG_RAFTROS_ZENOH_SERIAL_LINK`, on by
  default). The console stays on the USB port as usual: the link needs the
  port to itself - the router drops it on the first byte that is not a frame -
  so while the link is chosen RaftROS takes the port over. SerialConsole lets
  go of it (its `releasePort` command) and RaftCore's `RaftConsolePort`
  silences every other writer: Raft's log output, ESP-IDF's log output,
  `stdout`/`stderr` and the ROM `printf` channel. Choosing TCP again gives it
  all back. The console works normally until then - set up WiFi with
  `w/<SSID>/<password>`, and switch to the link from the console itself.
- **The configuration.** `link` is a RaftROS setting like `routerHost`: the
  value in the `RaftROS` block of SysTypes (`"tcp"`, the default, or
  `"usbjtag"`) is used at boot unless a persisted setting overrides it, and
  `/api/ros/set` changes it while running (`persist=1` keeps the change for
  later boots):

  ```bash
  curl 'http://<device-ip>/api/ros'                                # status: link, serialAvailable, counters
  curl 'http://<device-ip>/api/ros/set?link=usbjtag&persist=1'     # to the serial link, kept across reboots
  curl 'http://<device-ip>/api/ros/set?link=tcp&persist=1'         # back to TCP
  curl 'http://<device-ip>/api/ros/clear'                          # forget persisted RaftROS settings
  ```

  The same commands work typed at the console (`ros/set?link=usbjtag&persist=1`).
  Once the link has the port, the console is silent and does not read the
  port; to get it back, stop the router, open a terminal on the port and type
  `ros/set?link=tcp` (or `ros/clear`) - while it holds the port the link
  watches for those two commands among the incoming bytes. Opening the port
  with `raft monitor`, `screen` or the router does not reset the board;
  tools that pulse DTR/RTS (esptool) do.

  Application code can make the same change with `RaftROS::requestLinkConfig`.

  The change is applied on the next loop pass: a live session is closed and
  the node reconnects over the new link. An image that cannot use the serial
  link refuses with the reason. A board with no WiFi credentials can be
  reached on its access point (`RaftAP`, password `raftpassword`, device at
  `192.168.4.1`).

What `SysTypeFeatherS3` sets for the board:

- Two I2C buses, SDA 8 / SCL 9 on controller 0 and SDA 16 / SCL 15 on
  controller 1 (`i2cPort`). The second bus's `pwr` block switches GPIO 39,
  the enable of LDO2, which powers the vertical QWIIC connector; the first is
  on LDO1, always on. On the board tested both connectors were on the first
  bus.
- `DevTypes.json` adds APDS9960 (`/raft/illuminance_1_39`,
  `/raft/proximity_1_39`) and MPU6050 (`/raft/imu_1_68`) records.

Build with the whole repository mounted, since the example takes RaftROS from
four levels up (`raft build` in Docker mounts only this folder):

```bash
cd <RaftROS>
docker run --rm -u $(id -u):$(id -g) -e HOME=/tmp -v $PWD:$PWD \
    -w $PWD/examples/ExampleDiscoverable raftbuilder:idf-6.1 \
    bash -c 'git config --global --add safe.directory "*"; idf.py -B build/SysTypeFeatherS3 build'
cd examples/ExampleDiscoverable/build/SysTypeFeatherS3
esptool --chip esp32s3 -p /dev/ttyACM1 write-flash "@flash_args"
```

Then start the router with the board's port (use the stable
`/dev/serial/by-id` name; the board re-enumerates when it resets), and nothing
else may hold the port:

```bash
RAFTROS_SERIAL=/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00 \
    docker/distros/demo.sh lyrical
docker/distros/ros2z node list          # /raftros_<mac6>
docker/distros/ros2z topic hz /raft/imu_1_68
```

Without the container, the same endpoint goes in a copy of
`rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5`:

```json5
listen: { endpoints: ["tcp/[::]:7447", "serial//dev/serial/by-id/usb-Espressif_..._<MAC>-if00#baudrate=921600"] }
```

and `ZENOH_ROUTER_CONFIG_URI=<that file> ros2 run rmw_zenoh_cpp rmw_zenohd`.

**Bring-up with logs.** With `"linkLogs": 1` (SysTypes, or
`/api/ros/set?linkLogs=1`) the firmware also writes its log
lines onto the link between frames. A real router would drop the link on
them, so use the bridge instead, which relays frames to a TCP router and
prints everything else:

```bash
python3 tools/zenoh_serial_link.py bridge --serial /dev/serial/by-id/... --router 127.0.0.1:7447
```

`/api/rosstat` (over WiFi, when the board has credentials) adds the link's
counters: `serIn`/`serOut` frames, `serCrcErr`, `serBadFrame`, `serInits`,
`serInitsHeld`, `serLinkUps`, `serResets`, `serTxFull`. The investigation and
measurements are in
[RaftROS-zenoh-serial-investigation-plan.md](../../devdocs/RaftROS-zenoh-serial-investigation-plan.md).

## Try it (RTPS build)

On a ROS 2 host on the same subnet (no router needed):

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # or rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY                     # Jazzy: a leftover value restricts discovery
ros2 node list --no-daemon                   # /raftros_<mac6>
ros2 topic echo /chatter std_msgs/msg/String --no-daemon
ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range --no-daemon --qos-reliability best_effort
```

With CycloneDDS pass `--no-daemon` and the message type, and expect the first
query to miss the node for a few seconds (each CLI call is a new
participant). Under WSL2, discovery also needs mirrored networking and a
firewall rule - see "Host Setup Notes" in the [top-level README](../../README.md#host-setup-notes-read-first-if-discovery-isnt-working).

## Services (Zenoh build)

The example serves four ROS 2 services, declared relative to the node
(`"~/devices"` becomes `/<node>/devices`), so they follow its name:

| Service | Type | Does |
| --- | --- | --- |
| `~/devices` | `std_srvs/srv/Trigger` | Reports attached devices, chatter state, heap |
| `~/read` | `std_srvs/srv/Trigger` | A fresh reading from every attached device - a *deferred* reply (below) |
| `~/chatter_enable` | `std_srvs/srv/SetBool` | Starts / stops `/chatter` |
| `~/ping` | `std_srvs/srv/Empty` | Nothing - a round-trip check |

```bash
ros2 service list -t                         # expect the four, plus the parameter services
ros2 service call /<node>/devices std_srvs/srv/Trigger
#   success=True, message='2 device(s) attached, chatter on, chatter_in rx 0/0, heap free 123968 B'
ros2 service call /<node>/chatter_enable std_srvs/srv/SetBool "{data: false}"
ros2 topic echo --once /chatter              # nothing arrives until ...
ros2 service call /<node>/chatter_enable std_srvs/srv/SetBool "{data: true}"
```

A handler runs on the main loop and answers from state it already holds
(`MainSysMod.cpp`):

```cpp
pRaftROS->addService("~/chatter_enable", "std_srvs::srv::dds_::SetBool_",
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
`completeService(token, reply)`. `~/read` shows this, for any devices:

```bash
ros2 service call /<node>/read std_srvs/srv/Trigger
#   success=True, message='APDS9960 1_39: als=3740 red=1055 green=1162 blue=1342 prox=0;
#                          MPU6050 1_68: ax=0.6033 ay=0.3812 az=0.5671 temp=28.42 gx=-2.87 gy=0 gz=-0.6718'
```

The handler notes how many samples each attached device has delivered and
parks the request; `MainSysMod::loop()` completes it once every device has
delivered another, so each reading is taken after the call, not served from a
cache. If a device has not delivered one by the `readTimeoutMs` parameter
(2 s by default), the reply lists it as `no new reading` and has
`success=False`. A second call while one is parked is refused.

Nothing in it is specific to a device. `MainSysMod` follows `DeviceManager`'s
status callbacks to keep a table of identified devices and subscribes to each
one's data with `DeviceManager::registerForDeviceData`; RaftCore fans each
sample out to every subscriber, so this does not disturb auto-publishing
(RaftCore `78781c0` or later - before that a second registration silently
replaced the auto-publisher's and stopped the topic). A sample is decoded
with the device type record's own decoder (`pollResultDecodeFn`) and listed
from its field table (`pollFieldDescs`: name, type, divisor, addend), scaled
as RaftROS scales the device's topic. Plug in a sensor Raft can identify and
it appears in the reply, as it appears as a topic.

(`peekLatestDecodedPollResponse` returns the latest result without consuming
it, but the service wants a sample taken after the request.
`getLatestDecodedPollResponse` is not a peek: it reports only a result that is
new since its last call, so a second reader sees nothing.)

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
SAMPLE #1 "Hello from raftros_adbfb6 [35]"
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
ros2 param list /<node>
ros2 param dump /<node>
#   chatterEnable: true
#   chatterPeriodMs: 1000
#   readTimeoutMs: 2000
#   routerHost: 192.168.86.192
#   use_sim_time: false
ros2 param set /<node> chatterPeriodMs 250    # /chatter now at 4 Hz
ros2 param set /<node> chatterEnable false
ros2 param describe /<node> routerHost
ros2 param set /<node> routerHost 192.168.86.50
```

| Parameter | Type | Effect |
| --- | --- | --- |
| `chatterEnable` | bool | Starts and stops `/chatter` at once |
| `chatterPeriodMs` | int | Period of `/chatter`, 100-60000 ms |
| `routerHost` | string | Zenoh router IPv4 address. **Persisted** in the same settings overlay `/api/postsettings` writes; the node reconnects to it once the reply has gone out |
| `readTimeoutMs` | int | The example's own: how long `~/read` waits for every device, 100-4500 ms |
| `use_sim_time` | bool | Read-only `false` (every ROS 2 node has it) |

A refused set reports why, in the words a stock ROS 2 node uses
(`Wrong parameter type, expected 'Type.INTEGER' got 'Type.STRING'`,
`Trying to set a read-only parameter: use_sim_time.`) or the parameter
owner's own (`chatterPeriodMs must be 100-60000`, `readTimeoutMs must be
100-4500 ...`). Only `chatterEnable` and
`chatterPeriodMs` of the SysMod's settings are live as parameters;
`active`, `routerHost` and `routerPort` are live through `/api/ros/set`, and
the rest take effect through `/api/postsettings` and a reboot as before.

An application declares its own parameters on the SysMod and either reads
them where they are used or takes a callback that can refuse a value:

```cpp
pRaftROS->declareParameter("readTimeoutMs", (int64_t)2000,
    "How long ~/read waits for a fresh reading from every device, in ms", false,
    [](const RaftROS::ParamValue& value, const char*& reason)
    {
        if (value.integerValue < 100 || value.integerValue > 4500)
        {
            reason = "readTimeoutMs must be 100-4500 (RaftROS times a deferred request out at 5 s)";
            return false;
        }
        return true;
    });
...
const RaftROS::ParamValue* pTimeout = pRaftROS->parameter("readTimeoutMs");
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
| `enable` | the `alwaysEnable` argument of `registerSysMod` | Whether SysManager creates the SysMod at all (with it off there is no node and no `/api/ros`) |
| `active` | `enable`'s value, else 0 | Whether the node runs: connects to the router, publishes, serves. Changed live with `/api/ros/set?active=0\|1` |
| `domainId` | 0 | ROS 2 domain |
| `nodeName` / `nodeNamespace` | the device's friendly name, made ROS-legal (`raftros_<mac6>` with this example's `SysManager.DefaultName`) / `/` | The node's name; change the default with `/api/friendlyname` or set `nodeName` |
| `chatterEnable` | true | Publish `/chatter` |
| `qosProfiles` | built-ins | Per-topic and per-class QoS (below) |
| `routerHost` / `routerPort` | Kconfig (`CONFIG_RAFTROS_ZENOH_ROUTER_HOST`, empty as shipped) / 7447 | Zenoh router, IPv4 only; also set live with `/api/ros/set` |
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

With the node active and `DevMan.enable = 1`, every device that
`DeviceManager` brings online (I2C, BLE, ...) is automatically mirrored as a
ROS 2 publisher. No per-device code, no per-topic config needed for the
common case.

### Topic + type rules

- Topic name: `/raft/<slug>_<bus>_<addrHex>`
  (e.g. `/raft/imu_1_6a`, `/raft/temperature_1_38`).
- Message type is chosen by first-match on the device's `clas[]` tags and
  device type name (see `AutoPub/AutoPubClassMap.h`):
  - `{ACC, GYRO}` → `sensor_msgs/msg/Imu` (single writer).
  - `{TEMP, RH}` → **two writers**: `Temperature` + `RelativeHumidity`;
    with `CO2` as well (SCD30/SCD40/STCC4) a **third**, `std_msgs/Float32`
    on slug `co2`.
  - `{PRES, TEMP}` → **two writers**: `FluidPressure` + `Temperature`.
  - `{PROX, LGHT}` (VCNL4040) → **two writers**: `Illuminance` + `Range`
    on slug `proximity`.
  - Single-class rules: TEMP, RH, PRES → the `sensor_msgs` type; LGHT →
    `Illuminance`; PROX, DIST → `Range`; ACC → `Imu` (accelerometer only);
    MAG → `MagneticField` (µT → T); ANG, ANGL, SOIL, CO2, O2, WGHT →
    `std_msgs/Float32`; ROT → `Int32`; BTN → `Bool`; TCH → `ByteMultiArray`;
    FRCE → `geometry_msgs/Wrench`; HRM → `Float32MultiArray`; GAME →
    `sensor_msgs/Joy`; FUEL, BATT → `sensor_msgs/BatteryState`; SRVO, MOTR,
    PUMP → `sensor_msgs/JointState` (an actuator that reports its angle,
    velocity and current back is a joint: position in rad, effort as the
    device gives its current).
  - The ST multizone ToFs (VL53L5CX / L7CX / L8CX) → **two writers**: a
    `sensor_msgs/Image` (`32FC1`, 4×4 or 8×8, metres, NaN where the zone has
    no valid target) on slug `depth`, and a `Range` of the nearest valid zone
    on slug `range`.
  - Output-only devices (PIX, LED) publish nothing.
  - Any other class falls back to `std_msgs/msg/Float64MultiArray`, topic
    slug `data`: every decoded field as a float64, with a
    `layout.dim` entry labelled with the field's name, so a value can be
    picked by name (`ros2 topic echo` shows the labels).
- Unit scaling follows REP-103 (g→m/s², °/s→rad/s, mm→m, hPa→Pa,
  %→0..1). `Header.stamp` is taken from the poll record's `timeMs` field,
  so subscribers see sample-time not emit-time.

### QoS profiles

Four built-in profiles (see `AutoPub/AutoPubQoSProfile.h`):

| Profile           | Reliability | Durability       | Depth | Default for                                      |
|-------------------|-------------|------------------|-------|--------------------------------------------------|
| `fast_sensor`     | BEST_EFFORT | VOLATILE         | 10    | ACC, GYRO, IMU, PROX, LGHT, DIST, ANG, ANGL, MAG, HRM, FRCE, SRVO, MOTR, PUMP |
| `slow_sensor`     | RELIABLE    | VOLATILE         | 5     | TEMP, RH, PRES, SOIL, BTHM, CO2, O2, VOC, FUEL, BATT |
| `event`           | RELIABLE    | TRANSIENT_LOCAL  | 20    | BTN, TCH, ROT, GAME                              |
| `fallback_string` | RELIABLE    | VOLATILE         | 10    | any class not listed                             |

Override in SysTypes (resolution order: per-device alias → per-class
override → built-in default):

```jsonc
"RaftROS": {
  "active": 1,
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
