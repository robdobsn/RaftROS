# RaftROS Zenoh over Serial: Investigation Plan

**Date:** 2026-10-09
**Status:** Working on hardware (2026-10-09). The native Zenoh backend runs
over the z-serial link on an ESP32-S3's USB-Serial-JTAG port to a stock
`rmw_zenohd` (ROS 2 Lyrical, apt) listening on that port - no bridge, no
second cable. See [Results](#results-2026-10-09); the plan below is kept as
written, with notes where the results changed it.
**Scope:** Run the RaftROS Zenoh backend (Raft's own session, with zenoh-pico
as a proving tool) over a serial link to the `rmw_zenohd` router instead of
WiFi/TCP. The test bench is an ESP32-S3 Feather with an I2C device, cabled
to the Linux ROS box.
**Non-goals:** micro-ROS (it needs a DDS agent on the host and is invisible to
an rmw_zenoh ROS graph without `zenoh-bridge-ros2dds`), SLIP/PPP (removed from
ESP-IDF 5.x; `eppp_link` works but puts a full IP stack and a host `pppd`
under an unchanged TCP session for no gain), BLE (no Zenoh BLE link exists
anywhere - see [Follow-ons](#follow-ons)).

Related: [overview](RaftROS-overview.md),
[Zenoh implementation plan](RaftROS-zenoh-implementation-plan.md),
[zenoh-pico comparison](RaftROS-zenoh-pico-comparison.md),
[development status](RaftROS-development-status.md).

## Results (2026-10-09)

**Bench as built:** Unexpected Maker FeatherS3 (ESP32-S3, 16 MB flash, 8 MB
PSRAM) with an APDS-9960 and an MPU6050 on its QWIIC connectors, one USB
cable to the Linux box (`base8ubuntu`), ROS 2 Lyrical in the
`raftros-lyrical` container. With no USB-UART adapter to hand, the link is
the S3's **USB-Serial-JTAG** port (planned as a Step 5 follow-on); since
2026-10-11 the console stays on that port and RaftROS takes it over only
while the link is chosen (see "The console and the link share the port"). Firmware: `examples/ExampleDiscoverable`, SysType
`SysTypeFeatherS3` (renamed from `SysTypeFeatherS3Serial` on 2026-10-10, when
the link became a run-time choice).

| Check | Result |
| --- | --- |
| Step 0: router has serial | `ros-lyrical-rmw-zenoh-cpp` 0.10.6: `libzenohc.so` contains `zenoh-link-serial`; a `serial//...` listen endpoint starts, beside `tcp/[::]:7447` |
| pty proof (no board) | `tools/zenoh_serial_link.py relay` + the TCP session probe, and `zenoh_serial_tests --serial` (the C++ link and session) both hold a session with `rmw_zenohd` over a socat pty; `ros2 topic echo` decodes a Range sent over it |
| Negotiated batch | 1500 (the device now offers 1500 on a serial link) |
| Graph | the node (named after the device since 2026-10-11); `/raft/imu_1_68` (`sensor_msgs/Imu`), `/raft/illuminance_1_39` (`Illuminance`), `/raft/proximity_1_39` (`Range`), `/chatter`, `/chatter_in[2]` |
| Rates | IMU 18.7 Hz (20 Hz poll), APDS-9960 9.6 Hz (10 Hz poll), `/chatter` 1.0 Hz, 4.0 Hz after `param set chatterPeriodMs 250` |
| Services, parameters | Trigger, SetBool, Empty and `param dump/set` all answer over serial; three `/chatter_in` messages arrive (`chatter_in rx 3/0`) |
| Router restart | node back within ~1 s of the router listening again |
| Board reset with router down, router started later | node in the graph 1.6 s after the router starts |
| Image | 1.30 MB, 26% of the app partition free; heap free ~126 kB |
| 10-minute soak | Trigger call every 5 s: 115/115 answered, heap free 126,700 B at start and end, IMU still 18.8 Hz at the end |
| Link chosen at run time (2026-10-10) | an image booted on `"link":"tcp"` moved to serial after a link change through the REST dispatcher (then `/api/roslink/set?link=usbjtag` in MainSysMod; since 2026-10-11 the same keys are on RaftROS's own `/api/ros/set`); with `persist=1` a re-flashed image came up on serial; a USB-console image (`SysTypeMain`) refuses with the reason |

Not yet done: board reset while the router holds the port (the router opens
it exclusively, so it needs the reset button or a cable pull), the 12-hour soak,
loop-timing figures from `/api/rosstat` (this board has no WiFi credentials,
so the API is unreachable; SysMan's own report showed RaftROS at ~1 ms
typical), and baud/throughput limits (USB-Serial-JTAG ignores baud).

### What the hardware changed in the plan

- **The router tears the link down on any bad frame.** z-serial's
  `read_msg` returns an error for a COBS, length or CRC failure, and
  `zenoh-link-serial` closes the link on a read error. It tolerates junk only
  while waiting for the handshake. So the drop-and-resync sequence policy
  proposed below is moot on the router side, and the device keeps its strict
  `SequenceMismatch` (symmetrical: a corrupted frame costs a session either
  way, recovered in about a second). Over USB the link is CRC-checked by USB
  itself, so corruption is not expected.
- **Nothing else may write to the port.** For the same reason, while the
  link is on, nothing else may touch the port. At first that meant moving the
  console to UART0 at build time; since 2026-10-11 it is done at run time -
  see "The console and the link share the port" below. The ROM bootloader
  still prints to USB at reset; the router is then waiting for a handshake
  and ignores it.
- **Stale I frames.** Until the host opens the port, frames queue in the
  device's USB TX ring. Retrying I every 250 ms filled it, and when a
  listener opened the port it got a burst of I frames and answered
  alternately I|A and R (z-serial does the same), so the link flapped for
  several seconds. The device now sends another I only once the last has left
  (`usb_serial_jtag_wait_tx_done(0)`).
- **Batch size.** Through a TCP bridge the router negotiated 4096, larger
  than a serial frame; `ZenohTCPSession::start` now takes the batch to offer
  and the serial link offers 1500.
- **Step 2 (zenoh-pico proof) skipped.** Its ESP-IDF serial backend drives a
  UART, not USB-Serial-JTAG, so it would have needed more than the pin patch;
  the Python relay and the host-built C++ link proved the router side instead.
- **Board reset and USB re-enumeration.** `release_on_close` defaults to
  true, so the listener reopens the path on every accept, and it keeps
  retrying while the device is absent. A `/dev/serial/by-id/...` path keeps
  the name stable; colons in it are fine in the locator.

### The console and the link share the port (2026-10-11)

The link no longer needs a console-on-UART build. Everything that writes to
or reads from USB-Serial-JTAG in a normal Raft image, found by searching
RaftCore, RaftI2C, RaftSysMods, RaftWebServer and RaftROS:

| Writer / reader | Route |
| --- | --- |
| Raft `LOG_x` | `LoggerCore` - direct USB-Serial-JTAG writes once the driver is installed, `printf` before |
| ESP-IDF `ESP_LOGx` (WiFi, MQTT, LogManager, DNSResolver, WebServer...) | SerialConsole's `esp_log_set_vprintf` hook, writing to the driver |
| Console replies, RICSerial protocol | `SerialConsole::putStr` / `sendMsg` |
| Console input (would eat the router's frames) | `SerialConsole::getChar` |
| Anything else on `stdout`/`stderr` | VFS -> driver |
| ROM `printf` (`esp_rom_printf`, `ESP_EARLY_LOG`, `ESP_DRAM_LOG`, often from ISRs) | straight to the USB hardware |

No other `printf`, `puts`, `std::cout`, `ets_printf` or `esp_rom_printf` is in
the Raft libraries except `tinyexpr`'s unused `te_print` and RaftROS's
RTPS-only hex dump. Panic output happens at reset, when the link goes anyway.

So, when the link is chosen (at boot from settings, from the API, or typed
at the console):

1. **RaftSysMods `SerialConsole`** gets `{"cmd":"releasePort"}` through
   `SysManager::sendCmdJSON`: it stops reading the port and stops writing
   (`putStr`, `sendMsg`) until `{"cmd":"reclaimPort"}`. The driver stays
   installed and RaftROS uses it.
2. **RaftCore `RaftConsolePort::claim()`** silences the rest: `LoggerCore`
   stops writing to the console (other loggers - LogManager's network
   loggers - still get every line), the ESP-IDF log hook is replaced (the
   previous one is saved), `stdout` (one FILE shared by every task) is
   `freopen`ed to `/dev/null`, and ROM printf channel 1 goes to a no-op.
   `release()` restores each in turn, ROM output as ESP-IDF's startup sets
   it for the console.
3. The link then starts its handshake. Text already queued for the console
   reaches the router while it is waiting for an I frame, which it ignores.

Choosing TCP (or deactivating the node) reverses this. With the port claimed
the link also watches the incoming bytes for a typed line `ros/set?...` or
`ros/clear` and passes it to the REST API, so the console can be had back
from a terminal with the router stopped; on a taken line it resyncs its
decoder so a frame that follows is not lost.

RaftROS refuses the link, saying why, if this RaftCore has no
`RaftConsolePort` (`__has_include`) or SerialConsole does not accept
`releasePort`, so older libraries still build and behave as before. An image
whose console is on UART needs none of this and claims nothing.

Tested on the FeatherS3 (console on USB, `SysTypeFeatherS3`):

| Check | Result |
| --- | --- |
| Boot with `link=usbjtag` persisted | console output to 1.37 s, the "taking the USB port" line, then only I frames - 48 frames, 0 bytes of anything else in 13 s, across a SysMan report |
| 3 minutes of parameter sets, `~/read` and SetBool calls with the board logging | 423/423 calls answered, no serial errors in the router log, IMU 18.6 Hz |
| `ros/set?link=tcp` typed with the router stopped | "USB port given back to the console", SysMan reports and the endpoint list on the console again |
| `ros/set?link=usbjtag` typed at a live console (no reset, uptime 28 s) | reply printed, then the claim, 21 frames and 0 stray bytes; router sees the node, `~/read` answers, IMU 18.6 Hz |
| `ros/set?link=tcp&persist=1`, reset | console stays live, no claim |
| WiFi, TCP router (normal operation, console on `raft m --agent-stream`) | node, services, `/chatter`, IMU 18.6 Hz; console shows the session; `loopMaxUs` 4019 |
| `/api/ros/set?link=usbjtag` over HTTP | ready on serial in ~2 s; 285/285 calls in 2 min, IMU 18.5 Hz, batch 1500, `serCrcErr` 0, `loopMaxUs` 4476, heap 122.9 kB |
| `/api/ros/set?link=tcp` over HTTP | back on TCP in ~7 s; `~/read` answers; console live again (endpoint list, `ros` reply, SysMan reports); router restart recovered |

### Pieces

- `components/RaftROS/Zenoh/ZenohSerialFramer.h` - CRC, COBS, frame encode,
  byte-fed decoder with counters (host-tested against z-serial's vectors and
  a corruption fuzz).
- `components/RaftROS/Zenoh/ZenohSerialLink.h` - the I / I|A / R handshake
  and one-batch-per-frame I/O over any non-blocking byte port.
- `ZenohTCPSession` - `receiveBatch()` and `pendingBatch()` for datagram
  links; `start()` takes the batch size to offer.
- `RaftROSZenoh` - `"link": "tcp" | "usbjtag"` in the `RaftROS` SysTypes
  block (default tcp), changeable at run time with
  `/api/ros/set?link=tcp|usbjtag&linkLogs=0|1&persist=0|1` (or
  `RaftROS::requestLinkConfig` from application code) and persisted on
  request; `/api/ros` reports `link`, `serialAvailable` and the reason when it
  is not; `CONFIG_RAFTROS_ZENOH_SERIAL_LINK`
  builds the support in (default y), and the link's ~4.5 kB of buffers are
  allocated only when it is first used; serial counters in `/api/rosstat` (`serIn`, `serOut`, `serCrcErr`,
  `serBadFrame`, `serInits`, `serInitsHeld`, `serLinkUps`, `serResets`,
  `serTxFull`); serial-specific `ROUTER UNREACHABLE` lines that name the
  `/dev/serial/by-id` path to listen on. `"linkLogs": 1` copies log lines onto
  the link between frames for `tools/zenoh_serial_link.py bridge` to print -
  bring-up only, since a real router drops the link on them.
- `tools/zenoh_serial_link.py` - `selftest`, `bridge` (serial to a TCP
  router, printing the device's log lines), `relay` (a TCP client over serial
  to a serial router), `tap`.
- `linux_unit_tests/zenoh_serial_tests.cpp` (`make zenoh-serial-test`).
- `docker/distros/demo.sh` - `RAFTROS_SERIAL=<port>` gives the container the
  host's `/dev` (ttyACM/ttyUSB only) and adds the serial listen endpoint.

### Found on the way (not serial)

- **Connector power belongs to RaftI2C (fixed 2026-10-11).** The example
  powered its STEMMA QT / QWIIC connector from `main.cpp` (a GPIO toggled
  before the buses started). RaftI2C's `BusPowerController` - the bus `pwr`
  block the Axiom uses - does this properly (off at boot, on, power-cycle on
  demand) but only drove PCA9535 expander pins: it called `BusIOExpanders`
  directly, which refuses a pin no expander owns. It now hands such pins to
  `RaftBusSystem::virtualPinsSet`, which offers them to every bus and then
  drives them as plain GPIOs, so a GPIO-switched connector is configuration.
  Verified on the FeatherS3 with GPIO 39: off at 37 ms and 466 ms, on at
  968 ms, output register reads 1. The change is in `raftdevlibs/RaftI2C`
  and needs to reach RaftI2C `main`.

- **Double scaling in auto-publish (fixed in RaftCore's `DecodeGenerator.py`).**
  The standard generated decoder applies an attribute's `d` and `a` while
  decoding, but the field table carried them too, and RaftROS,
  `DeviceManager::getBusDeviceNamedValue` and the Axiom DataLogger apply the
  table's values - so every plain record with a `d` (SHT40, AHT20, TMP117,
  LPS25, ...) was scaled twice. The table now holds what is still to apply:
  1/0 for a standard decode, `d`/`a` for a custom decode (whose code stores
  raw values, as raftjs already assumed). The example's MPU6050 is now a
  standard record; APDS9960 stays custom as an example of one.
- **A custom decode without `next;` reported no records (fixed):** the
  generator appends `next;` when the program has none. The SCD40 record's
  custom code read single bytes of its 9-byte response and scaled values the
  table scaled again; it is now a standard record (words at 0, 3 and 6), in
  RaftCore's and the Axiom's device tables.
- `ExampleDiscoverable` cannot be built by `raft build` in Docker: its
  `EXTRA_COMPONENT_DIRS` points four levels up, outside the mounted folder,
  which resolves to `/` in the container. Building with the whole repository
  mounted works (see the example README).

## Why this is the right layer

Zenoh already defines a serial link. The router side of `rmw_zenoh` is built
with it, the wire format is a small COBS frame around an ordinary Zenoh
batch, and zenoh-pico implements it for ESP-IDF. So the ROS 2 side needs no
new process and the firmware change is confined to the link under
`ZenohTCPSession`: swap the socket for a UART and the 2-byte length framing
for the COBS frame. INIT/OPEN, leases, declarations, liveliness, services and
parameters are untouched.

## What is established

### Router side

- `rmw_zenoh`'s `zenoh_cpp_vendor/CMakeLists.txt` sets
  `ZENOHC_CARGO_FLAGS "--features=shared-memory zenoh/transport_serial"` on
  both the `jazzy` and `rolling` branches (checked at HEAD on 2026-10-09). So
  `rmw_zenohd` has the serial link compiled in. **Whether the apt package
  `ros-jazzy-rmw-zenoh-cpp` 0.2.10 on the ROS box was built from a commit
  that already had this is Step 0.**
- Stock standalone `zenohd` release binaries do **not** include serial
  (1.10.1 verified: `strings` shows only tcp/udp/tls/quic/ws/unixsock links and
  a serial listen endpoint fails with "Unicast not supported for serial
  protocol"). A standalone router must be built with
  `cargo build --release --bin zenohd --features transport_serial`.
- Locator syntax (note the double slash before an absolute path):
  `serial//dev/ttyUSB0#baudrate=921600`. Config keys from
  `zenoh-link-serial`: `baudrate`, `exclusive` (default `true`, takes the
  TIOCEXCL lock so nothing else can open the port), `tout` (connect retry
  interval, default 50000 us) and `release_on_close`.
- The router config is JSON5; `rmw_zenohd` reads its default from
  `rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5` and
  `ZENOH_ROUTER_CONFIG_URI=<file>` overrides it. Keep the TCP listen endpoint
  as well, because the `ros2` CLI on the box reaches the router over
  `tcp/localhost:7447`:

  ```json5
  listen: { endpoints: ["tcp/[::]:7447", "serial//dev/ttyUSB0#baudrate=921600"] }
  ```

### Wire format (z-serial, shared by zenoh and zenoh-pico)

One Zenoh batch per frame. Before COBS:

| field | size | notes |
| --- | --- | --- |
| header | 1 | flags: I=0x01 (init), A=0x02 (ack), R=0x04 (reset); 0 for data |
| length | 2 | payload length, little-endian |
| payload | 0..1500 | one Zenoh transport batch; MTU 1500 |
| CRC-32 | 4 | little-endian, over the **payload only** |

The whole frame is COBS-encoded and terminated with a single `0x00`
delimiter. Encoded size is at most 1517 bytes (`COBS_BUF_SIZE`).

The CRC is **not** the zlib/Ethernet CRC-32. It is a reflected table
algorithm (shift right, init `0xFFFFFFFF`, final inversion) that uses the
*unreflected* polynomial `0x04C11DB7` as its table constant. Verified against
the two test vectors in `z-serial`'s own tests:

| payload | CRC (LE bytes on the wire) | full frame, COBS-encoded |
| --- | --- | --- |
| `00 11 00` | `73 EC 75 F9` | `01 02 03 01 02 11 05 73 EC 75 F9 00` |
| `11 22 00 33` | `8D 03 6D FB` | `01 02 04 03 11 22 06 33 8D 03 6D FB 00` |

Reference implementation (Python, matches both vectors):

```python
POLY = 0x04C11DB7
TABLE = []
for n in range(256):
    r = n
    for _ in range(8):
        r = (POLY ^ (r >> 1)) if r & 1 else (r >> 1)
    TABLE.append(r)

def crc32_zserial(data: bytes) -> int:
    acc = 0xFFFFFFFF
    for b in data:
        acc = (acc >> 8) ^ TABLE[(acc & 0xFF) ^ b]
    return (~acc) & 0xFFFFFFFF
```

### Link-level handshake (below the Zenoh session)

Both implementations do this before any Zenoh INIT:

1. The connecting side (our device) sends an empty frame with header `I`.
2. The listening side (router) clears its port and, on seeing `I`, replies
   with an empty frame with header `I|A` and marks the link initialized.
3. The connecting side retries `I` every `tout` until it sees `I|A`. An `R`
   reply means "wait and retry".
4. If an initialized listener sees another `I` (the device rebooted), it
   replies `R`, drops back to uninitialized and re-runs step 2 on the next
   `I`. So a device reset is recovered by the device simply re-sending `I`.

Data frames then carry header `0`.

### Link properties and what they mean for Raft's session

- **Not streamed** (`is_streamed() == false`, zenoh-pico
  `Z_LINK_CAP_FLOW_DATAGRAM`): the router sends batches **without** the
  2-byte length prefix used on TCP. `ZenohStreamFramer` must be bypassed and
  each received frame handed straight to `processBatch`. On transmit, the two
  length bytes that `beginOutput` reserves must be skipped and each flush
  must be exactly one frame.
- **MTU 1500**: the router answers INIT with `min(our batch, 1500)`. Our
  INIT advertises `BATCH_CAPACITY` (4096) and the session already clamps
  `_negotiatedBatch` to the router's reply and refuses to build a batch above
  it, so no change is needed there - but it is worth a log line confirming
  that 1500 was negotiated.
- **Not reliable** (`IS_RELIABLE = false`): a CRC-failed frame is dropped by
  the receiver, so sequence-number gaps are possible. The router's receive
  path drops a frame whose sequence number does not follow and carries on
  (`zenoh-transport` `rx.rs`, `roll`/`precedes`). Our session currently
  fails with `SequenceMismatch` on any gap
  ([ZenohTCPSession.h:424](../components/RaftROS/Zenoh/ZenohTCPSession.h#L424)).
  On serial that would tear the session down on every corrupted frame, so
  the serial build needs the router's drop-and-resync behaviour, with a
  counter. Confirm the exact `roll` semantics (window, what counts as a
  follow-on) before copying them.
- Keep-alive and lease (`LOCAL_LEASE_MS` 4000) are unchanged. There is no
  scouting or multicast on serial; the session points at the router anyway.

### zenoh-pico support (for the proving step)

- `Z_FEATURE_LINK_SERIAL=1` builds the serial link; ESP-IDF is a supported
  platform (`src/link/transport/serial/uart_espidf.c`). `Z_FEATURE_LINK_SERIAL_USB`
  exists only for the RPi Pico, and there is no BLE link at all (the
  Bluetooth link is Classic SPP on Arduino, and the S3 has no Classic radio).
- Locator: `serial/UART_1#baudrate=921600` or `serial/<tx>.<rx>#baudrate=...`.
  The ESP-IDF backend only accepts the fixed pin pairs `1.3`, `10.9` and
  `17.16` (classic-ESP32 numbering) and maps them to UART 0/1/2, and
  `baudrate` is the only config key honoured. The Feather's UART pins are not
  in that table, so a small local patch is needed (see Step 2).
- The driver is installed with a 2 kB RX ring buffer, no TX ring buffer, and
  reads block for up to 1 s - fine for zenoh-pico's own receive task, not for
  the Raft main loop. Field reports from 2024 (ESP32 UART to a Linux
  `zenohd`): the 64 kB default buffers had to be shrunk and fragmentation was
  disabled. RaftROS already builds zenoh-pico with its own buffer sizes
  (`RAFTROS_ZENOH_PICO_TUNED`) and `Z_FEATURE_FRAGMENTATION 1`; keep
  fragmentation on and watch for it.

## Test bench on the Linux box

- **Board:** Adafruit ESP32-S3 TFT Feather with the VL6180 range sensor on
  STEMMA QT (I2C SDA 42 / SCL 41, GPIO 21 powers the connector; since
  2026-10-11 the bus's `pwr` block in SysTypes switches it, through RaftI2C's
  bus power controller). Its UART header pins are TX = GPIO1,
  RX = GPIO2 (confirm against the Adafruit pinout page before wiring).
- **Cabling:** two USB cables. The Feather's native USB stays the console
  and flashing port (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, as now). A
  3.3 V USB-UART adapter (FT232/CP2102N/CH340) goes on the header TX/RX,
  crossed, plus GND. This keeps `raft monitor` and the Zenoh link on separate
  ports, and the adapter has no auto-reset wiring so the router opening the
  port cannot reset the board.
- **Baud:** start at 921600 8N1 (about 92 kB/s, so a 1517-byte frame takes
  about 16 ms on the wire and about 920 bytes arrive per 10 ms loop pass).
  Raise to 2-3 Mbaud once working if the adapter supports it (CP2102N and
  FT232 do; CH340 tops out around 2 M). For reference the ESP32-S3 UART goes
  to 5 Mbaud.
- **Host port:** `/dev/ttyUSB0` for FTDI/CP210x/CH340 or `/dev/ttyACM0` for
  CDC adapters; the user must be in `dialout`. Add a udev symlink by serial
  number so the locator does not change between plug-ins. Nothing else may
  hold the port while the router has it (`exclusive=true`).
- **ROS box state:** Jazzy at `/opt/ros/jazzy`, `ros-jazzy-rmw-zenoh-cpp`
  0.2.10, zenoh-python 1.8.0 in `/home/rob/zenoh-venv`, tshark without sudo.
  Every `ros2` command needs `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.

## Steps

### Step 0: confirm the installed router speaks serial

1. `strings /opt/ros/jazzy/opt/zenoh_cpp_vendor/lib/libzenohc.so | grep -c 'serial//'`
   (or grep for `zenoh-link-serial`). Zero means the package predates the
   feature flag.
2. Functional check without hardware: `socat -d -d pty,raw,echo=0 pty,raw,echo=0`
   gives a pty pair; point a router config at one end
   (`serial//dev/pts/N#baudrate=115200`) and start
   `ros2 run rmw_zenoh_cpp rmw_zenohd`. It must start without "Unicast not
   supported for serial protocol".
3. If the package lacks serial: build `rmw_zenoh` from source in a workspace
   on the box (it vendors zenoh-c; the flag is in the CMake), or build a
   standalone `zenohd` with `transport_serial` and peer it to `rmw_zenohd`
   over TCP (`connect.endpoints` in its config; routers never find each other
   automatically). Or use the bridge tool from Step 1 and defer this.

### Step 1: host-side framer tool and pty loopback

Write `tools/zenoh_serial_link.py` (pyserial only): COBS encode/decode, the
CRC above, frame (de)serialization and the I/A/R handshake, with unit tests
on the two published vectors. Give it two modes:

- `--listen <pty>` + `--tcp host:port`: accept the device's link handshake on
  a serial port, then relay batches to a TCP router, adding the 2-byte length
  prefix on the way in and stripping it on the way out. This lets the
  firmware be tested against `tools/zenoh_router_stub.py` or a TCP-only
  router before Step 0 is resolved, and gives a place to log every frame.
- `--connect <pty>`: act as a device - do the handshake and then run the
  stub's INIT/OPEN against a real serial-enabled router across the socat pty
  pair. This proves the router side and the framer with no board attached.

Also useful: `--tap` that just decodes and prints frames from a port, as the
serial equivalent of tshark.

### Step 2: zenoh-pico proof on the board

Cheapest end-to-end proof that the hardware, baud and router config are right.

1. In `CMakeLists.txt` add `set(Z_FEATURE_LINK_SERIAL 1)` next to
   `Z_FEATURE_LINK_TCP` for the zenoh-pico backend, and `driver` to the
   component's `REQUIRES` if the UART driver is not already linked.
2. Patch `third_party/zenoh-pico/src/link/transport/serial/uart_espidf.c` so
   `txpin.rxpin` accepts the Feather's `1.2` (simplest: any pair maps to
   `UART_NUM_1`). Save it as `devdocs/patches/zenoh-pico-espidf-uart-pins.patch`
   like the RaftCore NVS patch, since `third_party/` is refetched.
3. In `RaftROSZenohPico.cpp`, build the locator from a new SysTypes setting
   instead of `routerHost` when present, e.g.
   `"routerSerial": "1.2#baudrate=921600"` giving `serial/1.2#baudrate=921600`.
4. Build with `CONFIG_RAFTROS_BACKEND_ZENOH_PICO=y`, flash, and expect
   `ros2 node list` to show the node, `/chatter` to echo, and the VL6180
   range topic to publish. Record `/api/rosstat` loop timings and heap.

If this works the host side is proven and everything after is firmware.

### Step 3: serial link in the native Zenoh backend

Keep `ZenohTCPSession` as the session; add a link beneath it.

- **3a Framer** `ZenohSerialFramer.h`: COBS + CRC + header, host-testable
  like `ZenohStreamFramer`, fed byte-by-byte from the UART ring buffer,
  bounded at 1517 bytes, resynchronising on the `0x00` delimiter. Counters:
  frames in/out, CRC failures, oversize, resyncs. Unit tests on the vectors
  above plus a fuzz of random corruption (it must never crash or stall, only
  count).
- **3b Link** `ZenohSerialLink` (ESP-IDF `driver/uart`): `uart_driver_install`
  with RX and TX ring buffers of at least 2 frames each (4 kB is plenty: the
  hardware FIFO is 128 bytes, so the driver's interrupt-fed ring is what
  covers a 10 ms loop pass). Reads with `uart_read_bytes(..., 0 ticks)` so
  they never block. `uart_write_bytes` blocks when the TX ring is full, so
  gate each frame on `uart_get_tx_buffer_free_size` and leave the remainder
  for the next pass, as `flushToRouter` already does for a partial `send`.
  The link runs the I/A/R handshake with retry and a reset-on-R path.
- **3c Session datagram mode**: a `receiveBatch(bytes, len)` entry that
  skips the stream framer, output that skips the reserved length bytes and
  flushes one batch per frame, and the drop-and-resync sequence policy with
  a counter (see Link properties). The negotiated batch of 1500 falls out of
  the existing INIT handling.
- **3d Link selection in `RaftROSZenoh`**: `startConnect`, `receiveFromRouter`
  and `flushToRouter` call through a small link interface (TCP socket or
  serial). On serial, "connected" means the link handshake completed; the
  session's own INIT/OPEN then runs exactly as on TCP. Lease expiry or a
  router restart must redo the link handshake, not just the session.
- **3e Configuration**: a `link` setting (`"tcp"` default, `"serial"`) with
  `serialUart`, `serialTxPin`, `serialRxPin`, `serialBaud`, layered like
  `routerHost` (Kconfig default < SysTypes < posted settings, settable via
  `/api/ros/set`). `/api/rosstat` and `getStatusJSON` report the link type,
  baud, negotiated batch and the framer counters. The `ROUTER UNREACHABLE`
  diagnostics gain serial causes: no `I|A` within N seconds, repeated `R`,
  CRC failure rate.

Main-loop budget to hold: 10 ms average, 50 ms worst case per pass
(Raft SysMod contract). Serial should be cheaper than TCP: no lwIP, no WiFi
task contention on CPU0.

### Step 4: verification against the ROS box

Same checklist as the Zenoh milestone, over serial:

- `ros2 node list` / `node info`, `topic info -v` for the range topic.
- `/chatter` echo and publish to `/chatter_in`, both directions.
- VL6180 range topic rate and latency; hot-plug (unplug the sensor, topic
  withdrawn, re-plug, re-announced).
- Services (`/<node>/devices`, `/<node>/chatter_enable`) and
  parameters (`ros2 param list/get/set/dump`).
- Recovery: board reset, router restart, cable pulled and replaced. Each
  must come back with one session and the counters explaining what happened.
- Corruption: a deliberately bad baud mismatch for a moment, or the `--tap`
  tool injecting a corrupted frame across the pty pair. Expect dropped
  frames and resyncs, not session loss.
- 12-hour soak with a service or parameter call every few seconds, free heap
  flat, loop timing within budget, as for the TCP milestone.
- Measure: image size, heap, loop max, frame counters, at 921600 and at the
  highest stable baud. Record in `RaftROS-zenoh-milestone-results.md`.

### Step 5: follow-ons

- **USB Serial/JTAG as the link (one cable).** The S3's USB Serial/JTAG
  peripheral appears on Linux as `/dev/ttyACM0` with no driver and ESP-IDF's
  `usb_serial_jtag` driver has the same ring-buffer, non-blocking read shape
  as the UART driver, so it is a second `ZenohSerialLink` backend. The
  console then has to move to UART0 (`CONFIG_ESP_CONSOLE_UART_DEFAULT`),
  which changes how `raft monitor` is used; and the host must not toggle
  DTR/RTS in the esptool pattern when opening the port. Throughput is lower
  than a fast UART.
- **TinyUSB CDC** (`esp_tinyusb`) for higher USB throughput, or
  **USB NCM** to put the board on the host's IP network with the TCP backend
  unchanged. Both reported slow or flaky with `esp_tinyusb` as of 2024; park
  unless the UART path is throughput-limited.
- **BLE.** No Zenoh BLE link exists. The nearest route is Nordic UART Service
  on the device (NimBLE is already in the Axiom build), `ble-serial` on the
  host to expose it as a pty, and the router listening on that pty via the
  same serial link. The framer and link from Step 3 are reused unchanged;
  only the byte pipe differs. S3 BLE throughput is around 0.7-1.35 Mbit/s raw
  and much less in practice, so this is for low-rate nodes. 6LoWPAN over
  BLE (IPSP) exists as a community NimBLE component plus Linux
  `bluetooth_6lowpan`, but it is experimental on the ESP side.

## Risks and open questions

- The apt `rmw_zenoh` may predate `transport_serial`; Step 0 decides whether
  a source build or the Step 1 bridge is needed first.
- `exclusive=true` on the router means the port cannot be shared with a
  tap; use the pty-pair or the `--tap` mode on a second adapter listening on
  the TX line only.
- Burst at connect: liveliness tokens, declarations and parameter
  descriptions go out in a few kB; at 921600 that is tens of ms of wire
  time, spread over passes by the TX ring gating. Fine, but confirm no
  single pass blocks.
- Fragmentation: the session already limits messages to the negotiated
  batch; large replies (parameter dump, device list) must fit 1500 or be
  fragmented. Check what the TCP build does for replies above the batch
  today and whether 1500 changes anything.
- Sequence policy: copy the router's exact `roll` semantics so the two
  sides agree on what a recoverable gap is.
- The zenoh-pico UART backend's fixed pin table is a known rough edge; an
  upstream PR making pins free-form would remove the local patch.

## Sources

- zenoh-pico README (platform support table) and `CMakeLists.txt` feature
  flags: https://github.com/eclipse-zenoh/zenoh-pico
- zenoh-pico ESP-IDF UART backend:
  https://github.com/eclipse-zenoh/zenoh-pico/blob/main/src/link/transport/serial/uart_espidf.c
- zenoh-pico serial link capabilities (`src/link/unicast/serial.c`) and
  framing (`src/link/transport/upper/serial_protocol.c`,
  `include/zenoh-pico/link/transport/serial_protocol.h`)
- z-serial (framing, CRC, handshake, test vectors):
  https://github.com/ZettaScaleLabs/z-serial/blob/master/src/lib.rs
- zenoh-link-serial (locator keys, MTU, reliability, streamed flag):
  https://github.com/eclipse-zenoh/zenoh/tree/main/io/zenoh-links/zenoh-link-serial
- zenoh transport receive path (sequence gap handling):
  https://github.com/eclipse-zenoh/zenoh/blob/main/io/zenoh-transport/src/unicast/universal/rx.rs
- rmw_zenoh vendor flags (jazzy and rolling):
  https://github.com/ros2/rmw_zenoh/blob/jazzy/zenoh_cpp_vendor/CMakeLists.txt
- Zenoh serial blog (MbedOS Nucleo to zenohd over USB serial, 2022):
  https://zenoh.io/blog/2022-08-12-zenoh-serial/
- zenoh-pico over serial field notes (ESP32 UART, 2024):
  https://github.com/eclipse-zenoh/roadmap/discussions/120
- micro-ROS with an rmw_zenoh host (why not micro-ROS):
  https://github.com/micro-ROS/micro-ROS-Agent/issues/243 and
  https://discourse.openrobotics.org/t/integrating-ros-2-with-microcontrollers-when-using-zenoh/43463
- `eppp_link` (PPP alternative, not chosen):
  https://github.com/espressif/esp-protocols/blob/master/components/eppp_link/README.md
- SLIP removal in ESP-IDF 5.0:
  https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/migration-guides/release-5.x/5.0/removed-components.html
- `ble-serial` (NUS to pty): https://github.com/Jakeler/ble-serial
- Adafruit ESP32-S3 TFT Feather pinout:
  https://learn.adafruit.com/adafruit-esp32-s3-tft-feather/pinouts
