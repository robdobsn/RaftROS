# RaftROS Zenoh over Serial: Investigation Plan

**Date:** 2026-10-09
**Status:** Investigation. Nothing implemented yet; the facts in
[What is established](#what-is-established) were verified against the sources
listed at the end on this date.
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
  STEMMA QT (I2C SDA 42 / SCL 41, GPIO 21 powers the connector; the example's
  `main.cpp` already does this). Its UART header pins are TX = GPIO1,
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
   `ros2 node list` to show `raft_esp32`, `/chatter` to echo, and the VL6180
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
- Services (`/raft_esp32/devices`, `/raft_esp32/chatter_enable`) and
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
