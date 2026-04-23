# RaftROS Auto-Publishing Design & Implementation Plan

**Status:** DRAFT — open questions at end, awaiting user input  
**Author:** RaftROS maintainers  
**Last Updated:** 2026-04-22  
**Scope:** Phase 4 of the RaftROS roadmap: automatic ROS 2 topic publishing for every
I²C device attached to a Raft bus, with dynamic online/offline handling and
per-`clas` message-type selection.

> Pre-requisite reading:
> - [RaftROS-overview.md](RaftROS-overview.md) — current phase roadmap
> - [RaftROS-development-status.md](RaftROS-development-status.md) — Phase 3 (topic subscribing) complete
> - [RaftROS-next-stages-implementation-plan.md](RaftROS-next-stages-implementation-plan.md) — slice plan
> - `/memories/repo/raftros-rtps-data-flags.md` — RTPS DATA flags lesson (Q-flag, dispose)

---

## 1. Goal & Success Criteria

### 1.1 Goal
When RaftROS is loaded on an ESP32-S3 and one or more I²C devices are attached,
each online device SHALL automatically appear in the ROS 2 graph as one or
more publishers with topic names, message types, QoS, and serialized payloads
that a standard ROS 2 subscriber (e.g. `ros2 topic echo`) can consume without
any host-side codegen. Devices that come online or go offline at runtime SHALL
be reflected as matching SEDP publication ADD / DISPOSE announcements.

### 1.2 Success criteria
1. Plugging in a supported device (e.g. `LSM6DS`, `VL53L4CD`, `AHT20`) causes a
   new topic to appear in `ros2 topic list` on the host within ≤ 2 s.
2. `ros2 topic echo <topic>` prints decoded, human-readable values at the
   device poll rate with correct units (already scaled by `divisor`/`addend`).
3. Unplugging the device (or bus error → offline) causes the topic to disappear
   from `ros2 topic list` within ≤ 10 s (bounded by lease duration).
4. Re-plugging the same device reuses the same topic name and resumes
   publishing.
5. A fallback path exists for devices whose `clas` has no standard ROS 2 mapping
   — they MUST still be observable (even if via a generic `std_msgs/String`
   JSON carrier).
6. ESP32 heap impact ≤ ~8 kB per active device-writer; total capped by a
   compile-time constant (`RAFTROS_MAX_DYNAMIC_WRITERS`).
7. Phase 3 subscription functionality continues to pass all 388 unit tests.

### 1.3 Non-goals (this phase)
- Publishing actuator commands (servo targets, LED pixel arrays). Only sensor
  publishing is in scope; actuator subscriptions are Phase 5 (services/params).
- ROS 2 services, parameters, actions, or lifecycle nodes.
- Host-side custom `.msg` generation. Every publication MUST use either a
  built-in ROS 2 type already known to FastDDS/rcl, or the JSON-in-`String`
  fallback.
- Per-attribute topic decomposition when a single `sensor_msgs/*` covers the
  whole device (e.g. `LSM6DS` → one `/imu` topic, not six `/ax`, `/ay`, …).

---

## 2. ROS 2 Publishing Primer (what we actually have to emit)

A ROS 2 publication is, at the wire level, an RTPS 2.2 DataWriter endpoint
inside a Participant. For RaftROS this has four distinct artefacts, in order
of appearance on the wire:

| # | Artefact | Who produces it today (Phase 3 code) | Who has to produce it for Phase 4 |
|---|----------|--------------------------------------|------------------------------------|
| 1 | **SPDP Participant announce** — UDP multicast to `239.255.0.1:7400` describing our Participant GUID, locators, vendor ID. | `RaftROS::setup()` via `DiscoveryRunner`. | Already done; no change. |
| 2 | **SEDP Publication announce (DATA(w))** — reliable unicast/multicast on `builtin_publications_reader`. Carries: writer GUID, topic name, type name, type hash (optional), QoS (reliability, history, durability, lease). | `SEDPHandler` currently publishes one static entry for `/chatter`. | Must be dynamic: **emit one publication DATA per device-writer when device comes online**, and a dispose DATA (flags&0x03==0x03) when it goes offline. |
| 3 | **User DATA submessages** — each sample sent on the per-topic writer's user-data endpoint, CDR-serialized payload prefixed with representation identifier + options (4 bytes). | `RaftROS::loop()` for `/chatter`. | Must serialize per-device structs into the correct CDR layout for the chosen message type. |
| 4 | **HEARTBEAT / ACKNACK churn** — reliability protocol. | `WriterHeartbeatRunner` + `ReliableWriter`. | Must scale to N writers; one HB per writer per period, ACKNACKs multiplexed. |

### 2.1 What a subscriber needs to accept us
FastDDS (Humble) matches a DataReader to our DataWriter when **all** of the
following hold:
- Topic name matches exactly (including any ROS 2 prefix — see §5.1).
- Type name matches exactly (e.g. `sensor_msgs::msg::dds_::Imu_`).
- QoS is compatible: `Reliability` (BEST_EFFORT ⊆ RELIABLE on subscriber),
  `Durability` (VOLATILE ⊆ TRANSIENT_LOCAL), `History` kind, `Liveliness`.
- Type hash is either absent (lenient) or matches.

### 2.2 CDR serialization (the only payload format we need)
Every standard message is serialized as **CDR** (OMG CDR) with a 4-byte
encapsulation header: `{0x00, 0x01, 0x00, 0x00}` for little-endian PL_CDR. After
that header, fields are emitted in struct order with natural alignment relative
to the CDR cursor. For e.g. `sensor_msgs/msg/Imu`:

```
header: std_msgs/Header  ← time {sec:int32, nanosec:uint32} + frame_id:string
orientation: geometry_msgs/Quaternion        (4× float64)
orientation_covariance: float64[9]
angular_velocity: geometry_msgs/Vector3      (3× float64)
angular_velocity_covariance: float64[9]
linear_acceleration: geometry_msgs/Vector3   (3× float64)
linear_acceleration_covariance: float64[9]
```

The serializer code is trivially hand-written for each of the small number of
types we actually use (§5). No external codegen is required on the ESP32.

### 2.3 ROS 2 topic/type conventions
ROS 2 on DDS uses specific naming:
- Topic on the wire: `rt/<topic>` (the `rt/` prefix marks a user topic, as
  opposed to `rq/` request, `rr/` response, `rs/` service reply).
- Type on the wire: `<package>::msg::dds_::<Type>_` (note trailing underscore
  and `dds_` sub-namespace).
- Example: `/imu` becomes `rt/imu`, type `sensor_msgs::msg::dds_::Imu_`.

Our SEDP announcer MUST produce these wire-level strings, not the Pythonic
`sensor_msgs/msg/Imu` form.

---

## 3. Dynamic Nodes & Topics in ROS 2 — how do they actually "come and go"?

ROS 2 nodes and topics are not first-class persistent entities; they are a
projection of the live DDS endpoint set. A node is "online" exactly for as
long as its Participant sends SPDP announces within its
`participantLeaseDuration` (default 12 s); a topic "exists" for exactly as
long as there is at least one DataReader or DataWriter endpoint announced via
SEDP and not yet disposed.

### 3.1 Add flow (device attaches)
1. RaftROS detects device online via `DeviceManager::registerForDeviceStatusChange`
   callback (`changeToOnline = true`).
2. RaftROS instantiates a new `DynamicWriter` (new writer GUID =
   `participantGuid:entityId`, `entityId` allocated from a free list).
3. RaftROS sends an SEDP publication DATA on `builtin_publications_writer`
   describing the new writer, topic, type, QoS, and PID_KEY_HASH derived from
   the writer GUID. Reliable delivery: retransmitted until matched readers ACK.
4. Any subscriber whose filter matches the topic+type receives the SEDP DATA,
   instantiates a matching reader, and will ACK user-data samples from that
   writer going forward.
5. RaftROS starts publishing CDR samples on its new user-data writer at the
   device's poll rate.

### 3.2 Remove flow (device detaches)
1. `registerForDeviceStatusChange` fires with `changeToOnline = false` (or no
   poll response for `DEVICE_OFFLINE_TIMEOUT_MS`).
2. RaftROS emits a **dispose** SEDP publication DATA: RTPS DATA with
   `flags & 0x03 == 0x03` (D=0, K=1, inlineQoS contains `PID_STATUS_INFO =
   0x0003` with disposed+unregistered bits). The payload contains only a
   PID_KEY_HASH and PID_SENTINEL — no serialized publication data.  
   This is exactly the pattern we already decode silently in `onData` via the
   inline-QoS Q-flag handling fixed in Phase 3 (see
   [memory: raftros-rtps-data-flags](/memories/repo/raftros-rtps-data-flags.md)).
3. Matched subscribers remove their reader; `ros2 topic list` no longer shows
   the topic (if no other publisher holds it).
4. RaftROS releases the `entityId` back to the free pool and destroys the
   `ReliableWriter` instance (releasing its history cache heap).

### 3.3 Lease-duration fallback
If RaftROS crashes or the ESP32 loses power without sending disposes, the
subscriber times out on the participantLeaseDuration and removes **all** of
our endpoints at once — functionally correct, just slower.

---

## 4. Data-Access Mechanism — Evaluation & Recommendation

Four candidates were considered. Evaluation criteria: latency,
thread/ordering guarantees against the bus polling thread, heap cost on
ESP32-S3, coupling to existing modules, and fit with the online/offline model
described in §3.

### 4.1 Option 1 — Register RaftROS as a CommsChannel and reuse `StatePublisher`
- **Mechanism:** `StatePublisher` already serves topic publications for BLE,
  WebSocket, etc., via `registerDataSource(topic, msgGenCB, stateDetectCB)`
  and routes through `CommsCoreIF` channels. RaftROS would register as a
  channel, `StatePublisher` would iterate `_pubSources` and call us back with
  string payloads to transmit.
- **Pros:** Reuses mature back-pressure / subscription filtering. Topic list
  is already visible to other systems.
- **Cons:** (a) `StatePublisher` is **string-oriented** — we need CDR binary
  buffers, would have to round-trip via string-encoded payloads or bypass the
  core string path. (b) Publication rate is poll-based inside
  `StatePublisher::loop()`, not event-driven off bus callbacks — adds jitter
  that is visible on `/imu` etc. (c) The channel model expects a
  pre-registered topic set; dynamic add/remove at sub-second granularity is
  not the channel abstraction's strong point. (d) Couples RaftROS to the
  presence of `StatePublisher`; users who only want ROS 2 publishing would
  have to include it anyway.
- **Verdict:** rejected for the hot path. **May still be useful as a
  supplementary surface** for diagnostics / string-formatted status topics
  (e.g. `/raft/diag` via the existing `StatePublisher` path), but not for
  sensor data.

### 4.2 Option 2 — Directly register per-device callbacks (DataLogger-style) ✅ RECOMMENDED
- **Mechanism:** RaftROS subscribes to
  `DeviceManager::registerForDeviceStatusChange(statusCB)`. On every
  online transition it calls
  `DeviceManager::registerForDeviceData(deviceID, dataCB, rateMs, pCtx)`
  (see [DataLogger.cpp](../../Robotical/RoboticalAxiom1/components/DataLogger/DataLogger.cpp)
  line 785 for the exact idiom). On offline it calls the same function with
  `unregister = true`. Data callback receives the raw poll buffer; we decode
  via the generated `pollResultDecodeFn` using cached `DeviceTypeRecord`.
- **Pros:** (a) **Event-driven** — we publish at the exact rate the device is
  polled, no extra timer, minimal jitter. (b) Raft already owns the online /
  offline lifecycle hook — fits §3 flow exactly. (c) Zero dependency on any
  other SysMod. (d) Already proven in DataLogger; well-understood thread
  model (callbacks run on bus polling task → keep them short). (e) Clean
  teardown for the dispose flow in §3.2. (f) Allows per-device callback
  context to cache `AttrFieldDesc*`, writer GUID, decode state, and a
  pre-allocated CDR scratch buffer — no per-callback allocation.
- **Cons:** (a) Callback runs on bus task; must not block on DDS writer
  (solved by queuing CDR blob onto writer's history cache, which is O(1) +
  one memcpy). (b) We must track online state ourselves (trivial — one map
  keyed by `RaftDeviceID`). (c) Bus-task CPU cost of CDR serialization is
  additive to poll cost (benchmarked < 50 µs for Imu_).
- **Verdict:** ✅ **Recommended.** Lowest latency, cleanest mapping to §3.

### 4.3 Option 3 — Intercept at `PollDataAggregatorIF` level
- **Mechanism:** Hook a custom aggregator into every `DeviceStatus` via
  `setAndOwnPollDataAggregator`; override `put()` to publish, then delegate
  to the default ring-buffer.
- **Pros:** One-stop interception point inside RaftCore; sees every raw poll.
- **Cons:** (a) Requires modifying `DeviceStatus` construction path or adding
  a factory hook — more invasive to RaftCore than the public registration
  API. (b) Aggregator sees raw pre-decode bytes plus the 2-byte timestamp
  prefix; we'd have to reproduce the decode dance ourselves instead of
  leveraging `pollResultDecodeFn`. (c) Ownership semantics (`setAndOwn`)
  conflict with any other consumer (e.g. UI polling REST) that expects the
  default aggregator. (d) Does not solve online/offline; we'd still need the
  status-change callback.
- **Verdict:** rejected — strictly inferior to Option 2.

### 4.4 Option 4 — Poll the `getLatestDecodedPollResponse` API from RaftROS's own loop
- **Mechanism:** Timer-driven loop in `RaftROS` that enumerates all bus
  devices and calls `getLatestDecodedPollResponse` per device.
- **Pros:** Simple; no callback threading concerns.
- **Cons:** (a) Polling the pollers — wasteful, introduces a second rate
  mismatch that can alias. (b) No notion of "new sample since last publish" —
  we'd publish duplicates or need extra bookkeeping. (c) Same online/offline
  problem as Option 3.
- **Verdict:** rejected.

### 4.5 Recommendation summary
**Option 2** is the recommended mechanism, with **Option 1** retained only as
a future hook for string-formatted diagnostic topics. All Phase 4 slices
below assume Option 2.

---

## 5. Device → ROS 2 Mapping Table

The `clas` tag in each device's `devInfoJson` (JSON array of class codes) is
the primary key for topic/type/serializer selection. Some devices carry
multiple classes (e.g. `LSM6DS` → `[ACC, GYRO]`); these are handled by a
**composite rule**: the combined class-set `{ACC, GYRO}` maps to
`sensor_msgs/Imu`, taking priority over the single-class rules.

### 5.1 Topic naming convention (resolved — see Q1 answer in §12)

```
/<namespace>/<device_alias>
```

- `<namespace>` — defaults to `raft_{{hostname}}`. Configured in SysTypes via
  a **pattern string** with `{{hostname}}` (and possibly `{{sysname}}`,
  `{{mac4}}`) placeholders. The pattern is expanded **once at setup time**
  and cached (no per-publish evaluation).
  - Example config: `"rosNamespace": "mytext_{{hostname}}"` → namespace
    becomes `/mytext_esp32-abcd`.
  - Placeholder resolution is centralised (a tiny helper `ExpandNamePattern`
    that also covers Q5 `frame_id` and Q1 topic-slug substitution).
- `<device_alias>` — defaults to `<class_slug>_<busN>_<addrHex>` (e.g.
  `imu_1_6a`, `temp_1_38`), overridable per-device via the DeviceManager
  friendly-name map (`DeviceManager::setDeviceName`) or a SysTypes
  `deviceAliases` map keyed by `busN:addrHex`.

### 5.2 Device-class → ROS 2 message table

Legend:
- **topic_slug** — default substring used in auto-generated topic name.
- **message** — fully-qualified ROS 2 message type (wire form
  `<pkg>::msg::dds_::<Type>_`).
- **field mapping** — how the `AttrFieldDesc` entries from
  `DeviceTypeRecords.json` are consumed. All values are post-`divisor`/
  `addend` (see `DeviceTypeRecord.h`), i.e. already in the `u` units.

| Device(s) | `clas` | topic_slug | message | field mapping (attr → field) | Notes |
|-----------|--------|------------|---------|------------------------------|-------|
| AHT20, MCP9808, LPS25 (partial) | `TEMP` | `temperature` | `sensor_msgs/msg/Temperature` | `temperature` → `temperature` (°C). `variance = 0`. | Header stamp filled per §6. |
| AHT20 (humidity channel) | `RH`   | `humidity`    | `sensor_msgs/msg/RelativeHumidity` | `humidity` → `relative_humidity` (0–1, so ÷100 from % unit). | Emit on same callback as TEMP: two publishes per tick. |
| LPS25 | `PRES` | `pressure` | `sensor_msgs/msg/FluidPressure` | `pressure` → `fluid_pressure` (Pa, so ×100 from hPa). | LPS25 also emits TEMP on its own topic. |
| VCNL4040, VEML7700, (RoboticalLightSensor) | `LGHT` | `illuminance` | `sensor_msgs/msg/Illuminance` | `als` → `illuminance` (lux). `white` additionally on `<topic>_white`. | `VCNL4040` also emits PROX (below). |
| VCNL4040 | `PROX` | `proximity` | `sensor_msgs/msg/Range` | `prox` → `range` (normalized 0–1.0, `radiation_type = INFRARED`, `min_range = 0`, `max_range = 1`). | Raw prox count has no physical unit; publish anyway with a noted range of `[0, 1]` derived from `65535` divisor. |
| VL6180, VL53L4CD | `DIST` | `range` | `sensor_msgs/msg/Range` | `dist` → `range` (metres, ÷1000 from mm). `radiation_type = INFRARED`. | Honour `valid`: skip publishing if `valid == 0`. |
| AS5600, MT6701 | `ANG` | `angle` | `std_msgs/msg/Float32` | `angle` → `data` (degrees). | Resolved: `Float32` chosen (Q3). |
| M5Encoder | `ROT` | `encoder` | `std_msgs/msg/Int32` | `rotation` → `data` (steps). Emit `press` on `<topic>_press` as `std_msgs/Bool`. | |
| ADXL313, MXC400xXC (standalone) | `ACC` | `accel` | `sensor_msgs/msg/Imu` | `ax,ay,az` → `linear_acceleration.{x,y,z}` (m/s², ×9.80665 from g). Gyro fields = 0, orientation = quaternion_identity with covariance `[-1,…]` to mark missing. | |
| LSM6DS | `{ACC, GYRO}` | `imu` | `sensor_msgs/msg/Imu` | `gx,gy,gz` → `angular_velocity.{x,y,z}` (rad/s, π/180 from °/s). `ax,ay,az` → `linear_acceleration.{x,y,z}` (×9.80665). orientation = identity w/ `[-1,…]` covariance. | Composite rule wins over single-class. |
| AHT20 (combined) | `{TEMP, RH}` | `climate` | `sensor_msgs/msg/Temperature` + `sensor_msgs/msg/RelativeHumidity` | Two topics per device: `<alias>/temperature`, `<alias>/humidity`. | No ROS 2 combined type; two writers. |
| LPS25 (combined) | `{PRES, TEMP}` | — | `FluidPressure` + `Temperature` | Two topics. | |
| MCP9808 | `LGHT` (typo in JSON? — should be `TEMP`) | `temperature` | `sensor_msgs/msg/Temperature` | `temperature` → `temperature` | **Open Q8** — MCP9808 is tagged `LGHT` but is a temperature sensor. JSON bug? |
| CAP1203 | `TCH` | `touch` | `std_msgs/msg/ByteMultiArray` | `A,B,C` → `data = [A,B,C]` (bool → 0/1). Skip `status`. | No ROS 2 standard touch type. |
| QwiicButton | `BTN` | `button` | `std_msgs/msg/Bool` | `press` → `data`. | |
| HX711 | `FRCE` | `force` | `geometry_msgs/msg/Wrench` | `force` → `force.z` (N); rest = 0. | Honour `valid` (skip if false). |
| MAX30101 | `HRM` | `ppg` | `std_msgs/msg/Float32MultiArray` | `[Red, IR]` → `data`. | No standard ROS 2 PPG type. |
| AdafruitSoilSensor | `SOIL` | `soil_moisture` | `std_msgs/msg/Float32` | `moisture` → `data`. | |
| BLEBTHome | `BTHM` | `bthome_<attr>` (one topic per attr) | see per-attr row | Split per attribute (resolved Q4): `temp`→`Temperature`, `battery`→`Float32` (% as 0–1), `motion`→`Bool`, `light`→`Illuminance`, `ID`→`Int32`, `MAC`→`String`. | Each attribute gets its own writer / SEDP announce. |
| AdafruitGamepad | `GAME` | `joy` | `sensor_msgs/msg/Joy` | `x,y` → `axes`; `SELECT,B,Y,A,X,START` → `buttons`. | |
| RoboticalServo, RoboticalWaterPump | `SRVO`,`PUMP` | — | — | **Excluded** (actuators, not sensors). Could publish status in Phase 5. | |
| QwiicLEDStick | `PIX` | — | — | **Excluded** (actuator, no poll attrs). | |
| RoboticalLightSensor | `LGHT` (4-attr) | `light` | `std_msgs/msg/Float32MultiArray` | `[left, center, right]` → `data`. | Arrayed light sensor. |
| **Unknown / new device** | _any_ | `raw` | `std_msgs/msg/String` | JSON of all decoded fields. | See §5.4. |

### 5.3 Composite-class precedence rules
Applied in order; first match wins:
1. `{ACC, GYRO} ⊆ clas` → `sensor_msgs/Imu`.
2. `{TEMP, RH} ⊆ clas` → two topics (`Temperature` + `RelativeHumidity`).
3. `{PRES, TEMP} ⊆ clas` → two topics (`FluidPressure` + `Temperature`).
4. Single-class table lookup (row by row in §5.2).
5. Fallback (§5.4).

### 5.4 Fallback path
When no rule matches (unknown `clas`, malformed `devInfoJson`, or a
missing-yet device type), publish a single `std_msgs/String` topic with:
```
{"type":"<deviceType>","addr":"<addrHex>","bus":<busNum>,"t":<ms>,
 "attrs":{"<name>":<value>, ...}}
```
This guarantees observability of any device, preserves Success Criterion 5,
and requires only the (already-needed) field-walking serializer from §6.

---

## 6. Attribute → CDR Field Serialization Rules

For each column in §5.2, the per-attribute serializer uses the existing
`AttrFieldDesc` walk (identical pattern to `DataLogger::formatFieldValue` but
emitting CDR instead of text):

```text
raw = read(type, pField)          # typed load from offset
scaled = raw / divisor + addend   # produces the unit stated in resp.a[i].u
cdr_value = unit_convert(scaled)  # e.g. deg→rad (×π/180), mm→m (÷1000), g→m/s² (×9.80665)
cdr_put(cdr_value)                # little-endian, natural alignment
```

Every conversion constant needed by §5.2 is fixed per mapping row and lives
in a compile-time table, e.g.:

```cpp
struct AttrToCdr {
    const char* attrName;   // "ax"
    float       unitScale;  // 9.80665f for g→m/s²
    uint16_t    cdrOffset;  // offset inside the target message struct
    CdrType     cdrType;    // float64, etc.
};
static constexpr AttrToCdr kImu[] = { … };
```

### 6.1 Header stamping (resolved — see Q6 answer in §12)

Each polled sample already carries a relative micros-since-boot timestamp
reconstructed by the generated decode function (same piecewise-EMA scheme
used by `DataLogger.cpp`: `decodeState.emaLastSampleTimeUs`). We reuse that
value directly as the sample time origin:

```
sampleAbsUs = sessionBaseRealTimeUs + sampleRelativeUs
                (where sessionBaseRealTimeUs is latched once at setup:
                   SNTP-synced → (gettimeofday() - millis()*1000)
                   else        → 0  // stamp is "uptime", not wall-clock)
stamp.sec     = sampleAbsUs / 1_000_000
stamp.nanosec = (sampleAbsUs % 1_000_000) * 1000
```

`sessionBaseRealTimeUs` is recomputed (re-latched) if SNTP syncs after
RaftROS setup — one-shot check in `loop()` until latched.

Batched callbacks (Q5, FIFO) therefore emit monotonically-increasing
timestamps across the batch without extra work on the RaftROS side — the
values come straight from the decoded record's `timeMs` (or EMA-reconstructed
equivalent) already populated by the existing poll decoder.

- `header.frame_id`: `<alias>` by default; overridable via SysTypes
  `frameIds` map (same pattern-expansion helper as §5.1).

### 6.2 Covariance handling
All `*_covariance[9]` fields in `Imu` are emitted as `[-1, 0, … 0]` to signal
unknown (standard ROS 2 convention). For future per-device calibration JSON,
we can lift this into a table.

---

## 7. Writer Lifecycle & RTPS Integration

This section folds into the existing RTPS runtime modules (§ RaftROS-overview)
rather than duplicating them.

| Concern | Component | Change for Phase 4 |
|---------|-----------|--------------------|
| Participant | existing `RTPSParticipant` | none |
| SPDP | `DiscoveryRunner` | none |
| SEDP publication announce | `SEDPHandler::announcePublications()` | must iterate the **dynamic writer registry** (new), not a static list. |
| Writer history cache | `ReliableWriter` | one instance per dynamic writer; `entityId` allocated from a free-list. |
| HEARTBEAT | `WriterHeartbeatRunner` | multi-writer fan-out (already designed for >1 writer; Phase 3 uses 1 for `/chatter`). |
| Matched-reader set | `RemotePublicationMap` (symmetrically `RemoteSubscriptionMap` — new) | track per-topic matched subscriber GUIDs, for ACKNACK bookkeeping. |
| Dispose-on-offline | `SEDPHandler::disposePublication(writerGuid)` (new) | emits RTPS DATA flags=0x03 with PID_STATUS_INFO + PID_KEY_HASH + PID_SENTINEL. |

### 7.1 Entity-ID allocation
Reserve user-writer entityIds in the range `0x0100xxxx..0x01FFxxxx` (per RTPS
spec for user DataWriters: kind byte `0x02` / `0x03` for builtin vs user
endpoints). A free-list of up to `RAFTROS_MAX_DYNAMIC_WRITERS` IDs is
maintained; exhaustion logs a warning and skips the registration.

### 7.2 QoS defaults (resolved — see Q2 answer in §12)
| Profile | Reliability | Durability | History | Depth | Default-for `clas` |
|---------|-------------|------------|---------|-------|--------------------|
| `fast_sensor` | BEST_EFFORT | VOLATILE | KEEP_LAST | 10 | `ACC`, `GYRO`, `{ACC,GYRO}`, `PROX`, `LGHT`, `DIST`, `ANG`, `HRM`, `FRCE` |
| `slow_sensor` | RELIABLE | VOLATILE | KEEP_LAST | 5  | `TEMP`, `RH`, `PRES`, `SOIL`, `BTHM` |
| `event`       | RELIABLE | VOLATILE | KEEP_LAST | 20 | `BTN`, `TCH`, `ROT`, `GAME` |
| `fallback_string` | RELIABLE | VOLATILE | KEEP_LAST | 10 | any unmapped `clas` |

**Override surface** (SysTypes `RaftROS` block):
```json
"qosProfiles": {
  "imu_1_6a": "slow_sensor",                     // per-device alias override
  "classDefaults": { "ACC": "slow_sensor" }      // per-class override
}
```
Profile resolution order: per-device alias → per-class override → built-in
default from the table above. A profile name may also be an inline JSON
object overriding individual fields (`{"reliability":"RELIABLE","depth":50}`).

---

## 8. ESP32 Resource Budget

Assumptions: ESP32-S3, PSRAM not required.

| Item | Per-writer cost | Notes |
|------|-----------------|-------|
| `ReliableWriter` history cache | ~history-depth × avg-CDR-size bytes | For `fast_sensor` IMU @ depth 10: ~600 B payload; incl. metadata ~1 kB. |
| `DynamicWriterCtx` | ~128 B | writer GUID, entityId, cached `DeviceTypeRecord*`, decode state, AttrToCdr ptr. |
| SEDP announce builder scratch | shared (~1 kB, static) | reused across all writers. |
| CDR scratch buffer | shared (~512 B, static) | per poll-task; re-entrant not required. |

**Target cap:** `RAFTROS_MAX_DYNAMIC_WRITERS = 16` (compile-time), ≈ 24 kB
total heap. Revisit if we see >16 devices on a single bus.

---

## 9. Implementation Plan — Slices

Each slice is independently testable, with unit tests added to
`linux_unit_tests/main.cpp`.

| Slice | Scope | Success criterion | Status |
|-------|-------|-------------------|--------|
| **4.1** | **Dynamic writer registry skeleton** — add `DynamicWriterCtx`, free-list, lookup by `RaftDeviceID`. No wire-side changes yet. | Unit test: register/unregister 20 contexts, free-list invariants hold. | ✅ |
| **4.2** | **Device attach/detach listener** — subscribe to `registerForDeviceStatusChange`; on online, instantiate a `DynamicWriterCtx` + cache `DeviceTypeRecord`; on offline, destroy. Still no RTPS announce. | Unit test with fake DeviceManager: 10 attach/detach cycles, no leaks. | ✅ |
| **4.3** | **Per-device data callback wiring** — `registerForDeviceData(deviceID, cb, rateMs, ctx)` on attach; callback decodes via `pollResultDecodeFn` into a stack/per-ctx buffer; logs decoded values. | Unit test: synthetic poll buffer for `AHT20` decodes to expected temp/humidity floats. | ✅ |
| **4.4** | **Class → mapping table** — implement the §5 dispatch (lookup row by `clas`); for now only `TEMP`, `RH`, `ACC`, `GYRO`, `DIST`, `LGHT` rows active. | Unit test: each `clas` resolves to the documented topic/type. | ✅ |
| **4.5** | **CDR serializers** — hand-written per ROS 2 type used in §5.2 (`Temperature`, `RelativeHumidity`, `FluidPressure`, `Illuminance`, `Range`, `Imu`, `Float32`, `Bool`, `String`). | Byte-for-byte golden tests against captures from `ros2 bag`. | ✅ |
| **4.6** | **SEDP dynamic announce** — extend `SEDPHandler` to iterate the writer registry; emit publication DATA(w) per active writer; honour QoS profile from §7.2. | On-device: `ros2 topic list` shows attached device topics. | ✅ |
| **4.7** | **User-data publishing hot path** — bus-task callback pushes CDR sample onto writer's history cache; `RaftROS::loop()` triggers heartbeat cycle. | On-device: `ros2 topic echo /raft_<host>/imu_1_6a` prints live data. | ✅ |
| **4.8** | **Dispose on offline** — `SEDPHandler::disposePublication()`; tie to offline status callback; free entityId. | On-device: pulling device causes `ros2 topic list` to drop the topic within ≤ 10 s. | ✅ |
| **4.9** | **Fallback path** — `std_msgs/String` JSON serializer for unmapped `clas` and for BLEBTHome. | Unit test: unknown device with synthetic `devInfoJson` publishes well-formed JSON. | ✅ |
| **4.10** | **Composite classes** — Imu, TEMP+RH, PRES+TEMP. | Unit + on-device for LSM6DS and AHT20. | ✅ |
| **4.11** | **QoS profile selector + SysTypes override** — runtime config of per-device topic alias & QoS override. | Config-driven test harness. | ✅ |
| **4.12** | **Documentation & polish** — update `development-status.md` and `overview.md`. | | ✅ |

**Phase 4 complete.** Linux unit tests: **911 passed, 0 failed**.
ESP32-S3 firmware size: `0x1435c0` (25% free on `app` partition).

---

## 10. Risks

1. **CDR alignment bugs.** Mitigation: golden-byte tests per type from a
   `ros2 bag` capture.
2. **Bus-task blocking.** Mitigation: writer history-cache push is O(1) +
   memcpy; heartbeat sender is its own task.
3. **EntityID collisions with builtin endpoints.** Mitigation: centralised
   allocator; reserve Raft range; unit test free-list.
4. **Subscriber mismatch on type hash.** Mitigation: omit type hash (lenient
   matching) — FastDDS accepts this for ROS 2 types.
5. **Heap fragmentation under rapid attach/detach.** Mitigation: pool
   allocator for `DynamicWriterCtx`; reuse writer slots.
6. **Silent MCP9808 clas bug** (see §5.2 note). Mitigation: add a devtype
   validation linter that emits a warning when `clas` conflicts with
   `desc`/attribute names.

---

## 11. Test Plan

### 11.1 Linux unit tests (host)
- Existing 388 pass baseline.
- Add: entity-id free-list, writer registry CRUD, attach/detach lifecycle,
  CDR golden-byte per type, class-mapping dispatch, fallback JSON,
  composite-class precedence.
- Target: +≈40 assertions.

### 11.2 On-device validation recipe
1. `raft r` in `examples/ExampleDiscoverable`, one device connected.
2. Host: `ros2 topic list` — confirm expected topic appears.
3. Host: `ros2 topic echo <topic>` — confirm live stream, correct units.
4. Disconnect device; confirm topic disappears within 10 s.
5. Reconnect; confirm topic reappears under same name and resumes.

---

## 12. Open Questions — Answers & Remaining

Answers received 2026-04-22 are folded into the sections above. Summary:

| # | Topic | Status | Resolution |
|---|-------|--------|------------|
| 1 | Topic naming | ✅ resolved | `/raft_{{hostname}}/<slug>_<bus>_<addrHex>` with SysTypes pattern string (`{{hostname}}`, `{{sysname}}`, `{{mac4}}`) expanded **once at setup**. Per-device alias override via SysTypes + `DeviceManager::setDeviceName`. |
| 2 | QoS | ✅ resolved | Per-class profile defaults (table in §7.2). SysTypes override surface documented: per-device alias and per-class override. |
| 3 | Angle type (`ANG`) | ✅ resolved | `std_msgs/Float32` (degrees). |
| 4 | BLEBTHome fan-out | ✅ resolved | Split per attribute (`.../temp`, `.../battery`, `.../motion`, `.../light`, `.../ID`, `.../MAC`). Each attribute has its own writer + SEDP announce. |
| 5 | FIFO batch handling | ✅ resolved | **Default: most-recent-only** (publish last sample of each callback). Per-device SysTypes override `publishMode: "all"` to emit every decoded sample with its reconstructed timestamp. Subscriber-side history-depth control is via QoS (§7.2). |
| 6 | Time source | ✅ resolved | Use the relative timestamp already produced by the existing poll decoder chain; add a latched `sessionBaseRealTimeUs` derived from SNTP at setup (re-latched if SNTP syncs later). See §6.1. |

**Still-open minor questions (defaults applied, user input welcome):**

| # | Topic | Default applied |
|---|-------|----------------|
| A | Max concurrent dynamic writers | `RAFTROS_MAX_DYNAMIC_WRITERS = 16` (compile-time). |
| B | MCP9808 `clas` mis-tag | Handle as a **special-case row** in the mapping table (`deviceType == "MCP9808"` → `TEMP`). Separately: open a RaftCore issue to fix `DeviceTypeRecords.json`. |
| C | `frame_id` policy | Default `<alias>`; SysTypes `frameIds` map override. |
| D | Diagnostic `/raft_<host>/diag` topic | **Deferred to a later slice** (Phase 4 completion + 1); will reuse `StatePublisher` + `std_msgs/String` path (Option 1 surface). |
| E | Runtime REST toggle | **Deferred.** Phase 4 MVP: SysTypes only; runtime REST (`/api/rosPub`) can be added later without breaking the config surface. |

---

## 13. Configuration Surface Summary

Consolidated SysTypes block expected by Phase 4:

```json
"RaftROS": {
  "enable": 1,
  "autoPublish": {
    "enable": true,
    "rosNamespace": "raft_{{hostname}}",
    "deviceAliases": {
      "1:6a": "main_imu",
      "1:38": "cabin_temp"
    },
    "qosProfiles": {
      "main_imu": "slow_sensor",
      "classDefaults": { "ACC": "slow_sensor" }
    },
    "frameIds": {
      "main_imu": "imu_link",
      "_default": "raft_base"
    },
    "publishMode": {
      "_default": "latest",
      "main_imu": "all"
    },
    "classOverrides": {
      "MCP9808": { "class": "TEMP" }
    }
  }
}
```

All keys are optional; the block reduces to `"autoPublish": { "enable": true }`
for zero-config operation with all defaults.
