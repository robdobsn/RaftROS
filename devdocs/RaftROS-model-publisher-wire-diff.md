# RaftROS — Model-Publisher Wire-Diff Procedure

**Scope update:** 2026-09-17. The commands and captures below validate the
existing RTPS backend. The CDR reference method is reusable for the planned
[Zenoh alternative](RaftROS-zenoh-implementation-plan.md), but RTPS packet
filters and SEDP comparisons are not Zenoh tests.

## Planned Zenoh Model-Publisher Extension

- Run the reference publisher and typed subscriber with the pinned
   `rmw_zenoh_cpp` version, domain and session configuration used by firmware
   interop tests; do not compare a FastDDS reference with a Zenoh capture.
- Capture the actual configured Zenoh endpoints, including TCP stream
   reassembly where used. The existing UDP 7400-7500 filters will miss them.
- Compare data key expressions (domain, normalized topic, type and hash),
   ROS graph-liveliness tokens (node/publisher association, mangling and QoS),
   attachment layout/identity/sequence/time, then the CDR payload separately.
   Normalize expected session IDs and timestamps rather than diffing complete
   packets as though they should be identical.
- Build golden tests from a pinned host serializer and verify with native
   typed deserialization for all mapped types. Never wrap RTPS DATA/SEDP bytes
   in a Zenoh value. Verify that the four-byte CDR encapsulation is included
   exactly once and alignment is relative to the correct origin.
- Check graph attribution and endpoint removal as well as sample delivery.
   Test late joiners, wrong domain/type/hash, reconnect, and malformed or absent
   required metadata. A generic Zenoh subscriber accepting bytes is not a ROS
   interoperability result.
- Record direct/routerless and router-assisted runs separately. The latter is
   a diagnostic control, not standalone acceptance. Keep the current RTPS
   harness unchanged until a backend-specific capture extension is tested.

> **Purpose.** When the ESP32 firmware announces a topic but a ROS 2
> subscriber either (a) cannot discover it, (b) discovers it but never
> receives samples, or (c) receives malformed samples, we need a way to
> *isolate the exact byte-level divergence* between what a canonical
> ROS 2 publisher would emit and what our firmware emits.
>
> This document describes the **model publisher** workflow — running a
> genuine rclpy publisher for the same topic/type/QoS, capturing both
> sides' RTPS traffic, and diffing the pcaps — and provides a reference
> CDR layout for every `sensor_msgs` / `std_msgs` / `geometry_msgs` type
> the auto-publisher can emit so that new types can be validated quickly.

---

## 1. Overview

RaftROS's auto-publisher ([RTPSAutoPubClassMap.h](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubClassMap.h))
emits one of a fixed set of ROS 2 message types. For every supported
type there is a canonical XCDR1 binary layout that the Fast DDS / CycloneDDS
deserialiser expects. Any deviation — wrong field order, missing padding,
wrong numeric width, stray bytes — will be silently dropped by the reader
(no error surfaced; the callback simply never fires).

The wire-diff harness lets us compare our firmware's output against a
**known-good reference publisher** written in rclpy on the same machine,
so that any deviation is obvious at the byte level.

### 1.1 Components

| File | Role |
|---|---|
| [scripts/range_reference_pub.py](../scripts/range_reference_pub.py) | Reference rclpy publisher for `sensor_msgs/Range` on `/raft/range_1_29` |
| [scripts/capture_rtps.sh](../scripts/capture_rtps.sh) | Capture + diff harness (`ref` / `esp` / `diff` modes) |
| [scripts/range_probe.py](../scripts/range_probe.py) | Raw hex-dumping subscriber (used to elicit ESP publication) |
| `/var/tmp/rtps_cmp/` | Output directory for pcaps and decoded PID summaries |

### 1.2 End-to-end flow

```
┌─────────────────────────┐      ┌────────────────────────────────┐
│  ref: host rclpy pub    │      │  esp: ESP32 firmware           │
│  (range_reference_pub)  │      │  (RaftROS autopub)             │
└───────────┬─────────────┘      └───────────────┬────────────────┘
            │ RTPS                                │ RTPS
            ▼                                     ▼
    dumpcap any → ref.pcap             dumpcap eth1 → esp.pcap
            │                                     │
            └──────── capture_rtps.sh diff ───────┘
                         │
                         ▼
          SEDP-PID diff + sample-CDR byte-by-byte diff
```

---

## 2. Running the diff workflow

### 2.1 Prerequisites

- ROS 2 Jazzy installed at `/opt/ros/jazzy` (adjust `ROS_SETUP` env-var for other distros).
- `tshark` + `dumpcap` installed, with the current user in the `wireshark` group **or** able to `sudo dumpcap` without password.
- ESP32 on the same subnet as the host. Confirm with:
  ```bash
  ip -br a | grep 192.168.1
  ```
- The host interface that has the `192.168.1.*` address (typically `eth1` in WSL2 mirrored mode).

### 2.2 Capture the reference (host rclpy publisher)

```bash
cd ~/rdev/raft/RaftROS
bash scripts/capture_rtps.sh ref
```

This:

1. Sources `/opt/ros/jazzy/setup.bash`.
2. Forces `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` so that intra-host traffic stays on the wire (otherwise Fast DDS would use SHM and the pcap would be empty).
3. Starts [range_reference_pub.py](../scripts/range_reference_pub.py) on `/raft/range_1_29`.
4. Starts a local subscriber (so the publisher actually sends DATA rather than idling at HEARTBEAT).
5. Runs `sudo dumpcap -i any -f "udp portrange 7400-7500"` for `$DURATION` (default 15 s) into `/var/tmp/rtps_cmp/ref.pcap`.

### 2.3 Capture the ESP firmware

```bash
bash scripts/capture_rtps.sh esp
```

This launches [range_probe.py](../scripts/range_probe.py) (so the ESP has a matching subscriber to send to) and captures 15 s of traffic from the wired interface (`eth1` by default) into `/var/tmp/rtps_cmp/esp.pcap`.

### 2.4 Run the diff

```bash
bash scripts/capture_rtps.sh diff
```

This produces three outputs:

1. **SEDP PID-set diff** — which discovery `parameterId` values one side emits but the other does not.
2. **Full SEDP publication decode** — `tshark -V` dump of the first `DATA -> publication` submessage from each side, written to `*.sedp.txt`.
3. **Sample-CDR field diff** — the first user-data DATA payload is decoded and its fields are compared field-by-field (stamp, frame_id, radiation_type, fov, min/max, range, trailing bytes).

Example output (from a working Range publication, 24 Apr 2026):

```
================ Range sample CDR diff ================
-- REF: 52 bytes  raw=15beeb69246ff320 10000000 726166745f72616e67655f315f3239 00 010000 00 e866df3e 0ad7233c 0000803f d00ed13d 00000000
   stamp=1777057301.552824612  frame_id='raft_range_1_29' (len=16)
   radiation=1  fov=0.4363  min=0.0100  max=1.0000  range=0.1021
-- ESP: 36 bytes  raw=14030000 c056fe03 05000000 72616674 00 010000 00 00000000 00000000 00000040 bc74133c
   stamp=788.067000000  frame_id='raft' (len=5)
   radiation=1  fov=0.0000  min=0.0000  max=2.0000  range=0.0090

-- field differences --
   frame_id: REF='raft_range_1_29'  ESP='raft'
   fov:      REF=0.4363            ESP=0.0
   min:      REF=0.01              ESP=0.0
   max:      REF=1.0               ESP=2.0
   total:    REF=52                ESP=36
```

---

## 3. Anatomy of an RTPS capture

Every RTPS message captured by the harness has three tiers. Understanding
what each layer carries is critical to knowing which pcap offset to
inspect when the diff points to a divergence.

### 3.1 Participant layer — SPDP (port 7400 / 250)

`DATA(p)` submessages sent periodically on UDP multicast `239.255.0.1:7400`
announcing the participant. Identified by builtin writer entity
**`0x000100c2`**. Contents: `ParameterList` with vendor, protocol-version,
participant GUID, locators, QoS, and ROS 2 extras (`PID_USER_DATA=enclave=/;`,
`PID_ENTITY_NAME`, `PID_PROPERTY_LIST`).

### 3.2 Endpoint layer — SEDP (unicast port 7410 + participant-index)

`DATA(w)` on `0x000003c2` (publication), `DATA(r)` on `0x000004c2`
(subscription). Carries the per-endpoint QoS `ParameterList`:
`PID_TOPIC_NAME`, `PID_TYPE_NAME`, `PID_RELIABILITY`, `PID_DURABILITY`,
`PID_USER_DATA` (typehash for ROS 2 type-description), etc. Matched
participants exchange these before any sample DATA flows.

### 3.3 User layer — sample DATA

`DATA` submessages on the publication's own writer entity (auto-allocated,
ESP uses `0x00011003` for the first user writer). Payload is a
**serialised CDR encapsulation**:

```
+--------+--------+--------+--------+
|  0x00  |  0x01  |  options (u16) |   ← 4-byte CDR_LE encap header
+--------+--------+--------+--------+
|         message body in XCDR1     |
+-----------------------------------+
```

`rtps.issueData` in tshark shows the body **without** the 4-byte
encap header — that is what the per-type layouts below describe.

---

## 4. CDR layouts per supported type

The auto-publisher's serialisers live in
[RTPSAutoPubCDRSerializer.cpp](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.cpp).
All payloads are **XCDR1 little-endian**. Every string is
`{uint32 length incl. NUL, bytes..., NUL}` with 4-byte alignment applied
before subsequent primitive fields. `std_msgs/Header` is always:

```
int32  sec
uint32 nanosec
string frame_id   ← uint32 length + bytes + NUL
```

### 4.1 sensor_msgs/Range

| off | bytes | field | type | notes |
|----:|:------|:------|:-----|:------|
| 0   | `sec` | header.stamp.sec | int32 | wall-clock seconds |
| 4   | `nsec`| header.stamp.nanosec | uint32 | |
| 8   | `len` | header.frame_id.length | uint32 | incl. NUL |
| 12  | ASCII+NUL | header.frame_id.bytes | char[] | pad to 4 |
| +0  | `rt`  | radiation_type | uint8 | `0=ULTRASOUND`, `1=INFRARED` |
| +1  | 3×pad | — | — | align float32 |
| +4  | `fov` | field_of_view | float32 | rad |
| +4  | `min` | min_range | float32 | m |
| +4  | `max` | max_range | float32 | m |
| +4  | `rng` | range | float32 | m |
| +4  | `var` | variance | float32 | 0 = unknown (Jazzy-added field) |

Total size = 32 + padded-frame_id. ROS 2 Jazzy added the trailing
`float32 variance` to `sensor_msgs/Range`; omitting it causes FastCDR to
silently reject the message in typed subscriptions (raw=True still receives
the bytes, which is why `ros2 topic hz` works but `ros2 topic echo` does not).
ESP firmware emits the publisher-topic name as `frame_id` (e.g.
`raft_range_1_29`).

### 4.2 sensor_msgs/Imu (also used for `Accel`-only devices)

```
Header
float64 orientation.x
float64 orientation.y
float64 orientation.z
float64 orientation.w       ← identity = (0,0,0,1)
float64[9] orientation_covariance       ← [0]=-1 → "unknown" (REP-145)
float64 angular_velocity.x / .y / .z     (rad/s)
float64[9] angular_velocity_covariance
float64 linear_acceleration.x / .y / .z  (m/s²)
float64[9] linear_acceleration_covariance
```

Total = header + 4×f64 + 9×f64 + 3×f64 + 9×f64 + 3×f64 + 9×f64 = **header + 296 bytes**.

Field-name sources: `gx`/`gy`/`gz` for gyro (°/s → rad/s);
`ax`/`ay`/`az` or `x`/`y`/`z` for accel (g → m/s²). When `accelOnly=true`
(device only has `ACC` class), the gyro vector is zero-filled and its
covariance uses the "unknown" marker.

### 4.3 sensor_msgs/Temperature, RelativeHumidity, FluidPressure, Illuminance

All four share an identical wire layout:

```
Header
float64 value          ← temperature / humidity / pressure / illuminance
float64 variance       ← 0.0 = "unknown"
```

Unit conversions applied before serialisation:

| type | source field | conversion |
|---|---|---|
| Temperature | `temperature` | none (°C) |
| RelativeHumidity | `humidity` | `% × 0.01` → 0..1 |
| FluidPressure | `pressure` | `hPa × 100` → Pa |
| Illuminance | `als` or `illuminance` | none (lux) |

### 4.4 sensor_msgs/Joy

```
Header
sequence<float32> axes      ← from fields named "x","y" (present → appended)
sequence<int32>   buttons    ← every other non-timestamp field
```

Each sequence is encoded as `uint32 length` + `length × element`.
Alignment after a sequence resets to the element width.

### 4.5 std_msgs/Float32, Int32, Bool

```
Float32 → float32 data    (4 bytes)
Int32   → int32   data    (4 bytes)
Bool    → uint8   data    (1 byte)      ← CDR pads to 4 before encap trailer
```

Field-name priority: `angle` → `moisture` → `value` → `data` → first
non-timestamp field (Float32); `rotation`/`count`/`value`/`data` (Int32);
`press`/`state`/`value`/`data` (Bool).

### 4.6 std_msgs/ByteMultiArray

```
MultiArrayLayout layout:
    sequence<MultiArrayDimension> dim     ← length=0 (always empty)
    uint32 data_offset                    ← 0
sequence<uint8> data                      ← one byte per Bool/Uint8/Int8 field
```

### 4.7 std_msgs/Float32MultiArray

```
MultiArrayLayout layout (empty, as above)
sequence<float32> data                    ← one float per non-timestamp field
                                            (skipping "status" and "valid" flags)
```

Used for `RoboticalLightSensor` (left/center/right channels).

### 4.8 geometry_msgs/Wrench (HX711 load cell)

```
Vector3 force  { float64 x; float64 y; float64 z }    ← x=0, y=0, z=force field (N)
Vector3 torque { float64 x; float64 y; float64 z }    ← all zero
```

No header — `Wrench` is **not** `WrenchStamped`.

### 4.9 std_msgs/String (fallback for unmapped devices)

```
string data       ← uint32 length incl. NUL + bytes + NUL
```

Content is the device's JSON-attribute blob as emitted by
[RTPSAutoPubCDRSerializer_serializeString](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.cpp).

---

## 5. Adding a new message type to the diff workflow

When the auto-publisher gains a new `RTPSAutoPubMsgKind` (e.g. `PointCloud2`, `Twist`, ...):

1. **Find the ROS 2 IDL for the type.** The DDS-wire type name is
   `<pkg>::msg::dds_::<Type>_`. Add it to [RTPSAutoPubClassMap_typeName](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubClassMap.h).
2. **Compute the type-hash.**
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 interface hash <pkg>/msg/<Type>
   ```
   Add the `RIHS01_<64hex>` string to the `lookupRos2TypeHash()` table in
   [SEDPHandler.cpp](../components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp) so
   that `PID_USER_DATA` contains it during SEDP announcement.
3. **Write a reference publisher** alongside [range_reference_pub.py](../scripts/range_reference_pub.py)
   that publishes a canonical instance of the new type on `/raft/<topic>`.
4. **Capture both sides.** `capture_rtps.sh ref` then `esp`.
5. **Extract the user-DATA hex** from each pcap:
   ```bash
   REF_WR=$(tshark -r ref.pcap -Y 'rtps.issueData and ip.src == 127.0.0.1' \
                   -T fields -e rtps.sm.wrEntityId | head -1 | cut -d, -f1)
   tshark -r ref.pcap -Y "rtps.sm.wrEntityId == $REF_WR" \
          -T fields -e rtps.issueData | head -1
   tshark -r esp.pcap -Y 'rtps.sm.wrEntityId == 0x00011003' \
          -T fields -e rtps.issueData | head -1
   ```
6. **Compare byte-by-byte.** The capture_rtps.sh diff branch can be
   extended with a per-type decoder (see the Python block under
   `Range sample CDR diff`) — add a new `elif type == "Foo":` that mirrors
   the layout from section 4.
7. **Re-run the diff until the only remaining differences are timestamps
   and device-specific payload values.**

---

## 6. Known pitfalls

1. **SHM transport hides intra-host traffic.** Fast DDS prefers
   `shared_memory_transport` for same-host participants. Always export
   `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` before the ref publisher; otherwise
   `ref.pcap` will contain zero user-DATA packets.

2. **`ros2 topic echo` without a type fails before the daemon has seen the
   publisher.** Prefer typed form: `ros2 topic echo /topic <pkg>/msg/<T>`,
   or use an rclpy script directly.

3. **tshark display filters do not cross submessages.** Filtering by
   `rtps.sm.wrEntityId == X and rtps.sm.id == 0x15` (DATA submessage)
   always produces zero hits when the two conditions match different
   submessages within the same RTPS packet. Filter on writer alone and
   post-filter the full decode.

4. **The CDR encap header is 4 bytes and `tshark` strips it.** The hex
   returned by `rtps.issueData` is the *body*; the leading `00 01 00 00`
   (little-endian XCDR1) is not in that hex. When comparing against
   `struct`-packed Python output, remember to skip the encap header in
   any hand-computed reference.

5. **String fields are NUL-terminated in CDR.** Length includes the NUL.
   A firmware bug that writes `len=N` but `N` non-NUL bytes yields a
   payload that tshark dissects fine but that Fast DDS *may* accept and
   then report via `frame_id` truncation (see §4.1 Range: `"raft"` vs
   `"raft_range_1_29"`).

6. **Byte-level alignment matters between fields of different widths.**
   A Range payload has `uint8 radiation_type` followed by `float32[4]` — CDR
   requires three pad bytes before the floats. Forgetting these pads
   silently shifts every subsequent field and produces a "matches at
   discovery but never receives samples" symptom.

7. **The Range-writer `frame_id` truncation bug currently present** is
   cosmetic: Fast DDS will still deserialise the payload. It is *not* the
   cause of zero-sample-delivery. Participant-level SPDP gaps
   (missing `PID_ENTITY_NAME`, `PID_PROPERTY_LIST`) are the current
   prime suspect — see [RaftROS-development-status.md](RaftROS-development-status.md).

---

## 7. Next steps

- Extend the `diff` branch of [capture_rtps.sh](../scripts/capture_rtps.sh)
  with per-type Python decoders for **Imu**, **Temperature**,
  **Illuminance**, and **Float32MultiArray** so that we can validate each
  serialiser against its rclpy equivalent with the same one-command
  workflow.
- Add a **`ref-all`** mode that launches one rclpy publisher per supported
  `RTPSAutoPubMsgKind`, so a single capture covers every message layout.
- Record a known-good reference pcap for each type under
  `test_data/rtps_reference_pcaps/` so future CI regressions can diff
  against a golden file rather than requiring a live host publisher.
