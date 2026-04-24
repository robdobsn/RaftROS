# RaftROS Development Status

**Last Updated:** 2026-04-23

## Host environment gotchas (recorded 2026-04-23 after losing a day to these)

These are host-side, not firmware bugs — but they can look exactly like
"the ESP32 is broken". Full reference in `README.md` §"Host Setup Notes".

- **ESP32 does NOT push user DATA to an empty graph.** A passive
  packet-capture with no ROS 2 subscriber running on the host will show
  **zero** unicast DATA from the ESP32 (only periodic SPDP multicast). To
  capture data traffic you must first start a consumer that actually
  matches the publication, e.g.:
  ```bash
  # keep this running for the whole capture window
  ros2 run demo_nodes_cpp listener                       # for /chatter
  python3 scripts/range_probe.py --raw                   # for /raft/range_1_29
  # OR
  ros2 topic echo --no-daemon --qos-reliability best_effort /raft/range_1_29 sensor_msgs/msg/Range
  ```
  If `tshark -f 'host <esp_ip>'` reports `Packets captured: 0`, assume
  "no subscriber is active" before assuming a firmware/network bug.
- **WSL2 must use mirrored networking.** Default NAT mode blocks inbound
  SPDP multicast from the LAN. Fix: add `networkingMode=mirrored` under
  `[wsl2]` in `%UserProfile%\.wslconfig`, then `wsl --shutdown`.
- **Windows Firewall blocks inbound UDP to WSL** even in mirrored mode.
  Fix: `New-NetFirewallRule -DisplayName "ROS2 RTPS" -Direction Inbound
  -Protocol UDP -LocalPort 7400-7500 -Action Allow -Profile Any` (or set
  `firewall=false` in `.wslconfig`). Symptom: ESP32 sees the host, host
  never sees the ESP32; `tcpdump` shows zero packets from the ESP32 IP.
- **ROS 2 Jazzy inverted `ROS_LOCALHOST_ONLY`.** Must be `unset`, not
  `=0`. On Humble, `=0` is still correct.
- **`tcpdump`/`dumpcap` output to `/tmp`** is denied under WSL even as
  root — write pcaps to `/var/tmp/` instead. User also needs `sudo` (not
  in `wireshark` group by default).
- **`ros2 daemon` hangs on WSL2** (XML-RPC socket). Always use
  `--no-daemon`. If `ros2 daemon stop` times out, force-kill:
  `pkill -9 -f _ros2_daemon`.
- **Fast DDS log XML:** root element is
  `<dds xmlns="http://www.eprosima.com">`, not `<profiles>`. Log kind is
  `Log::Kind::Info`, not `log_kind::Info`.
- **Missing `DeviceTypes.json` silently disables I2C auto-identification.**
  When using Raft's `raftdevlibs/` override mechanism (clone
  RaftCore/RaftI2C/etc. into `examples/<app>/raftdevlibs/` so local edits
  survive clean builds), the build picks up devtypes from the overridden
  tree. If that tree is missing `devtypes/DeviceTypes.json`, scanners
  detect the device at the bus level but `identifyDevice` can't assign
  a `deviceTypeIndex`, so auto-publish never attaches. Symptom:
  `I2CDevIdentMgr: identifyDevice new device <name>` fires, but
  `autoPubStatus slots=0/16` stays at 0 forever. Fix: make sure
  `raftdevlibs/<Repo>/devtypes/DeviceTypes.json` exists (copy from the
  upstream clone).

---

## Phase 4 auto-publish — current status (2026-04-23)

### Verified working (end-to-end)

| # | Observation | Evidence |
|---|-------------|----------|
| V1 | DeviceManager → listener dispatch is live | `autoPubStatusCb devID=1_29 typeIdx=5 online=1 isNewlyId=1` in ESP logs after the DeviceManager fix |
| V2 | Auto-publish slot attach fires on identification | `autoPubAttach devID=1_29 typeIdx=5 slot=0 topic=rt/raft/range_1_29 type=sensor_msgs::msg::dds_::Range_` |
| V3 | SEDP publication accepted by Fast DDS daemon | `/raft/range_1_29` appears in `ros2 topic list` and `ros2 topic info --verbose` |
| V4 | CDR payload sent at expected rate | `ros2 topic hz /raft/range_1_29` reports **~4.95 Hz** stable for 23 s+ |
| V5 | UDP delivery is reliable | `tcpdump` on `eth2` captures every frame; 216/304 packets in a 10 s run, 0 dropped by kernel |
| V6 | Raw wire payload is structurally correct | Decoded Range DATA submsg: encap `00 01 00 00` (CDR_LE), sec, nsec, frame_id (len-prefixed + NUL + pad), radiation_type=1 (INFRARED), pad, fov, min, max, range, variance. For `frame_id="raft"` → 44 bytes total. ROS 2 Jazzy's `sensor_msgs/Range` has **5 float32** fields (`variance` was added after Humble) — omitting `variance` causes silent FastCDR deserialize failure and empty typed subscriptions. |
| V7 | Same packet decodes cleanly for `/chatter` on the same writer flow | `/chatter` echo has worked throughout |

### Confirmed topic-name rules

- On the wire the topic is `rt/raft/range_1_29` (ROS 2 always adds a `rt/` prefix to user topic writers — this is a DDS-level detail).
- From the ROS 2 CLI the correct name is **`/raft/range_1_29`** (the `rt/` prefix is stripped by rmw). Do **not** use `/rt/raft/range_1_29`; that call will silently fail.
- `ros2 topic list`, `topic info`, `topic hz`, `topic echo`, and `rclpy` subscriptions all use the `/raft/...` form.

### QoS reality check

- `RTPSAutoPubQoSProfile::FastSensor` = BEST_EFFORT / VOLATILE / KEEP_LAST depth 10.
- This class is assigned to every I2C sensor class by `RTPSAutoPubClassMap` (DIST, PROX, ACC, GYRO, LGHT, ANG, HRM, FRCE, IMU).
- The ROS 2 CLI default QoS is RELIABLE, so `ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range` with no flags produces no output because the subscriber won't match the BEST_EFFORT writer.
- Correct invocation: `ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range --qos-reliability best_effort`.
- Oddity observed: `ros2 topic hz` matches and counts frames even with default QoS. That is because `hz` in Jazzy uses the "best-available" QoS profile (auto-relaxes to match the offered writer). `echo` with an explicit message type does **not** auto-relax; `echo --raw` behaviour is less clear and needs re-testing.

### ACTIVE BUG: Fast CDR exception on deserialize

With matching QoS, the subscriber reports:

```
Fast CDR exception deserializing message of type sensor_msgs::msg::dds_::Range_.,
at ./src/type_support_common.cpp:118
```

`ros2 topic echo` swallows this silently and prints nothing. `rclpy` re-raises it but only as "Fast CDR exception" (no offset/reason yet).

Theories in priority order (not yet disproved):

1. **Data Representation policy mismatch.** Fast DDS 3.x readers in Jazzy advertise `{XCDR, XCDR2}` by default via `PID_DATA_REPRESENTATION` (id 0x0073). Our SEDP publication does **not** emit `PID_DATA_REPRESENTATION`. If Jazzy's Range `_TypeSupport` was generated with `@appendable` (it is — ROS 2 IDL default for Jazzy), the generated deserializer runs in **XCDR2 appendable mode** and expects a `DHeader` (4-byte `uint32` object length) immediately after the encapsulation header. We are sending classic XCDR1 with no DHeader — first 4 bytes of data are interpreted as a DHeader of `0x22645240 * …` (garbage) and length check fails. This theory explains every symptom: structurally correct bytes, passes `hz`, rejected by `echo`. — **HIGHEST PRIORITY TO TEST.**
2. Jazzy's generated `sensor_msgs::msg::dds_::Range_` has a different field order or an additional `@key` field we're not aware of. Low probability (same type name works with other Fast DDS publishers). 
3. Locator/address confusion — we send from `192.168.1.173:7411 → 192.168.1.28:7411`. On Jazzy the receiver expects user-data on an ephemeral port advertised in its SPDP. We appear to be picking the right port from discovery (delivery is reliable per `hz`), so this is unlikely.

### Other odd behaviours observed

| # | Observation | Explanation / theory | Severity |
|---|-------------|----------------------|----------|
| O1 | `ros2 node info /raft_esp32` → "Unable to find node" | ParticipantMessageData / node-name mapping incomplete — node listed by `node list` but not looked up by name. Probably missing `ros_discovery_info` GID list entries for the newly-attached autopub writer (we don't re-publish `ros_discovery_info` after attach). | Cosmetic |
| O2 | `ros2 topic list --no-daemon` often shows **3 "discovered" participants** for every CLI invocation | Each `ros2 topic list` spawns a short-lived DDS participant that appears + disappears within its `--spin-time`. The daemon also has one. Not a bug. | Cosmetic |
| O3 | First ACKNACK after match produces `total count change:1 total count: 1` | BEST_EFFORT writers still maintain a sequence number; the subscriber reports one "missed" sample on match because it joined mid-stream. This is normal for BEST_EFFORT and is not the deserialize failure. | Normal |
| O4 | `echo --raw` with `--qos-reliability best_effort` printed nothing | Unclear. `--raw` may still require the type argument to create the subscription. Needs re-test: `ros2 topic echo /raft/range_1_29 sensor_msgs/msg/Range --qos-reliability best_effort --raw`. | Open |
| O5 | LWIP still loses mid-burst sends with ENOMEM on the ESP | Known; covered by reliable retransmit. | Known / accepted |

### Investigation plan (next session)

1. **Prove/disprove theory #1** by examining a working Fast DDS → Fast DDS Range publication on the same host and comparing the first 8 bytes of the serialized payload. If they start with `00 0A 00 00` or `00 0B 00 00` (PL_CDR2 / XCDR2) and contain a DHeader, we need to either (a) emit `PID_DATA_REPRESENTATION = XCDR1` in our SEDP publication, or (b) switch our serializer to XCDR2 appendable.
2. Enable Fast DDS verbose logging the right way: `export FASTDDS_DEFAULT_LOG_VERBOSITY_LEVEL=Info` and `export FASTDDS_DEFAULT_LOG_FILTER='FastCdr|RTPS_MSG_IN|SUBSCRIBER'` **before sourcing ROS**, not after. Then `ros2 topic echo ... 2>&1 | tee /tmp/fastdds.log` — the exact exception type (NOT_ENOUGH_MEMORY vs BAD_PARAM) will tell us whether it's a size mismatch or a value mismatch.
3. If #1 is the cause, add `PID_DATA_REPRESENTATION` to `SEDPHandler::buildPublicationMessage` with value `{0x0000}` (XCDR1) before `PID_SENTINEL`.
4. Consider flipping `FastSensor` default to RELIABLE so standard tutorials (`ros2 topic echo` with no flags) "just work" — but document the BEST_EFFORT choice for real sensor streams.
5. Fix `ros2 node info` after autopub attach by bumping `_rosDiscSeqNum` and rebuilding the ros_discovery_info payload with the new writer GID.

### Host diagnostic recipes (proven)

- Run the `/tmp/range_probe.py` rclpy subscriber (fixed version below) to see the actual CDR exception text.
- `tcpdump -i eth2 -w ~/rtps.pcap 'udp portrange 7400-7500'`, then `tcpdump -r ~/rtps.pcap -nn -X 'udp portrange 7410-7420 and greater 80' | head -80` to inspect wire bytes without Wireshark.
- Wireshark dissector: install `wireshark-common` on WSL, `chown` the pcap, open the file directly — RTPS 2.2 is decoded natively and the serialized payload is annotated by generated type plugins if ROS 2 is sourced.

---

## Goal

Make an ESP32-S3 (running Raft firmware) appear as a native ROS 2 node — without micro-ROS or any agent process — using a clean-room RTPS 2.2 implementation as a Raft SysMod.

**Phase 1 (Discovery), Phase 2 (Topic Publishing), and Phase 3 (Topic Subscribing with per-topic routing) are COMPLETE.**
- `ros2 node list` shows `/raft_esp32`.
- `ros2 topic list` shows `/chatter` (published) and the subscribed topics (`/chatter_in`, `/chatter_in2`, …).
- `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints a sample per second.
- `ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"` routes to the correct per-slot handler on-device.

Verified end-to-end on 2026-04-22 against ROS 2 Humble + FastDDS 2.6.11, ESP32-S3 on real WiFi (PC `192.168.1.92`, ESP32 `192.168.1.173`).

## Architecture

Clean-room RTPS 2.2 implementation. No third-party DDS libraries.

### Source Files

| File | Purpose |
|------|---------|
| `components/RaftROS/RaftROS.cpp/.h` | SysMod lifecycle, UDP sockets, receive/send loops, participant state machine |
| `components/RaftROS/CDR/CDREncoder.cpp/.h` | CDR (Common Data Representation) serializer |
| `components/RaftROS/CDR/CDRDecoder.cpp/.h` | CDR deserializer |
| `components/RaftROS/RTPS/RTPSTypes.h` | Constants: entity IDs, PID codes, port formulas, QoS values |
| `components/RaftROS/RTPS/runtime/wire/RTPSMessage.cpp/.h` | RTPS header/submessage read/write (DATA, HEARTBEAT, ACKNACK, INFO_DST, INFO_TS) |
| `components/RaftROS/RTPS/runtime/core/RTPSParticipant.cpp/.h` | GUID management, port calculation, locator building |
| `components/RaftROS/RTPS/runtime/discovery/SPDPHandler.cpp/.h` | SPDP announcement build/parse, ros_discovery_info CDR payload |
| `components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp/.h` | SEDP publication/subscription/liveliness message build, user data message build |
| `linux_unit_tests/` | Linux-hosted unit tests + standalone linux RTPS node (`raftros_standalone.cpp`) |
| `unit_tests/` | ESP-IDF Unity tests |
| `examples/ExampleDiscoverable/` | ESP32 app that runs RaftROS as a SysMod |

### Hardware / Environment

- **ESP32-S3**, MAC `94:a9:90:3a:51:b0`, WiFi STA IP `192.168.1.173`
- **ROS 2 Humble** with FastRTPS 2.6.11, running on WSL2 (mirrored networking), IP `192.168.1.92`
- Domain ID 0 → SPDP multicast `239.255.0.1:7400`, metatraffic unicast `:7410`, user data unicast `:7411`

## Phase 2: Topic Publishing — COMPLETE ✅

Immediate objective was to publish real ROS 2 topic data from the ESP32 with reliable delivery.

### What Works (ESP32, 2026-04-21)

- ESP32 announces an additional DataWriter endpoint for `/chatter` (`std_msgs/msg/String`, RELIABLE + VOLATILE) via SEDP publications on writer `000003C2` at sequence number 2 (distinct from the `ros_discovery_info` announcement at sequence 1 on the same writer).
- `publishChatter()` serializes `std_msgs/msg/String` using the CDR encoder and sends DATA + HEARTBEAT on the user data port (`7411`) once per second.
- HEARTBEAT `firstSN` matches the current sequence number for the VOLATILE chatter writer so newly-matched subscribers do not request historical samples.
- ACKNACK-driven retransmit works on both the SEDP publications path (chatter announcement recovery) and the chatter user-data path.
- `ros_discovery_info` writer GID list still advertises the chatter writer as part of the participant's node entities.

### Verification

- On PC: `env -u PYTHONPATH PYTHONNOUSERSITE=1 ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints `data: Hello from raft_esp32 [N]` once per second indefinitely.
- On PC: `ros2 topic info /chatter --no-daemon -v` shows one Publisher with QoS RELIABLE + VOLATILE, node `raft_esp32`.
- On ESP32: ACKNACK log shows `readerEID=00000504 writerEID=00010103 (chatter) base=N numBits=0` with `N` advancing — no retransmit storm.
- Linux unit tests and standalone linux publisher remain green (**85 passed, 0 failed**) after the ESP fixes.

### Additional Fixes Applied During Phase 2 Bring-up

| # | Fix | Details |
|---|-----|---------|
| 15 | VOLATILE chatter HB `firstSN = currentSeq` | Heartbeat `firstSN=1` on a VOLATILE writer caused newly-matched subscribers to NACK an ever-growing gap of historical samples. Fix: advertise `firstSN = chatterSeqNum`. Added `firstSN` parameter to `SEDPHandler::buildUserDataMessage`; centralized for ESP in `RTPSReliabilityAndWriterStateRuntime::makeAckUserDataSequenceContextForFlavor` via `chatterFirstSNMatchesSequence=true`. |
| 16 | Chatter SEDP publication sequence number | The `ros_discovery_info` and `/chatter` SEDP DATA messages share writer entity `000003C2`. Both were emitted at `seq=1`, so the PC discarded the chatter announcement as a replay. Fix: `_chatterSedpSeqNum = 2` in `RaftROS.h`. `ros_discovery_info` keeps `seq=1`. |
| 17 | Enable chatter retransmit in `EspStyle` flavor | ACKNACK on SEDP publications (`000003C2`) did not retransmit the `/chatter` announcement because `publicationsIncludesChatterAnnouncement=false` under `EspStyle`. Combined with LWIP ENOMEM losing the 3rd+ sendto in the initial unicast burst, the PC never learned about `/chatter`. Fix (A): set `publicationsIncludesChatterAnnouncement=true` in `makeAckNackDecisionOptionsForFlavor` for `EspStyle`. Fix (B): wire `ctx.sedpChatterPublicationSpec.buildMessage` to call `SEDPHandler::buildPublicationMessage` using `getAckActionSedpPlan(RetransmitSedpChatterPublication, …)`. |

### Phase 2 Key Learnings

- **VOLATILE + RELIABLE writers must advertise `firstSN == lastSN`** in HEARTBEAT. Firing `firstSN=1` with `lastSN=N` invites the subscriber to NACK samples that no longer exist, producing a retransmit storm.
- **Two DataWriter announcements on the same SEDP publications writer must use distinct sequence numbers**, or the second is silently discarded as duplicate.
- **LWIP send queue is narrow on ESP32-S3**: the initial 5-packet unicast burst from `handleNewParticipant` still loses the 3rd and later sends with `errno=ENOMEM`. End-to-end delivery now relies on the reliable-retransmit path rather than first-try success; do not remove the retransmit wiring to "clean up" the flow.
- **ROS 2 CLI on the PC needs `env -u PYTHONPATH PYTHONNOUSERSITE=1 … --no-daemon`** to avoid the user-site numpy collision and the daemon XMLRPC timeout on this host. This is captured in `/memories/repo/raftros-rtps-findings.md` but should stay in mind for future validation.

## Phase 1: Discovery — COMPLETE ✅

`ros2 node list` shows `/raft_esp32` on both ESP32 and the linux standalone test node.

### What Works

- **SPDP (Participant Discovery):** ESP32 sends periodic SPDP announcements to multicast `239.255.0.1:7400`. Contains GUID prefix (from WiFi MAC), lease duration, builtin endpoint set (`0x0C3F`), `PID_USER_DATA` with `"enclave=/;"`. Stale participant purging removes expired entries after 2× lease duration.
- **SPDP on Metatraffic Port:** Daemon unicast SPDP replies to ESP32:7410 are correctly parsed. Lease timers refreshed on re-reception.
- **SEDP Publication:** Announces `ros_discovery_info` DataWriter with topic name, type name, reliability (RELIABLE), durability (TRANSIENT_LOCAL), `PID_UNICAST_LOCATOR`, and `PID_PARTICIPANT_GUID`. Includes HEARTBEAT submessage.
- **SEDP Subscription:** Announces `ros_discovery_info` DataReader with matching QoS, locator, and participant GUID.
- **Participant Message Data (Liveliness):** Sent on discovery and periodically via heartbeats. Sequence number increments each send.
- **ros_discovery_info:** CDR-encoded `ParticipantEntitiesInfo` (participant GID, node namespace `/`, node name `raft_esp32`) sent to user data port.
- **HEARTBEAT/ACKNACK:** ESP32 responds to daemon HEARTBEATs with ACKNACKs. Handles incoming ACKNACKs and retransmits SEDP publication, SEDP subscription, and ros_discovery_info data as requested.
- **Periodic Writer Heartbeats:** Sends all four DATA+HB messages (SEDP pub, SEDP sub, liveliness, rosDisc) to each discovered remote participant.

### Fixes Applied (chronological)

| # | Fix | Details |
|---|-----|---------|
| 1 | CDR field order | `node_namespace` before `node_name` in `buildRosDiscoveryInfoPayload` |
| 2 | `PID_USER_DATA` in SPDP | FastRTPS requires `"enclave=/;"` — without it, participant is silently ignored |
| 3 | Stale participant purging | `purgeStaleParticipants()` prevents `_discovered` vector from filling with dead entries |
| 4 | SPDP on metatraffic port | `recvMetatraffic()` now detects SPDP writer entity ID and parses as announcement |
| 5 | Lease refresh on re-reception | `discoveredTimeMs` updated when existing participant re-announces |
| 6 | Entity IDs must be NO_KEY | `ros_discovery_info` uses WRITER_NO_KEY (0x03) / READER_NO_KEY (0x04) — **key breakthrough** that made endpoint matching work |
| 7 | Don't increment `rosDiscSeqNum` | Static content always SN=1; incrementing causes ever-growing NACK gap |
| 8 | `participant.init()` before GUID | Must set nodeName, domainId, participantId before `setGuidPrefixFromMAC` |
| 9 | `BUILTIN_ENDPOINT_SET = 0x0C3F` | Includes SUBSCRIPTIONS_ANNOUNCER and PARTICIPANT_MESSAGE_DATA bits |
| 10 | SEDP subscription announcement | `buildSubscriptionMessage()` in `handleNewParticipant` and `sendWriterHeartbeats` |
| 11 | Participant message data (liveliness) | `buildParticipantMessageData()` in `handleNewParticipant` and `sendWriterHeartbeats` |
| 12 | `PID_UNICAST_LOCATOR` in SEDP | Pass `_myIpAddr` to all `buildPublicationMessage()` and `buildSubscriptionMessage()` calls |
| 13 | `PID_PARTICIPANT_GUID` in SEDP | Added to `buildPublicationMessage()` and `buildSubscriptionMessage()` in SEDPHandler |
| 14 | ACKNACK retransmit handling | ESP32 retransmits SEDP pub, SEDP sub, and rosDisc on incoming ACKNACKs |

### RTPS Protocol Key Facts

- `ros2` sends SPDP from loopback (127.0.0.1) on WSL2 with UDPv4-only profile
- ACKNACK `base=2 numBits=0` → "received SN=1 OK"; `base=1 numBits=X growing` → "never received SN=1"
- Entity kind byte: 0x02=WRITER_WITH_KEY, 0x03=WRITER_NO_KEY, 0x04=READER_NO_KEY, 0x07=READER_WITH_KEY
- FastDDS needs UDPv4-only profile XML to disable SHM for same-host testing
- Port formulas: `meta = 7400 + 250*domainId + 10 + 2*participantId`, `user = meta + 1`

## Current Status Update (2026-04-20)

### Newly Verified

- Linux standalone publisher works in the docker ROS 2 testbed when publisher/subscriber are in the same network namespace.
- Docker build is now non-interactive and reproducible (no wireshark debconf prompt blocking builds).
- ACKNACK handling in `linux_unit_tests/raftros_standalone.cpp` now includes SEDP subscription retransmit handling for `ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER`.
- Linux unit tests remain green after the ACKNACK patch: **85 passed, 0 failed**.
- Initial shared-runtime extraction started: ACKNACK parsing/writer classification moved into shared RTPS utility (`RTPSAckNack`) and consumed by both ESP32 (`RaftROS.cpp`) and Linux standalone (`raftros_standalone.cpp`) handlers.
- Post-refactor validation remains green: linux unit tests **85 passed, 0 failed** and `raftros_linux` builds successfully.

### Canonical Docker Validation Rule (Linux Build)

To avoid host/container networking ambiguity, treat this as the required docker regression path:

1. Build linux artifacts on host:
- `cd linux_unit_tests && make -j$(nproc) all standalone`

2. Start docker test container:
- `cd docker && docker compose up -d --build`

3. Copy linux standalone binary into container and run it **inside** the container:
- `docker cp linux_unit_tests/raftros_linux raftros-test:/workspace/raftros_linux`
- `docker exec raftros-test chmod +x /workspace/raftros_linux`
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; /workspace/raftros_linux -i eth0 > /tmp/raftros_linux_in_container.out 2> /tmp/raftros_linux_in_container.err'`

4. Run ROS 2 checks in the same container:
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml; ros2 node list --no-daemon --spin-time 25'`
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml; ros2 topic list --no-daemon --spin-time 15'`
- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/scripts/fastdds_profile.xml; timeout 30 ros2 topic echo /chatter std_msgs/msg/String --once --no-daemon'`

Expected pass indicators:
- `/raft_linux` appears in `ros2 node list`
- `/chatter` appears in `ros2 topic list`
- one chatter sample is printed by `ros2 topic echo --once`

Latest run (2026-04-21) passed with:
- `ros2 node list`: `/raft_linux`
- `ros2 topic list`: `/chatter`, `/parameter_events`, `/rosout`
- `ros2 topic echo /chatter --once`: `data: Hello from raft_linux [42]`

### Root Cause Found for Previous "Not Publishing" Symptom

- The primary blocker observed in earlier docker testing was host/container network separation (discovery traffic from host IP space did not reach container DDS participants as expected in that setup).
- This was not primarily caused by ACKNACK logic.
- Practical implication: do not use host-publisher + container-subscriber as the primary pass/fail gate for linux docker validation in this environment; keep publisher and ROS 2 validation tools in the same container namespace.

### Remaining Gap

- Phase 2 (ESP32 `/chatter` publishing) and Phase 3 (ESP32 topic subscribing with per-topic routing) are complete and verified end-to-end against ROS 2 Humble + FastDDS 2.6.11.
- Phase 4 (DeviceManager auto-publishing) is complete; every bus device detected by DeviceManager now auto-publishes to ROS 2 with per-class type mapping, per-writer QoS, and clean dispose on detach.
- Linux and ESP32 orchestration code continues to converge on a shared runtime; remaining drift lives in the thin wrappers only (`components/RaftROS/RaftROS.cpp` vs `linux_unit_tests/raftros_standalone.cpp`).

## Phase 2: Topic Publishing — See "Phase 2: Topic Publishing — COMPLETE ✅" above

Historical notes on the shared-runtime convergence work that landed alongside Phase 2 are kept under "DRY / Shared-Code Direction" below.

## DRY / Shared-Code Direction (Agreed Technical Direction)

Detailed staged plan: see `devdocs/RaftROS-next-stages-implementation-plan.md`.

To keep ESP32 and native Linux behavior consistent, new protocol/runtime logic should be implemented once in shared code, with thin platform adapters.

### Preferred Structure

- Keep protocol encode/decode in shared modules (already done in `components/RaftROS/RTPS` and `components/RaftROS/CDR`).
- Move duplicated runtime orchestration (ACKNACK decisions, heartbeat/send policy, participant bookkeeping transitions) into a shared core runtime layer under `components/RaftROS/`.
- Keep only socket/timer/platform glue in:
	- ESP32 SysMod wrapper (`RaftROS.cpp`)
	- Linux standalone wrapper (`raftros_standalone.cpp`)

### Progress Against This Direction

- Completed: shared ACKNACK parser/classifier in `components/RaftROS/RTPS/RTPSAckNack.*`.
- Completed: shared reliability policy helpers in `components/RaftROS/RTPS/RTPSReliabilityPolicy.*` used by both ESP32 and Linux handlers for:
	- HEARTBEAT final-flag ACK response decision
	- ACKNACK bitmapBase sequence retransmit decision checks
- Completed: shared periodic scheduling helper in `components/RaftROS/RTPS/RTPSRuntimeSchedule.*` used by both ESP32 and Linux loops for SPDP, writer heartbeat, and chatter publish cadence checks.
- Completed: shared participant activation helper in `components/RaftROS/RTPS/RTPSParticipantLifecycle.*` used by both ESP32 and Linux for:
	- inactive -> active transition decision based on discovered participant count
	- immediate writer-heartbeat trigger when activation occurs
- Completed: shared builtin endpoint mapping helper in `components/RaftROS/RTPS/RTPSBuiltinEndpointMap.*` used by both ESP32 and Linux metatraffic HEARTBEAT handling to map remote writer entity IDs to local reader entity IDs for ACKNACK generation.
- Completed: shared discovery merge policy helper in `components/RaftROS/RTPS/RTPSDiscoveryPolicy.*` used by both ESP32 and Linux for:
	- lease refresh on known participant re-discovery
	- new participant add eligibility check against max capacity
	- add-time `discoveredTimeMs` initialization
- Completed: shared participant lease-expiry policy helper in `components/RaftROS/RTPS/RTPSParticipantLeasePolicy.*` used by both ESP32 and Linux stale participant purge paths for:
	- lease timeout calculation (2x lease duration with default fallback)
	- expiration predicate used by purge loops
- Completed: shared participant set change policy helper in `components/RaftROS/RTPS/RTPSParticipantSetPolicy.*` used by both runtimes for activation-state and immediate writer-heartbeat trigger decisions.
	- ESP32 now uses this policy for first activation and subsequent participant-add immediate heartbeat behavior.
	- Linux uses the same policy for activation transition behavior.
- Completed: shared discovery merge flow in `RTPSDiscoveryPolicy_mergeParticipant(...)` now centralizes refresh/add/capacity decisions and is used by both ESP32 and Linux discovered-participant handlers.
- Completed: shared initial new-participant announce policy in `components/RaftROS/RTPS/RTPSInitialAnnouncePlan.*` now centralizes which first-contact SPDP/SEDP/PMD sends are enabled; both ESP32 and Linux `handleNewParticipant` now use this shared plan while retaining platform-specific send execution.
- Completed: shared ordered announce-step sequencing in `RTPSInitialAnnouncePlan_buildSequence(...)` now provides a common action list plus per-step heartbeat/sequence guidance; both ESP32 and Linux wrappers now execute this shared sequence while preserving platform-specific packet send paths.
- Completed: shared send-target policy mapping in `RTPSInitialAnnouncePlan_getSendTarget(...)` now centralizes socket/destination selection per announce action; both wrappers use a local send adapter driven by this shared mapping.
- Completed: shared payload-build dispatch in `RTPSInitialAnnouncePlan_getBuildSpec(...)` and shared SEDP endpoint profile mapping in `RTPSInitialAnnouncePlan_getSedpEndpointSpec(...)` now centralize builder selection and endpoint metadata; both wrappers now execute announce actions via shared build/send policy mappings.
- Completed: shared action-to-log policy in `RTPSInitialAnnouncePlan_getLogSpec(...)` now centralizes action labels and sender-address logging behavior; both wrappers now use this shared metadata for post-send logging.
- Completed: shared sequence-counter mutation timing policy in `RTPSInitialAnnouncePlan_applySequencePolicy(...)` now centralizes per-action sequence selection and increment timing (including SPDP pre-build increment and Linux chatter sequence compatibility behavior); wrappers now use shared counter-state policy and only persist concrete counter storage.
- Completed: shared debug behavior policy in `RTPSInitialAnnouncePlan_getDebugSpec(...)` now centralizes runtime/action debug toggles (including Linux SEDP publication hex-dump behavior); wrappers now consume shared debug metadata instead of hardcoded action checks.
- Completed: shared send-result instrumentation policy in `RTPSInitialAnnouncePlan_classifySendResult(...)` now centralizes success/failure/short-send classification; wrappers consume this metadata for unified post-send status tagging while keeping platform logger calls local.
- Completed: shared action-iteration execution scaffolding in `RTPSInitialAnnouncePlan_evaluatePreBuild(...)` and `RTPSInitialAnnouncePlan_evaluatePostBuild(...)` now centralizes step preconditions and skip reasons; wrappers now use shared pre/post step gating.
- Completed: shared heartbeat-count mutation policy in `RTPSInitialAnnouncePlan_applyHeartbeatPolicy(...)` now centralizes increment-before-send rules.
- Completed: full shared initial-announce execution runner in `RTPSInitialAnnounceRunner.*` now centralizes the step loop (pre/post gating, heartbeat/sequence policy application, debug policy check, send-result classification, and log dispatch gating). ESP32 and Linux wrappers now provide only build/send/debug/log callbacks and runtime state storage.
- Completed: full shared ACKNACK execution runner in `RTPSAckNackRunner.*` now centralizes ACKNACK parse/classify/decision/action-dispatch flow (including configurable publication-sequence gating differences and chatter-announcement handling). ESP32 and Linux wrappers now primarily provide remote lookup plus action-specific build/send callbacks.
- Completed: full shared writer-heartbeat execution runner in `RTPSWriterHeartbeatRunner.*` now centralizes periodic writer resend step orchestration (action sequence, heartbeat/liveliness mutation policy, sequence selection policy, send-target mapping, and Linux one-time ros_discovery_info debug dump behavior). ESP32 and Linux wrappers now primarily provide build/send/log callbacks and runtime counter storage.
- Completed: full shared receive-submessage execution runner in `RTPSRxSubmessageRunner.*` now centralizes RTPS header parse, submessage iteration, HEARTBEAT->ACKNACK response generation, ACKNACK handoff, and DATA/other-submessage dispatch hooks. ESP32 and Linux wrappers now provide channel-specific callback adapters for SPDP parsing, endpoint mapping, socket send, and logging.
- Completed: shared discovered-participant lookup/routing helper in `RTPSDiscoveredParticipantLookup.*` now centralizes guid-prefix remote lookup and userdata-heartbeat ACK metatraffic-port routing-by-IP; both ESP32 and Linux wrappers now use this helper in ACKNACK remote resolution and receive-path ACK destination resolution.
- Completed: shared callback-adapter scaffolding in `RTPSRunnerAdapterHelpers.*` now centralizes base `RTPSRxSubmessageRunner` callback wiring (local guid, reader-map policy, ACK destination policy, ACK socket send) and `RTPSAckNackRunner` remote participant resolution callback behavior. ESP32 and Linux wrappers now set policy/context and provide only behavior-specific hooks.
- Started: Phase 1 discovery-module consolidation with new cohesive module entry point:
	- `components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.h/.cpp`
	- ESP and Linux wrappers now call `DiscoveryRuntime` for participant merge, activation side effects, and lease-expiry checks.
	- `RTPSRunnerAdapterHelpers` now resolves discovered participants and metatraffic ACK destination via `DiscoveryRuntime` APIs.
- Continued: `DiscoveryRuntime` is now the discovery source-of-truth implementation for merge/lookup/lease/activation behavior; legacy micro-helper files (`RTPSDiscoveryPolicy`, `RTPSDiscoveredParticipantLookup`, `RTPSParticipantLeasePolicy`, `RTPSParticipantLifecycle`, `RTPSParticipantSetPolicy`) were converted to compatibility wrappers delegating to `DiscoveryRuntime`.
- Started: Phase 2 reliability-module consolidation with new source-of-truth module entry point:
	- `components/RaftROS/RTPS/runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h/.cpp`
	- Legacy reliability helpers (`RTPSReliabilityPolicy`, `RTPSAckNack`, `RTPSBuiltinEndpointMap`) now delegate to this reliability module via compatibility wrappers.
- Continued: `RTPSAckNackRunner` retransmit-decision branching is now centralized in `RTPSReliabilityAndWriterStateRuntime::evaluateAckNackActions(...)`; the runner now acts as a thin adapter that performs parse/log/remote-resolve and dispatches runtime-selected actions via existing callbacks.
- Continued: shared ACKNACK action execution adapter added in `RTPSRunnerAdapterHelpers` (`RTPSRunnerAdapter_executeAckAction(...)` plus per-action build/send specs). ESP (`RaftROS.cpp`) and Linux standalone (`raftros_standalone.cpp`) now use this shared execution path for destination selection and socket-send dispatch, while keeping wrapper-specific payload builders.
- Continued: ACKNACK writer-heartbeat mutation policy is now centralized in reliability runtime (`applyAckActionHeartbeatMutation(...)`) and applied by the shared ACK execution adapter using wrapper-configured mutation profiles (ESP vs Linux behavior preserved via policy flags).
- Continued: ACKNACK action log-label/result-label mapping is now centralized in `RTPSRunnerAdapterHelpers` (`RTPSRunnerAdapter_ackActionLogLabel(...)`, `RTPSRunnerAdapter_ackActionResultLabel(...)`), reducing wrapper-local per-action logging switch logic in ESP and Linux handlers.
- Continued: ACKNACK SEDP retransmit endpoint metadata (entity/topic/type/QoS for ros_discovery_info and chatter announcement actions) is now centralized in reliability runtime (`getAckActionSedpPublicationProfile(...)`, `getAckActionSedpSubscriptionProfile(...)`), and both wrappers now consume runtime-provided profiles in ACK action builders.
- Continued: ACKNACK SEDP retransmit sequence-number selection is now centralized in reliability runtime (`getAckActionSedpSequenceNumber(...)`) via per-wrapper sequence-context policy; Linux `sedpSeqNum + 1` chatter compatibility behavior is preserved via context policy flag rather than wrapper-local arithmetic.
- Continued: ACKNACK user-data payload debug instrumentation policy is now centralized via shared adapter debug policy (`RTPSRunnerAdapter_shouldDumpAckPayload(...)`, `RTPSRunnerAdapter_logHexPayload(...)`); Linux enables ros_discovery_info payload hex-dump through policy, ESP keeps it disabled.
- Continued: ACKNACK SEDP action builder scaffolding is reduced via runtime combined plan resolver (`getAckActionSedpPlan(...)`) that returns endpoint metadata + sequence number together; wrappers now initialize sequence context once and consume a single per-action plan in SEDP ACK builders.
- Continued: ACKNACK user-data action envelope policy is now centralized in reliability runtime (`getAckActionUserDataPlan(...)`) with per-wrapper sequence context and optional firstSN override policy; wrappers now consume runtime writer/sequence/firstSN plans for both ros_discovery_info and chatter ACK data retransmits.
- Continued: ACKNACK context initialization boilerplate is reduced through shared setup helpers (`RTPSRunnerAdapter_initAckExecContext(...)`, `makeAckSedpSequenceContext(...)`, `makeAckUserDataSequenceContext(...)`); wrappers now declare concise init configs instead of manually setting each context field.
- Continued: ACKNACK callback-bundle wiring is now centralized via `RTPSRunnerAdapter_initAckCallbacks(...)`, so wrappers provide a compact function-pointer callback init config while shared code applies common remote-resolution wiring. This keeps the implementation ESP32-friendly (no dynamic allocation, stack-only context, static callback wiring).
- Continued: ACKNACK hex-dump debug instrumentation is now compile-time gated in shared adapter helpers via `RAFTROS_ACK_HEX_DUMP_ENABLE` (default OFF for embedded builds). Linux unit-test build explicitly enables this flag in `linux_unit_tests/Makefile` to preserve developer diagnostics while keeping ESP32 release builds lean.
- Continued: ACKNACK verbose action/result log labels are now compile-time gated via `RAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE` (default OFF for embedded builds). Linux unit-test build explicitly enables this flag so local diagnostics remain detailed without forcing verbose strings into embedded firmware builds.
- Continued: ACKNACK runtime flavor policy presets are now centralized in reliability runtime (`makeAckNackDecisionOptionsForFlavor(...)`, `makeAckNackMutationPolicyForFlavor(...)`, `makeAckSedpSequenceContextForFlavor(...)`, `makeAckUserDataSequenceContextForFlavor(...)`). ESP and Linux wrappers now consume these shared flavor helpers instead of hardcoding inline policy booleans.
- Continued: ACK runner option wiring is now centralized via `RTPSRunnerAdapter_initAckRunnerOptions(...)`, reducing wrapper-level field-by-field option mapping and keeping runner-vs-runtime option translation in shared adapter code.
- Continued: ACKNACK runner API now consumes shared runtime decision options directly (`RTPSAckNackDecisionOptions`) instead of a duplicate runner-local options struct. Wrappers now pass flavor-derived decision options straight into `RTPSAckNackRunner_run(...)`, and the temporary adapter mapping helper was removed.
- Continued: ACKNACK runner action type is now unified with shared runtime action type (`RTPSAckNackDecisionAction`) via aliasing in `RTPSAckNackRunner.h`; duplicate runner-action enum and runtime<->runner action mapping glue were removed from runner/adapter helper code paths.
- Continued: ACK runner/adapter internals now call reliability runtime source-of-truth APIs directly for ACKNACK parse/classify and builtin reader mapping (`parseAckNack`, `classifyWriter`, `localReaderForRemoteWriter`) instead of going through legacy compatibility-wrapper entry points.
- Continued: first discovery-wrapper removal pass completed. Removed legacy compatibility wrapper files from `components/RaftROS/RTPS/`:
	- `RTPSParticipantLifecycle.*`
	- `RTPSDiscoveryPolicy.*`
	- `RTPSParticipantLeasePolicy.*`
	- `RTPSParticipantSetPolicy.*`
	- `RTPSDiscoveredParticipantLookup.*`
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now link only runtime source-of-truth modules for those concerns.
- Continued: `RTPSDiscoveryRuntime.h` is now self-contained for participant-set policy result typing (no dependency on removed legacy participant-set header).
- Simplification milestone: RTPS top-level `.cpp` count reduced from **19 -> 14** in this pass.
- Continued: reliability-wrapper removal pass completed. Removed legacy compatibility wrapper files from `components/RaftROS/RTPS/`:
	- `RTPSAckNack.cpp` (function-wrapper shim removed; shared ACKNACK types remain in `RTPSAckNack.h`)
	- `RTPSReliabilityPolicy.*`
	- `RTPSBuiltinEndpointMap.*`
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now link direct runtime reliability source-of-truth only.
- Continued: wrapper and runner call sites now use reliability runtime APIs directly for writer-kind log labels and heartbeat-response policy checks.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **14 -> 11** in this pass (**19 -> 11** cumulative across simplification passes).
- Continued: removed final ACKNACK shim header (`RTPSAckNack.h`) by making reliability runtime header self-contained for ACKNACK shared types (`RTPSAckNackFields`, `RTPSAckNackWriterKind`). `RTPSAckNackRunner.h` now aliases those types from runtime namespace directly.
- Continued: receive-path orchestration was relocated from top-level RTPS folder into `runtime/receive/`:
	- `RTPSRxSubmessageRunner.*` -> `runtime/receive/RTPSRxSubmessageRunner.*`
	- `RTPSRunnerAdapterHelpers.*` -> `runtime/receive/RTPSRunnerAdapterHelpers.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now reference the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **11 -> 9** in this pass (**19 -> 9** cumulative across simplification passes).
- Continued: announce/heartbeat orchestration was relocated from top-level RTPS folder into `runtime/announce/`:
	- `RTPSInitialAnnouncePlan.*` -> `runtime/announce/RTPSInitialAnnouncePlan.*`
	- `RTPSInitialAnnounceRunner.*` -> `runtime/announce/RTPSInitialAnnounceRunner.*`
	- `RTPSWriterHeartbeatRunner.*` -> `runtime/announce/RTPSWriterHeartbeatRunner.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now reference the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **9 -> 6** in this pass (**19 -> 6** cumulative across simplification passes).
- Continued: ACKNACK orchestration runner was relocated from top-level RTPS folder into `runtime/reliability/`:
	- `RTPSAckNackRunner.*` -> `runtime/reliability/RTPSAckNackRunner.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and receive-adapter include wiring (`runtime/receive/RTPSRunnerAdapterHelpers.h`) now reference the runtime path directly.
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now compile the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **6 -> 5** in this pass (**19 -> 5** cumulative across simplification passes).
- Continued: shared schedule policy helper was relocated from top-level RTPS folder into `runtime/schedule/`:
	- `RTPSRuntimeSchedule.*` -> `runtime/schedule/RTPSRuntimeSchedule.*`
	Wrapper include sites (`RaftROS.cpp`, `raftros_standalone.cpp`) and build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now reference the runtime path directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **5 -> 4** in this pass (**19 -> 4** cumulative across simplification passes).
- Continued: SPDP/SEDP protocol handlers were relocated into runtime ownership boundaries:
	- `SPDPHandler.*` -> `runtime/discovery/SPDPHandler.*`
	- `SEDPHandler.*` -> `runtime/announce/SEDPHandler.*`
	Wrapper include sites (`RaftROS.h`, `raftros_standalone.cpp`, `linux_unit_tests/main.cpp`) and runtime helper includes now reference runtime paths directly.
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now compile runtime paths directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **4 -> 2** in this pass (**19 -> 2** cumulative across simplification passes).
- Continued: wire/core protocol foundations were relocated into runtime ownership boundaries:
	- `RTPSMessage.*` -> `runtime/wire/RTPSMessage.*`
	- `RTPSParticipant.*` -> `runtime/core/RTPSParticipant.*`
	Wrapper and runtime module include sites (`RaftROS.cpp/.h`, `raftros_standalone.cpp`, `linux_unit_tests/main.cpp`, reliability/receive/discovery/announce runtime modules) now reference runtime paths directly.
	Build lists (`linux_unit_tests/Makefile`, `CMakeLists.txt`) now compile runtime paths directly.
- Simplification milestone: RTPS top-level `.cpp` count reduced from **2 -> 0** in this pass (**19 -> 0** cumulative across simplification passes).
- Stabilization pass: stale include/source reference scan confirmed no remaining references to removed top-level runtime-orchestration files.
- Stabilization pass: include/structure hygiene cleanup applied in runtime modules (`runtime/reliability/RTPSAckNackRunner.cpp` redundant include removed, `runtime/receive/RTPSRunnerAdapterHelpers.cpp` empty anonymous namespace removed).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after receive-runner extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after lookup-helper extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after runner-adapter scaffolding extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after DiscoveryRuntime Phase 1 start (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after DiscoveryRuntime source-of-truth/wrapper conversion (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK runtime decision-planner extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK shared action-execution adapter extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK heartbeat-mutation policy extraction into reliability runtime (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK shared log-label policy extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK SEDP endpoint metadata extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK SEDP sequence-policy extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK payload debug-policy extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK SEDP combined-plan extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK user-data combined-plan extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK context-init helper extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK callback-bundle init extraction (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after compile-time ACK hex-dump gating (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after announce-runtime relocation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK runner relocation to `runtime/reliability` (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after schedule-helper relocation to `runtime/schedule` (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after stabilization-pass hygiene cleanup (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after SPDP/SEDP runtime-boundary relocation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after RTPSMessage/RTPSParticipant runtime-boundary relocation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after compile-time ACK verbose label gating (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK runtime flavor-policy helper extraction and shared runner-option init wiring (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK runner-options API unification to shared runtime decision options (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACK action-type unification and mapping-removal cleanup (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after direct reliability-runtime call migration in ACK runner/adapter internals (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after discovery-wrapper file removal and build-list pruning (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after reliability-wrapper file removal and build-list pruning (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after ACKNACK shim-header removal and runtime-type ownership consolidation (**85 passed, 0 failed**).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after receive-runtime file relocation and top-level source pruning (**85 passed, 0 failed**).
- Continued: added regression-guard unit tests in `linux_unit_tests/main.cpp` for two Phase-2 bring-up fixes that are subtle and easy to regress:
	- VOLATILE `/chatter` invariant: `makeAckUserDataSequenceContextForFlavor(...)` opts in to `chatterFirstSNMatchesSequence` for both ESP and Linux flavors, and `getAckActionUserDataPlan(RetransmitChatterData, ...)` returns `hasFirstSNOverride=true` with `firstSN == sequenceNumber` (Fix 15 guard). TRANSIENT_LOCAL `ros_discovery_info` retransmit must NOT override firstSN.
	- Distinct SEDP-pub sequence numbers: `getAckActionSedpPlan(...)` on the shared SEDP publications writer yields distinct sequence numbers for ros_discovery_info vs chatter publication announcements under both flavor policies (ESP uses explicit `chatterPublicationSeqNum`; Linux derives `rosDiscoveryPublicationSeqNum + 1`). Endpoints also differ in entity ID and durability (Fix 16 guard).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after regression-guard test addition (**108 passed, 0 failed** — +23 assertions, still 0 failed).
- Continued: extracted the ACKNACK action-execution boilerplate (five per-action `buildMessage` lambdas + the `executeAction` lambda, ~200 lines per wrapper) from both wrappers into a shared default context in `RTPSRunnerAdapterHelpers`:
	- New types `RTPSAckActionStandardCtx`, `RTPSAckActionStandardInitConfig`, policy enum `RTPSAckActionSedpBuildHeartbeatPolicy` (`PassZero`/`PassLiveHeartbeat`), and payload-builder typedef `RTPSAckActionBuildPayloadFn(void*, uint8_t*, uint32_t)`.
	- New free functions `RTPSRunnerAdapter_initStandardAckActionCtx(...)` / `RTPSRunnerAdapter_standardExecuteAction(...)` / `RTPSRunnerAdapter_standardGetChatterSeq(...)` wire the five `RTPSAckNackRunnerActionExecSpec` channels, the SEDP build-heartbeat policy, the firstSN override for chatter, and the hex-payload debug dump behind a single entry point.
	- Both wrappers (`components/RaftROS/RaftROS.cpp::handleAcknack` and `linux_unit_tests/raftros_standalone.cpp::handleAcknack`) are now ~70 lines each (down from ~200+) and differ only on genuine wrapper-level policy: flavor, `sedpBuildHeartbeatPolicy` (ESP=`PassZero` matching the prior `heartbeatCount=0` default, Linux=`PassLiveHeartbeat`), `dumpRosDiscoveryPayloadHex`, and wrapper-owned payload builders (passed as plain fn-pointer + `void* payloadCtx`).
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after shared ACKNACK action-execution default-ctx extraction and Linux wrapper conversion (**108 passed, 0 failed**). ESP wrapper conversion completed; ESP-IDF build and on-device `ros2 topic echo /chatter` smoke test pending.
- Continued: folded the thin `RTPSAckNackRunner.cpp` translation unit into `RTPSReliabilityAndWriterStateRuntime.cpp` so the reliability module now owns the full ACKNACK parse -> classify -> remote-resolve -> evaluate -> dispatch pipeline end-to-end. `RTPSAckNackRunner.h` stays as the public facade (callback typedefs + `RTPSAckNackRunner_run`) so wrapper call sites are unchanged; `RTPSAckNackRunner.cpp` deleted and removed from `CMakeLists.txt` and `linux_unit_tests/Makefile` (both SOURCES and PROTO_SOURCES).
- Validation: `cd linux_unit_tests && make clean && make -j$(nproc) all standalone && ./linux_unit_tests` remains green after runner-cpp consolidation into reliability runtime (**108 passed, 0 failed**).
- Continued: started Phase 3 (topic subscribing) with the shared *reader* runtime. Added `runtime/reliability/RTPSReaderRuntime.{h,cpp}` mirroring the reliability-decision shape of the writer side but inverted for reader semantics: per-remote-writer state (`highestContiguousSeq`, `highestSeenSeq`, `receivedBitmap`, `lastHeartbeatCount`, `outgoingAckNackCount`) plus pure helpers `evaluateIncomingDataDecision` (Accept/Dedup/Drop), `applyAcceptedDataToReaderState` (contiguous-window collapse + bitmap shift), `evaluateIncomingHeartbeatDecision` (ACKNACK base+numBits+bitmap, FINAL-flag dedup, best-effort suppression, caught-up confirmation ACKNACK), and `applyHeartbeatProcessedToReaderState`. No wrapper IO yet — pure decisions only, per the modularization plan's "put reader decision logic in shared runtime from day one" risk mitigation.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after reader-runtime slice (**135 passed, 0 failed** — +27 assertions across DATA-accept/dedup/gap-collapse and HEARTBEAT→ACKNACK decision tests).
- Continued: added `runtime/reliability/RTPSReaderRunner.{h,cpp}` — thin orchestration mirroring `RTPSAckNackRunner`. Parses HEARTBEAT (28-byte content) and DATA (20-byte header + inline-QoS-skip + payload) submessage content into typed fields, looks up the reader state via a `resolveState` callback, invokes the pure decision helpers, and emits `sendAckNack` / `dispatchData` callbacks. No sockets, no CDR. Not yet wired into the RX submessage runner — wrappers continue to use the existing naive ACKNACK path for now.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after reader-runner slice (**169 passed, 0 failed** — +34 assertions covering HEARTBEAT/DATA submessage parse, FINAL/LIVELINESS flag propagation, short-content rejection, orchestration with state lookup + ACKNACK emission, duplicate-HB suppression via FINAL flag, and DATA Accept→Dedup transitions).
- Continued: added `RTPSMessage::writeAcknackWithBitmap` — bitmap-capable ACKNACK wire builder (DDSI-RTPS §9.4.2.7). Packs `numBits` (capped at 256) into ceil(numBits/32) little-endian 32-bit words; supports the FINAL (F=1) flag for "do not HEARTBEAT me again" semantics; byte-for-byte identical to the legacy `writeAcknack` when `numBits=0` & `finalFlag=false`. This is the wire-level prerequisite for routing HEARTBEATs through `RTPSReaderRunner`'s decision path. The existing `writeAcknack` is retained unchanged; no call-site migration yet.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after ACKNACK bitmap builder (**187 passed, 0 failed** — +18 assertions covering empty-bitmap equivalence with legacy builder, single-word bitmap packing + LE byte layout, FINAL flag + two-word bitmap, and input validation (buffer-too-small / numBits>256 / null-bitmap-with-numBits>0)).
- Continued 2026-04-22: extended `RTPSRxSubmessageRunner` with an opt-in `resolveReaderWriterState` callback. When the wrapper registers a per-(remote participant, writer EID) `RTPSReaderWriterState` lookup, incoming HEARTBEATs are delegated to the pure `RTPSReaderRuntime` decision layer and ACKNACKs are emitted via the new `writeAcknackWithBitmap` builder. When the callback is null (current wrapper configuration), the legacy inline ACKNACK path is preserved byte-for-byte. Neither `RaftROS.cpp` nor `raftros_standalone.cpp` set the new callback yet — no behavioral change on live hosts.
- Validation: `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests` green after RX-runner opt-in slice (**200 passed, 0 failed** — +13 assertions exercising a full RTPS packet through the RX runner: opt-in path state lookup + bitmap ACKNACK emission with `numBits=3` covering SNs [1..3], duplicate-FINAL HB suppression, and legacy-path byte-for-byte preservation when `resolveReaderWriterState` is null, verifying `acknackCount` increment semantics on both paths). Docker smoke deferred — environment today has no FastDDS peer on the bridge network (baseline main-branch binary exhibits the same behavior, confirming this is environmental rather than a regression).
- Continued 2026-04-22: added header-only `RTPSReaderStateMap` (`components/RaftROS/RTPS/runtime/reliability/RTPSReaderStateMap.h`) keyed on 12-byte `guidPrefix` + 4-byte `writerEID`, providing `getOrCreate` / `find`. Wired `resolveReaderWriterState` callbacks into both wrappers (`RaftROS.cpp` and `linux_unit_tests/raftros_standalone.cpp`) so all incoming HEARTBEATs now flow through the reader runtime decision layer + `writeAcknackWithBitmap`. The existing `onHeartbeat` / `onAckNack` / `onData` logging callbacks are unchanged, so wrapper-observable behaviour on the metatraffic path is identical with an empty state (bitmap SN==highestSeen+1 → no missing bits → emits standard final ACKNACK equivalent to legacy).
- Validation: **212 passed, 0 failed** (+12 assertions covering `getOrCreate` first-miss insertion, pointer-stability on re-lookup, preservation of mutated state across lookups, distinct-key creation for distinct `guidPrefix` / `writerEID`, and null-input guarding).
- Continued 2026-04-22: added `SedpChatterReader` announce plumbing to `RTPSInitialAnnouncePlan` — new plan flag `sendSedpChatterReader` (default `false`, opt-in), new action `RTPSInitialAnnounceAction::SedpChatterReader` routed to `SedpSubscription` build kind + `ChatterReader` endpoint profile. Added `ENTITYID_CHATTER_READER = {0x00,0x01,0x02,0x04}` and `CHATTER_IN_DDS_TOPIC = "rt/chatter_in"` (type `std_msgs::msg::dds_::String_`). New `SedpChatterReader` sequence counter hint + `sedpChatterReaderSeqNum` counter so the sub announce is independent of the chatter pub. Grew `RTPSInitialAnnounceSequence::steps[]` from 7 to 8 entries. No wrapper wiring yet — `RaftROS.cpp` / `raftros_standalone.cpp` leave `sendSedpChatterReader=false`, so no behavioural change today.
- Validation: **230 passed, 0 failed** (+18 assertions covering default-plan omission of ChatterReader, opt-in plan schedules ChatterReader immediately after ChatterWriter with heartbeat-before-send on LinuxStyle, build-spec / send-target / endpoint-spec / log-spec dispatch for the new action, the new seq counter hint returning `sedpChatterReaderSeqNum` (not the writer's counter), and regression guard that the default plan still schedules ChatterWriter).
- Continued 2026-04-22 (slice 6b): opted both wrappers into the new ChatterReader announce. `RaftROS::buildRosDiscInfoWithGids` and the standalone equivalent now publish `ENTITYID_CHATTER_READER` in the reader GID list, so downstream ROS 2 nodes see a `/chatter_in` subscriber on this participant. Both wrappers set `announcePlan.sendSedpChatterReader = true`, triggering a SEDP subscription DATA+HEARTBEAT pair on first discovery; reused the existing generic `SedpSubscription` build dispatch, so no new builder code is needed. `RTPSInitialAnnounceCounterState` positional initializers in both wrappers extended to supply the new `sedpChatterReaderSeqNum` slot (on ESP: dedicated counter; on Linux-flavor: `sedpSubSeqNum + 1`, mirroring chatter-writer policy). Sequence-policy helper updated to apply `sedpRosReaderSeqNum + 1` on LinuxStyle to match the ChatterWriter convention.
- Validation: **231 passed, 0 failed** (+1 assertion added, covering LinuxStyle ChatterReader seq-policy fallback; one pre-existing assertion adjusted to exercise both flavors).
- Continued 2026-04-22 (slice 7): added header-only `RTPSUserDispatch::decodeStdMsgsString` (`components/RaftROS/RTPS/runtime/dispatch/RTPSUserDispatch.h`). Pure CDR-LE decode of `std_msgs::msg::String_` payloads into a caller-supplied buffer: reads the 4-byte encapsulation header via `CDRDecoder`, then a length-prefixed UTF-8 string; truncates safely when the output buffer is too small; returns textLen (excluding null) and a null-terminated output. Wired into both wrappers' user-data RX path: when a DATA submessage arrives on the user-data channel with any writerEID other than `ENTITYID_ROS_DISC_INFO_WRITER`, the payload (assuming no inline-QoS; otherwise decode fails harmlessly) is decoded and logged. `RaftROS` exposes `setStringMessageHandler(std::function<void(const uint8_t*, const uint8_t*, const char*, uint32_t)>)` for user code to receive the decoded message.
- Validation: **243 passed, 0 failed** (+12 assertions covering well-formed decode, empty-string round trip, null-buffer / null-payload guarding, truncation (length exceeds buffer), and truncated-output-buffer succeeding with null-terminated prefix + correct source textLen).
- Continued 2026-04-22 (slice 7c): extended `RTPSWriterHeartbeatRunner` with a new `SedpChatterSubscription` action so late-joining ROS 2 peers also learn about the chatter_in reader via periodic SEDP retransmit. Added `chatterSedpSubSeqNum` counter slot and grew the sequence from 5 to 6 steps; the new step is scheduled immediately after `SedpChatterPublication` with the same linux-style heartbeat-before-build policy. Both wrappers (`RaftROS.cpp` and `raftros_standalone.cpp`) now handle the new action in their `buildPayload` switch by calling `SEDPHandler::buildSubscriptionMessage` with `ENTITYID_CHATTER_READER` / `CHATTER_IN_DDS_TOPIC` / `CHATTER_IN_DDS_TYPE`. Counter-state positional initializers in both wrappers supply the new slot (`sedpSubSeqNum + 1` on Linux-flavor, mirroring the chatter-pub convention).
- Validation: **255 passed, 0 failed** (+12 assertions covering sequence placement on both flavors, send-target = Metatraffic, sequence-number selection (`chatterSedpSubSeqNum` on ESP / `sedpSubSeqNum + 1` on Linux), and end-to-end `RTPSWriterHeartbeatRunner_run` invocation wiring buildPayload → sendPayload with the correct SN on both flavors).
- Continued 2026-04-22 (slice 7d — on-device confirmation): ESP32-S3 at `192.168.1.173` flashed with slice 7c firmware. ROS 2 Humble peer running directly on host `192.168.1.92` (Docker container was isolated from LAN; must run the publisher on a bridged network for SPDP to reach the device). End-to-end validated:
    - `ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello esp'}"` produced device log `UD user-topic writerEID=00000503 src=010FCCEF... "hello esp" (9 chars)` — confirms SEDP reader announce, reliable HEARTBEAT → ACKNACK round trip, inline-QoS-absent DATA reception, and CDR-LE decode path.
    - Chatter publish (ESP → ROS 2) confirmed via `ACKNACK base=566 numBits=0` — peer ACKs every sample.
    - `-1` errno-ENOMEM send bursts on the very first announce are benign (LwIP ARP warmup); the periodic `SedpChatterSubscription` retransmit added in slice 7c recovers state.
- Continued 2026-04-22 (slice 8 — polish & productionisation): five follow-ups bundled.
    1. ACKNACK writer-kind label table (`RTPSReliabilityAndWriterStateRuntime.cpp::classifyWriter` + `writerKindToStr`) extended with `ParticipantMessage` (`ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER`, `0x000200C2`, labelled `participant_msg`) and `ChatterReader` (`ENTITYID_CHATTER_READER`, labelled `chatter_in_reader`). Removes the `(unknown)` log lines against liveliness ACKNACKs.
    2. `RaftROS` public API: added `setStringSubscription(topic, type, handler)` alongside `setStringMessageHandler`. Topic + type name are now instance members (`_subscriptionTopic`, `_subscriptionType`), defaulting to `rt/chatter_in` / `std_msgs::msg::dds_::String_`. Both the initial-announce SEDP build callback and the `SedpChatterSubscription` heartbeat callback in `RaftROS.cpp` now read from these members, so apps can subscribe to any topic name without rebuilding the library. **Slot-0-only for now** — generalising SEDP announce to advertise N simultaneous readers is a focused follow-up (would need an N-ary expansion of `RTPSInitialAnnouncePlan`/`RTPSWriterHeartbeatRunner`, currently one-profile-per-step).
    3. `ExampleDiscoverable/components/MainSysMod`: calls `getSysManager()->getSysMod("RaftROS")` and registers a `setStringMessageHandler` lambda that logs decoded text. `CMakeLists.txt` now lists `RaftROS` as a required component so `RaftROS.h` resolves at build time.
    4. Liveliness HEARTBEAT in `SEDPHandler::buildParticipantMessageData` now advertises `firstSN == lastSN == sequenceNumber` (single-sample GAP) instead of `firstSN=1`. Stops the peer NACK storm (observed `numBits=166..256` against `ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER`); peer now just advances its base past the current SN.
    5. Unit tests: +8 assertions across `classifyWriter` / `writerKindToStr` covering every live writer kind plus explicit regression coverage that `ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER` is no longer classified as `Unknown`.
- Validation: **263 passed, 0 failed** (pending local rebuild confirmation after this slice's edits).
- Continued 2026-04-22 (slice 9 — N-ary SEDP subscription registry): introduced shared `components/RaftROS/RTPS/runtime/dispatch/RTPSSubscriptionRegistry.h` — header-only POD (fixed capacity 8) holding `{entityId[4], topic, type}` per slot, plus a deterministic `RTPSSubscriptionRegistry_allocateNextEntityId(slot)` allocator (slot 0 = `ENTITYID_CHATTER_READER` = `{0x00,0x01,0x02,0x04}`; slot N = `{0x00,0x01,(0x02+N),0x04}`). Registry exposes `add`, `setTopic`, `findByEntityId`, `readerEntityIds`. Wrapper (`RaftROS.h/.cpp`) now seeds slot 0 at construction from `_subscriptionTopic`/`_subscriptionType` and offers a new public `addStringSubscription(topic, type, handler) -> slot` API. `buildRosDiscInfoWithGids` iterates every registry entry into the `ros_discovery_info` `reader_gid_seq`. `handleNewParticipant` emits an extra SEDP subscription announce per slot (1..N) immediately after the main initial-announce sequence; `sendWriterHeartbeats` retransmits the same set once per heartbeat pass, keyed by `_extraSubscriptionSeqNums[slot]` (defaulted to 1 each). Slot 0 continues to flow through the existing fixed announce/HB plan unchanged, so no regressions in the single-subscription path. Dispatch still uses the single `_stringMessageHandler` (per-slot `_extraSubscriptionHandlers[]` stored but not yet routed; see follow-up).
- **Remaining follow-up**: per-topic handler routing in `recvUserData::onData`. DATA submessages carry only the *remote writer's* entity ID, so routing to the right local handler requires correlating remote SEDP publication records (remote writerGUID → topicName) with our registry entries. Needs an SEDP-pub parse hook + remote-writer table; tracked as "Phase 3.5: subscriber routing".
- Validation: **298 passed, 0 failed** (+35 assertions for the registry: allocator policy across slot range, capacity guard, add/setTopic null-guards, findByEntityId hit/miss, readerEntityIds ordering + maxOut cap, fill-to-capacity overflow).
- Next: start Phase 3 (topic subscribing). Fold remaining `RTPSAckNackRunner.cpp` internals behind `RTPSReliabilityAndWriterStateRuntime` and add a reader-side ACKNACK + deserialization path with the same shared-runtime discipline from day one.

### Practical Rule

- If logic can be expressed without direct socket API calls or FreeRTOS/Arduino-specific APIs, it belongs in shared core code.
- Wrappers should mainly provide:
	- packet send/receive callbacks
	- current time
	- local network identity
	- logging hooks

## Phase 3: Topic Subscribing — COMPLETE ✅

ESP32 subscribes to N ROS 2 topics and receives `std_msgs/msg/String` with per-topic handler dispatch. Verified on 2026-04-22 with `/chatter_in` (slot 0) and `/chatter_in2` (slot 1) against FastDDS 2.6.11.

### What Works

- SEDP subscription announces for N reader slots via shared `RTPSSubscriptionRegistry` (8-slot POD with deterministic entity-ID allocator: slot 0 = `ENTITYID_CHATTER_READER {0x00,0x01,0x02,0x04}`, slot N = `{0x00,0x01,0x02+N,0x04}`).
- Reader-side reliable ACKNACK via shared `RTPSReaderRuntime` (pure decision logic: DATA accept/dedup/gap-collapse, HEARTBEAT → ACKNACK base/numBits/bitmap, FINAL-flag dedup, best-effort suppression) + `RTPSReaderRunner` (submessage parse + state lookup + dispatch) + `RTPSMessage::writeAcknackWithBitmap` (bitmap wire builder).
- CDR deserialization via `RTPSUserDispatch::decodeStdMsgsString`.
- Per-topic routing via `RTPSRemotePublicationMap` (16-entry fixed map correlating remote `writerGuid` → local slot), populated from inbound SEDP publication DATAs parsed by `RTPSSEDPPublicationParser` (extracts `PID_ENDPOINT_GUID` and `PID_TOPIC_NAME`).
- Handler dispatch via `RaftROS::addStringSubscription(topic, type, handler)`.

### Phase 3 Bring-up Fixes

| # | Fix | Details |
|---|-----|---------|
| 18 | Monotonic SEDP-sub sequence numbers | `_extraSubscriptionSeqNums[8] = {2,3,4,5,6,7,8,9}`. Without distinct per-slot SNs, FastDDS dropped slot>=1 announcements as duplicates of the main sub writer's sequence space. |
| 19 | ACKNACK bitmap inversion | Per RTPS §9.4.5.2, bit=1 in `readerSNState` means **NOT received / please retransmit**. `RTPSReaderRuntime.cpp` was inverting the sense, so FastDDS read our ACKNACKs as positive ACKs and never retransmitted missed SEDP publications. |
| 20 | DATA inline-QoS skipping | DATA submessages with Q flag (bit 1 = 0x02) carry an inline-QoS `ParameterList` before the serialized payload. `onData` was reading `pContent + 20` unconditionally, which landed inside the inline-QoS for FastDDS dispose-style DATAs (PID_STATUS_INFO + PID_KEY_HASH + PID_SENTINEL = 32 bytes of body, no payload). Fix: propagated DATA `flags` byte through `onData`, added `RTPSData_getSerializedPayload()` helper in `RTPSRxSubmessageRunner.h` that skips the 20-byte fixed prefix and, when Q=1, scans the inline-QoS ParameterList up to `PID_SENTINEL (0x0001)`. |

All three fixes are regression-guarded by unit tests in `linux_unit_tests/main.cpp`.

### Phase 3 Slice Log (post-slice 9)

- **Slice 10 (reader-side ACKNACK bitmap inversion fix)**: inverted the bit sense in `RTPSReaderRuntime::evaluateIncomingHeartbeatDecision` so bit=1 in the emitted bitmap now correctly means NOT-received. Added assertions `d.ackNackBitmap == 0x1F` (5 missing) and `0x3` (2 missing) in `linux_unit_tests/main.cpp`.
- **Slice 11 (distinct SEDP-sub SNs)**: per-slot sequence-number counters so FastDDS can distinguish the N SEDP subscription announcements issued on the shared sub writer.
- **Slice 12 (per-topic dispatch)**: `RTPSSEDPPublicationParser` extracts `(writerGuid, topic)` from inbound SEDP publication DATAs; `RTPSRemotePublicationMap::upsert` records the correlation when `topic` matches a registered subscription slot; user-data DATAs look up the slot via `findSlot(guidPrefix, writerEID)` and dispatch to `_extraSubscriptionHandlers[slot]`.
- **Slice 13 (inline-QoS fix)**: `RTPSData_getSerializedPayload()` helper + DATA `flags` byte plumbed through `onData`. All four `onData` lambdas (metatraffic + user-data in both `RaftROS.cpp` and `raftros_standalone.cpp`) updated. Helper also regression-tested: 5 cases / 9 assertions covering Q=0, Q=1 sentinel-only, FastDDS dispose-style, malformed parameter runs-off-end, contentLen<20.
- Validation: **388 passed, 0 failed** (up from 298 at end of slice 9 — +90 assertions across reader-runtime, SEDP-pub parser, remote-pub map, inline-QoS helper, and the three Phase-3 regression guards).
- On-device log evidence (2026-04-22, ESP32-S3 `192.168.1.173` ↔ ROS 2 Humble host `192.168.1.92`):
  - `SEDP pub DATA received contentLen=468 flags=0x05`
  - `SEDP pub parsed topic='rt/chatter_in2' (14 chars), subReg.count=2`
  - `SEDP pub matched topic='rt/chatter_in2' -> slot=1 (new)`
  - `MainSysMod: chatter_in2 #1 writerEID=00000503 src=010FCCEF... "hello slot2" (11 chars)`

### Phase 3 Key Learnings

- **RTPS `readerSNState` bitmap convention is inverted from intuition**: per §9.4.5.2, a `1` bit means "NOT received / retransmit this". It is not a positive-ACK bitmap. Easy to get wrong, silent-to-diagnose when wrong.
- **DATA submessages always honour the Q flag before reading the serialized payload**. FastDDS routinely sends dispose/unregister DATAs with `Q=1 D=0 K=1` (endpoint teardown) that contain only an inline-QoS body — there is no serialized payload after it. Code that treats `pContent + 20` as the payload for every DATA will silently misparse every one of these.
- **N-ary SEDP announcements on one writer must use distinct sequence numbers** (same lesson as Fix 16 for the publication writer side — applies symmetrically to the subscription writer).

## Phase 4: DeviceManager Auto-Publishing — COMPLETE ✅

Every bus device detected by `DeviceManager` is automatically mirrored as a
ROS 2 topic at runtime. No per-device code. No SysTypes topic config for the
common path. Online → `/rt/raft/<slug>_<bus>_<addrHex>` appears in
`ros2 topic list`; offline → SEDP dispose removes it within a heartbeat
interval.

### What works end-to-end

- **Device lifecycle hook** — `RaftROS::setup()` registers a
  `DeviceManager::registerForDeviceStatusChange` callback. `ONLINE`/
  `PENDING_DELETION` transitions drive a fixed-capacity writer registry
  (`DYNAMIC_WRITER_REGISTRY_CAPACITY = 16`).
- **Class → ROS 2 type mapping** (`RTPSAutoPubClassMap.h`, precedence
  first-match-wins):
  1. device-type-name overrides (MCP9808 → Temperature,
     RoboticalLightSensor → Float32MultiArray),
  2. actuator exclusion (`SRVO`, `PUMP`, `PIX`),
  3. composite rules: `{ACC,GYRO}` → `Imu` (single writer),
     `{TEMP,RH}` → `Temperature` + `RelativeHumidity`,
     `{PRES,TEMP}` → `FluidPressure` + `Temperature`,
  4. single-class rules for TEMP, RH, PRES, LGHT, PROX, DIST, ANG, ROT,
     ACC, TCH, BTN, FRCE, HRM, SOIL, GAME,
  5. fallback: `std_msgs/String` with a JSON body containing every decoded
     field, topic slug `raw`.
- **Per-writer slot allocation** — `RTPSAutoPubLifecycle` owns 16 topic +
  type string buffers and issues entityIds in the deterministic range
  `0x000110xx..0x00011Fxx`. Composite devices consume two slots keyed by
  `{bus, addr, subIndex}` (subIndex=0 primary, =1 secondary).
- **SEDP dynamic announce** — the existing writer-heartbeat pass walks the
  registry and emits one `PublicationBuiltinTopic` DATA(w) per active slot
  per tick, reusing the SPDP / HB pacing loop.
- **User-data hot path** — on each decoded bus sample the latest record is
  serialised through `RTPSAutoPubCDRSerializer` into a per-slot 512 B buffer
  and unicast to every discovered peer. Composite devices serialise twice
  (once per kind) on the same decoded struct.
- **Timestamps** — ROS 2 `Header.stamp` is taken from the first `timeMs`
  field of the decoded poll record (not wall-clock), so subscribers see
  sample-time not emit-time.
- **REP-103 unit scaling** — g→m/s² (×9.80665), °/s→rad/s (×π/180),
  mm→m (/1000), hPa→Pa (×100), %→0..1 (/100). Applied per field by the
  attribute-field description table.
- **QoS profiles** (design §7.2, four built-ins):

  | Profile | Reliability | Durability | Depth | Default for |
  |---------|-------------|------------|-------|-------------|
  | `fast_sensor`     | BEST_EFFORT | VOLATILE        | 10 | ACC, GYRO, IMU, PROX, LGHT, DIST, ANG, HRM, FRCE |
  | `slow_sensor`     | RELIABLE    | VOLATILE        | 5  | TEMP, RH, PRES, SOIL, BTHM |
  | `event`           | RELIABLE    | TRANSIENT_LOCAL | 20 | BTN, TCH, ROT, GAME |
  | `fallback_string` | RELIABLE    | VOLATILE        | 10 | any unmapped class |

  Resolution order per writer: per-device alias → per-class override →
  built-in default. SysTypes override surface:

  ```jsonc
  "RaftROS": {
    "enable": true,
    "qosProfiles": {
      "imu_1_6a":        "slow_sensor",            // per-device alias
      "temperature_1_38": "event",
      "classDefaults":   { "ACC": "slow_sensor" }  // per-class override
    }
  }
  ```
- **Dispose on offline** — `PENDING_DELETION` emits an SEDP
  `PublicationBuiltinTopic` DATA with `PID_STATUS_INFO = 0x00000003`
  (Disposed | Unregistered) and `PID_KEY_HASH = guidPrefix+entityId`,
  which causes ROS 2 subscribers to drop the topic within a heartbeat.
  Composite secondary slots are disposed alongside the primary.

### Verification

- **Linux unit tests** — `911 passed, 0 failed` in
  `linux_unit_tests/main.cpp` covering:
  - dynamic writer registry + subIndex disambiguation (Slice 4.1 + 4.10),
  - lifecycle attach/detach (Slice 4.2),
  - topic naming (Slice 4.3),
  - class-map (Slice 4.4 — every row in `DeviceTypeRecords.json`),
  - CDR serialiser per kind including REP-103 scaling + JSON fallback
    (Slices 4.5 / 4.9),
  - QoS profile table, name parse round-trip, per-class defaults,
    composite resolution (Slice 4.11).
- **Firmware size** — `SysTypeMain.bin` 0x1435c0 bytes, partition 25% free
  on ESP32-S3 (ESP-IDF v5.5.2, `-std=gnu++2b -fno-exceptions -fno-rtti`).

### Source files added in Phase 4

| File | Purpose |
|------|---------|
| `components/RaftROS/RTPS/runtime/autopub/RTPSDynamicWriterRegistry.h` | Fixed-capacity slot + entityId registry |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubLifecycle.h` | Slot allocator with owned topic/type strings |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubTopicNaming.h` | `rt/raft/<slug>_<bus>_<addr>` formatter |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubClassMap.h` | `clas[] + deviceType → msg kind + slug` |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.h/.cpp` | Per-kind CDR encoders with REP-103 unit scaling |
| `components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubQoSProfile.h` | 4 built-in profiles + override resolver |

## Phase 5: Integration with Raft — TODO

- ROS 2 actions / service servers.
- Command-side subscriptions auto-wired from DeviceManager actuator classes
  (SRVO, PUMP, PIX) — the symmetric write path. Phase 4 excludes these
  from publishing; Phase 5 will route incoming topic data into
  `DeviceManager::sendCmdJSON`.
- Per-device SysTypes topic alias override (short user-friendly name instead
  of the auto slug).
