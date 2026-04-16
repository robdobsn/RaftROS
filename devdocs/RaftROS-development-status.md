# RaftROS Development Status

**Last Updated:** 2026-04-16

## Goal

Make an ESP32-S3 (running Raft firmware) appear as a native ROS 2 node — discoverable via `ros2 node list` as `/raft_esp32` — without micro-ROS or any agent process. This is Phase 1: discovery only, no topic data exchange yet.

## Architecture

Clean-room RTPS 2.2 implementation as a Raft SysMod. No third-party DDS libraries.

### Source Files

| File | Purpose |
|------|---------|
| `components/RaftROS/RaftROS.cpp/.h` | SysMod lifecycle, UDP sockets, receive/send loops, participant state machine |
| `components/RaftROS/CDR/CDREncoder.cpp/.h` | CDR (Common Data Representation) serializer |
| `components/RaftROS/CDR/CDRDecoder.cpp/.h` | CDR deserializer |
| `components/RaftROS/RTPS/RTPSTypes.h` | Constants: entity IDs, PID codes, port formulas, QoS values |
| `components/RaftROS/RTPS/RTPSMessage.cpp/.h` | RTPS header/submessage read/write (DATA, HEARTBEAT, ACKNACK, INFO_DST, INFO_TS) |
| `components/RaftROS/RTPS/RTPSParticipant.cpp/.h` | GUID management, port calculation, locator building |
| `components/RaftROS/RTPS/SPDPHandler.cpp/.h` | SPDP announcement build/parse, ros_discovery_info CDR payload |
| `components/RaftROS/RTPS/SEDPHandler.cpp/.h` | SEDP publication message build, user data (rosDisc) message build |
| `linux_unit_tests/` | Linux-hosted unit tests (84/85 passing) |
| `unit_tests/` | ESP-IDF Unity tests |
| `examples/ExampleDiscoverable/` | ESP32 app that runs RaftROS as a SysMod |

### Hardware / Environment

- **ESP32-S3**, MAC `94:a9:90:3a:51:b0`, WiFi STA IP `192.168.1.173`
- **ROS 2 Humble** with FastRTPS 2.6.11, running on WSL2 (mirrored networking), IP `192.168.1.92`
- Domain ID 0 → SPDP multicast `239.255.0.1:7400`, metatraffic unicast `:7410`, user data unicast `:7411`

## What Works

### SPDP (Participant Discovery) — Working
- ESP32 sends periodic 196-byte SPDP announcements to multicast `239.255.0.1:7400`
- Contains correct GUID prefix (derived from WiFi MAC), lease duration, builtin endpoint set (`0x2F`), `PID_USER_DATA` with `"enclave=/;"` (required by FastRTPS), default unicast/multicast locators
- Daemon receives these and discovers the ESP32 as a participant
- ESP32 receives daemon's 392-byte SPDP (with multiple locators including SHM) and correctly extracts the UDPv4 locator IP
- **Stale participant purging** now implemented — expired entries are removed after 2× lease duration, preventing the `_discovered` list from filling up with dead daemon instances

### SPDP on Metatraffic Port — Working
- Daemon sends unicast SPDP replies to ESP32's metatraffic port (7410)
- ESP32 now parses these as SPDP announcements (previously ignored)
- Lease timers are refreshed on re-reception of known participants

### SEDP (Endpoint Discovery) — Partially Working
- ESP32 sends 244-byte SEDP publication DATA to daemon's metatraffic port (7410)
- Announces the `ros_discovery_info` DataWriter with topic name, type name, reliability (RELIABLE), and durability (TRANSIENT_LOCAL)
- Message includes HEARTBEAT submessage (SN=1, count incrementing)
- **Issue:** Only 5 PIDs are serialized (ENDPOINT_GUID, TOPIC_NAME, TYPE_NAME, RELIABILITY, DURABILITY). FastRTPS WriterProxyData normally includes more (locators, PARTICIPANT_GUID, KEY_HASH, etc.) but missing PIDs should use defaults.

### ros_discovery_info — Partially Working
- ESP32 sends 168-byte CDR-encoded `ParticipantEntitiesInfo` to daemon's user data port (7411)
- Contains participant GID, node namespace `/`, node name `raft_esp32`, empty reader/writer GID sequences
- CDR field order is correct (namespace before name — this was a bug fix)
- Message wrapped in RTPS header + INFO_DST + INFO_TS + DATA + HEARTBEAT

### HEARTBEAT/ACKNACK — Partially Working
- ESP32 correctly responds to daemon's HEARTBEATs with ACKNACKs (on both metatraffic and user data ports)
- Daemon's HEARTBEAT sequence numbers are tracked and acknowledged
- **Issue:** ESP32 does NOT handle incoming ACKNACKs from the daemon (see below)

## What Doesn't Work

### `ros2 node list` returns empty

Despite correct SPDP mutual discovery, SEDP publication, and ros_discovery_info being sent, the node `/raft_esp32` does not appear.

## Debugging History

### Bug #1 — CDR Field Order (Fixed)
`buildRosDiscoveryInfoPayload` was serializing `node_name` before `node_namespace`. The IDL requires `node_namespace` first. Fixed.

### Bug #2 — Missing PID_USER_DATA in SPDP (Fixed)
FastRTPS silently ignores participants that don't include `PID_USER_DATA` containing `"enclave=/;"` in their SPDP announcement. Added.

### Bug #3 — Stale Participant Accumulation (Fixed)
`_discovered` vector (MAX_DISCOVERED=8) never purged expired entries. After 8 daemon restarts, the list was full of stale entries and the current daemon could never be added. ESP32's SEDP publications were being sent to 8 dead guidPrefixes. **Root cause confirmed via `tcpdump -XX` packet capture.**

Fix: `purgeStaleParticipants()` removes entries older than 2× lease duration. Called every loop iteration.

### Bug #4 — SPDP Ignored on Metatraffic Port (Fixed)
Daemon sends unicast SPDP DATA replies to ESP32:7410 (metatraffic). `recvMetatraffic()` only handled HEARTBEATs, discarding SPDP DATA. Now detects `writerEntityId == SPDP_PARTICIPANT_WRITER` and parses as announcement.

### Bug #5 — No Lease Refresh on Re-reception (Fixed)
When `recvSPDP()` found an existing participant by guidPrefix, it returned immediately without updating `discoveredTimeMs`. This meant participants could be purged even though they were still actively announcing. Now refreshes the timer.

### Investigation: FastDDS Source Code
Read `EDP.cpp`, `WriterProxyData.cpp`, `EDPSimpleListeners.cpp`, `PDP.cpp` to understand matching logic. The `valid_matching()` check compares topic name, type name, topicKind, reliability, durability, ownership, deadline, liveliness, and partitions. All should pass for our data.

### Investigation: Python Fake Node
Created standalone Python RTPS participants that mimic the ESP32's exact byte sequence. Also failed, ruling out ESP32-specific issues.

### Investigation: strace
`strace -e sendto,sendmsg` on `ros2 node list --no-daemon` showed the daemon sends 64-byte ACKNACKs to ESP32:7410 requesting SEDP data, and 68-byte HEARTBEATs, but never sends anything to port 7411 (user data). This confirms the daemon reaches the SEDP exchange phase but ESP32 doesn't respond to the daemon's data requests.

## Current Root Cause Hypothesis

**ESP32 ignores incoming ACKNACKs from the daemon.**

The RTPS reliable writer protocol requires:
1. Writer sends DATA + HEARTBEAT
2. Reader processes DATA, or if it missed it, sends ACKNACK requesting retransmission
3. Writer receives ACKNACK and retransmits the requested DATA

Currently the ESP32 pushes SEDP DATA once (in `handleNewParticipant`) and resends DATA+HB every second (in `sendWriterHeartbeats`), but there's a race condition: the daemon may not have created its WriterProxy yet when ESP32's first DATA arrives, so it sends an ACKNACK to request retransmission. ESP32 ignores this ACKNACK, so the daemon never gets the SEDP publication data.

The same issue applies to the ros_discovery_info writer on the user data port.

## What Needs to Be Done

### Immediate — Handle ACKNACKs (Reliable Writer)
1. In `recvMetatraffic()`: when an ACKNACK is received for `writerEntityId = SEDP_PUBLICATIONS_WRITER (0x000003C2)`, retransmit SEDP publication DATA
2. In `recvUserData()`: when an ACKNACK is received for `writerEntityId = ROS_DISC_INFO_WRITER (0x00000102)`, retransmit ros_discovery_info DATA
3. Both retransmissions should be triggered by the ACKNACK's bitmap indicating SN=1 is not yet received

### Secondary — SEDP Publication PIDs
Consider adding more PIDs to the SEDP publication message:
- `PID_PARTICIPANT_GUID` — links the writer back to the participant
- `PID_KEY_HASH` — endpoint key for keyed topics
- `PID_DEFAULT_UNICAST_LOCATOR` / `PID_UNICAST_LOCATOR` — explicit locators (fallback uses SPDP defaults)
- `PID_PROTOCOL_VERSION`, `PID_VENDORID`

FastRTPS handles missing PIDs with defaults, but some may be expected.

### Future — Phase 2
- Auto-generate ROS 2 publishers from Raft DeviceManager detected devices
- ROS 2 topic subscription (receive commands)
- Service server support
- Dynamic topic creation on device attach/detach
