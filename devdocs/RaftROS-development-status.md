# RaftROS Development Status

**Last Updated:** 2026-04-20

## Goal

Make an ESP32-S3 (running Raft firmware) appear as a native ROS 2 node — without micro-ROS or any agent process — using a clean-room RTPS 2.2 implementation as a Raft SysMod.

**Phase 1 (Discovery) is COMPLETE.** The ESP32 appears as `/raft_esp32` in `ros2 node list`.

## Architecture

Clean-room RTPS 2.2 implementation. No third-party DDS libraries.

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
| `components/RaftROS/RTPS/SEDPHandler.cpp/.h` | SEDP publication/subscription/liveliness message build, user data message build |
| `linux_unit_tests/` | Linux-hosted unit tests + standalone linux RTPS node (`raftros_standalone.cpp`) |
| `unit_tests/` | ESP-IDF Unity tests |
| `examples/ExampleDiscoverable/` | ESP32 app that runs RaftROS as a SysMod |

### Hardware / Environment

- **ESP32-S3**, MAC `94:a9:90:3a:51:b0`, WiFi STA IP `192.168.1.173`
- **ROS 2 Humble** with FastRTPS 2.6.11, running on WSL2 (mirrored networking), IP `192.168.1.92`
- Domain ID 0 → SPDP multicast `239.255.0.1:7400`, metatraffic unicast `:7410`, user data unicast `:7411`

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

### Root Cause Found for Previous "Not Publishing" Symptom

- The primary blocker observed in earlier docker testing was host/container network separation (discovery traffic from host IP space did not reach container DDS participants as expected in that setup).
- This was not primarily caused by ACKNACK logic.

### Remaining Gap

- ESP32 path still needs Phase 2 completion (topic publishing endpoint lifecycle + runtime validation against ROS 2 subscribers under real WiFi conditions).
- Linux and ESP32 orchestration code still have drift risk because key runtime logic is duplicated in:
	- `components/RaftROS/RaftROS.cpp`
	- `linux_unit_tests/raftros_standalone.cpp`

## Phase 2: Topic Publishing — IN PROGRESS

Immediate objective: publish real ROS 2 topic data from the ESP32 with reliable delivery.

- Announce a new DataWriter endpoint (e.g. `std_msgs/msg/String` on `/chatter`) via SEDP
- Serialize ROS 2 messages using CDR encoder
- Send DATA messages to remote subscribers' user data ports
- Handle HEARTBEAT/ACKNACK reliable delivery for the new writer
- Update `ros_discovery_info` payload to include the new writer's GID
- Validate on ESP32 against ROS 2 `demo_nodes_cpp` listener in a controlled testbed

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
- Next: move heartbeat scheduling/retransmit policy decisions into shared runtime APIs while keeping send/recv adapters platform-specific.

### Practical Rule

- If logic can be expressed without direct socket API calls or FreeRTOS/Arduino-specific APIs, it belongs in shared core code.
- Wrappers should mainly provide:
	- packet send/receive callbacks
	- current time
	- local network identity
	- logging hooks

## Phase 3: Topic Subscribing — TODO

- Announce a new DataReader endpoint via SEDP subscription
- Receive and deserialize incoming ROS 2 messages
- Handle ACKNACK (reader side) for reliable subscriptions

## Phase 4: Integration with Raft — TODO

- Auto-generate ROS 2 publishers from Raft DeviceManager detected devices
- ROS 2 topic subscription for receiving commands
- Dynamic topic creation on device attach/detach
- Service server support
