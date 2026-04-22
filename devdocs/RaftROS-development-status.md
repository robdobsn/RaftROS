# RaftROS Development Status

**Last Updated:** 2026-04-21

## Goal

Make an ESP32-S3 (running Raft firmware) appear as a native ROS 2 node — without micro-ROS or any agent process — using a clean-room RTPS 2.2 implementation as a Raft SysMod.

**Phase 1 (Discovery) and Phase 2 (Topic Publishing) are COMPLETE.**
- `ros2 node list` shows `/raft_esp32`.
- `ros2 topic list` shows `/chatter`.
- `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints a sample per second.

Verified end-to-end on 2026-04-21 against ROS 2 Humble + FastDDS 2.6.11, ESP32-S3 on real WiFi (PC `192.168.1.92`, ESP32 `192.168.1.173`).

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

- Phase 2 (ESP32 `/chatter` publishing) is complete and verified end-to-end against ROS 2 Humble + FastDDS.
- Phases 3 (subscribing) and 4 (auto-wiring from DeviceManager) are still TODO.
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
- Next: start Phase 3 (topic subscribing). Fold remaining `RTPSAckNackRunner.cpp` internals behind `RTPSReliabilityAndWriterStateRuntime` and add a reader-side ACKNACK + deserialization path with the same shared-runtime discipline from day one.

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
