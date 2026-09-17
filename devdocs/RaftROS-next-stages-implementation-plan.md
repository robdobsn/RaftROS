# RaftROS Next Stages Implementation Plan

**Date:** 2026-04-22
**Status update:** 2026-09-17 (Zenoh planning pointer; RTPS results unchanged)
**Scope:** Historical implementation plan for Phases 2–4. Phase 2
(publishing), Phase 3 (subscribing, including N-ary per-topic routing), and
Phase 4 (DeviceManager auto-publishing) are now complete. This document is
kept as an implementation history; use `RaftROS-development-status.md` for the
current status and known issues.

## Current Forward Work: Zenoh Alternative

Use [RaftROS-zenoh-implementation-plan.md](RaftROS-zenoh-implementation-plan.md)
for the next transport milestone. Its Z0-Z6 sequence covers feasibility under
the existing Raft-only/source-license and standalone goals, a native ROS
interoperability proof, common sensor/CDR extraction, mutually exclusive
RTPS/Zenoh builds, and dynamic sensor/subscription/QoS validation.

RTPS stays the default; Zenoh is not implemented. Runtime selection of both
backends is deferred. The remaining RTPS hardening/alignment items below stay
valid but do not block the initial Zenoh experiment. Do not reuse DDS graph,
type-hash leniency, or heartbeat assumptions as a Zenoh design.

## Status Summary

- Stage 1 (Lock in current behavior) — **done** for discovery and publish paths.
- Stage 2 (Extract shared runtime core) — **mostly done**; all four runtime flows (initial announce, ACKNACK, writer heartbeat, receive submessage dispatch) are now shared runners. A small amount of writer-state/action-execution policy still lives in wrappers.
- Stage 3 (ESP32 publishing completion) — **done and verified 2026-04-21**: `/chatter` publishes at 1 Hz, `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints every sample, RELIABLE + VOLATILE QoS honored, ACKNACK-driven retransmit proven to recover from initial LWIP send-queue drops.
- Stage 4 (Native Linux app path alignment) — **in progress**; most orchestration is shared; `raftros_standalone.cpp` is now mostly callbacks.
- Stage 5 (Hardening and test expansion) — **ongoing**.
- Stage 6 (Topic subscribing, Phase 3) — **done and verified 2026-04-22**: ESP32 subscribes to N topics (`/chatter_in`, `/chatter_in2`, …) with per-topic dispatch; `ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"` routes to `MainSysMod::chatter_in2` handler via remote-writer-GUID → slot mapping populated from SEDP publication DATAs.
- Stage 7 (DeviceManager auto-publishing, Phase 4) — **done and verified by 2026-04-25**: plugging a supported I2C sensor into `ExampleDiscoverable` creates a typed `/raft/...` ROS 2 topic, publishes CDR samples with standard message types, and disposes the endpoint on detach.
- Demo polish (2026-04-27) — **done for first demo path**: `examples/DemoSimple` dynamically discovers `/raft/...` topics, subscribes with compatible QoS, and displays live values. Foxglove Studio is documented with `foxglove_bridge` for both WSL-to-Windows and native Linux.
- Known caveat — `ros2 topic info -v` can still show `_NODE_NAME_UNKNOWN_` on the current WSL2/Jazzy/FastDDS setup. Data subscribers, `DemoSimple`, and Foxglove Bridge work. See `RaftROS-development-status.md` for the Task D investigation and current WSL-specific assumption.

## Remaining RTPS Objectives (Historical Sequence)

1. Finish convergence of the remaining writer-state/action-execution policy under `RTPSReliabilityAndWriterStateRuntime`.
2. ~~Add topic subscribing (Phase 3): SEDP reader announcement, reader-side ACKNACK, CDR deserialization, message dispatch hook.~~ **DONE 2026-04-22** (Stage 6).
3. ~~Start Phase 4: auto-wire publishers from Raft DeviceManager data sources through StatePublisher into RaftROS as a CommsChannel.~~ **DONE** via direct DeviceManager status/data callbacks and dynamic writer lifecycle. See `RaftROS-auto-publishing-design.md`.

## Guiding Principles

- Shared protocol behavior lives in common code.
- Platform wrappers stay thin (sockets, timers, startup/lifecycle).
- Add tests before and after moving logic to protect behavior.
- Preserve current externally visible behavior while refactoring.

## Stage 1: Lock In Current Behavior — DONE ✅

### Deliverables (delivered)

- Baseline protocol traces exist for:
  - discovery only
  - discovery + `/chatter` publish
  - ACKNACK-triggered retransmit paths (both SEDP publications and chatter user-data)
- Known-good command set for docker validation captured in `devdocs/RaftROS-development-status.md`.
- Known-good command set for real-hardware validation captured in `/memories/repo/raftros-rtps-findings.md`.

### Exit Criteria (met)

- Linux path still passes unit tests (**85 passed, 0 failed**) and reproduces expected pub/sub in docker.
- ESP32 path verified on WiFi against FastDDS Humble host.

## Stage 2: Extract Shared Runtime Core — MOSTLY DONE

### Deliverables (delivered)

- Platform-neutral runtime modules under `components/RaftROS/RTPS/runtime/`:
  - `runtime/discovery/` — participant set merge, lease expiry, activation
  - `runtime/reliability/` — ACKNACK decisions, writer-state policy, flavor presets
  - `runtime/announce/` — initial announce runner, writer-heartbeat runner, SEDP/SPDP handlers
  - `runtime/receive/` — RX submessage runner, runner-adapter helpers
  - `runtime/schedule/` — periodic cadence helper
  - `runtime/wire/` — RTPS message read/write
  - `runtime/core/` — participant/GUID/port/locator helpers
- ESP32 and Linux wrappers calling the same runtime APIs via runner callbacks.

### Remaining

- Move the last bits of writer-state / action-execution policy (still sitting in wrappers) behind `RTPSReliabilityAndWriterStateRuntime` while preserving existing wrapper callback APIs.

### Exit Criteria

- Diff between ESP32 and Linux wrappers stays mostly platform glue (sockets, timers, logging) — already largely true.

## Stage 3: ESP32 Publishing Completion — DONE ✅

### Deliverables (delivered)

- ESP32 advertises and publishes `/chatter` reliably.
- `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` on the PC receives periodic messages.
- ACKNACK-driven retransmit confirmed for both SEDP publications (chatter announcement) and chatter user-data.

### Bring-up fixes that landed

- `EspStyle` flavor now enables chatter retransmit on SEDP publications ACKNACK (`publicationsIncludesChatterAnnouncement=true`).
- Chatter SEDP DATA on writer `000003C2` now uses `seq=2` (vs `ros_discovery_info` at `seq=1`) to avoid duplicate-SN discard on the PC.
- VOLATILE chatter HEARTBEAT `firstSN` now equals the current sequence number so newly-matched subscribers do not NACK historical samples.
- `publishChatter()` passes `_chatterSeqNum` as `firstSN` to `SEDPHandler::buildUserDataMessage`.
- `ctx.sedpChatterPublicationSpec.buildMessage` is now a live lambda calling `SEDPHandler::buildPublicationMessage` via `getAckActionSedpPlan(RetransmitSedpChatterPublication, …)` — previously `nullptr`.

### Exit Criteria (met)

- Stable publish observed for >= 2 minutes with no endpoint disappearance and no ACKNACK retransmit storm.

## Stage 4: Native Linux App Path Alignment — IN PROGRESS

### Deliverables

- Linux standalone switched from "prototype runtime" to shared runtime wrapper.
- Minimal API for embedding RaftROS runtime in future native Linux apps documented.

### Tasks

- Continue reducing `raftros_standalone.cpp` to setup + IO loop + callbacks.
- Document required integration hooks for non-Raft Linux apps (what a host app must implement: socket send, current time, local GUID/IP, log).
- Keep docker validation script as a smoke test.

### Exit Criteria

- Linux standalone still works and code duplication is materially reduced (already largely true post-runtime extraction).

## Stage 5: Hardening and Test Expansion — ONGOING

### Deliverables

- Broader unit/integration coverage for reliability and edge cases.
- Regression checklist for both ESP32 and Linux.

### Suggested Tests

- ACKNACK base transitions (`base=1`, `base=2`, larger gaps), including the VOLATILE firstSN = currentSeq invariant
- Participant lease expiry/rejoin
- Writer heartbeat cadence under packet loss
- Multiple discovered participants (port/address routing)
- Initial-burst LWIP ENOMEM recovery: fail the 3rd+ unicast in `handleNewParticipant` in simulation, assert that the reliable path still delivers the chatter announcement.

## Stage 6: Topic Subscribing (Phase 3) — DONE ✅

### Deliverables (delivered)

- ESP32 subscribes to N ROS 2 topics (demonstrated with `rt/chatter_in` and `rt/chatter_in2`) and receives `std_msgs/msg/String` from FastDDS 2.6.11 publishers.
- Reader-side ACKNACK generation for RELIABLE topics via shared `RTPSReaderRuntime` / `RTPSReaderRunner` + bitmap-capable `RTPSMessage::writeAcknackWithBitmap`.
- CDR deserialization via `RTPSUserDispatch::decodeStdMsgsString`.
- Dispatch hook: `RaftROS::addStringSubscription(topic, type, handler)` registers a slot in `RTPSSubscriptionRegistry`; incoming user-data DATA is routed via `RTPSRemotePublicationMap` (remote writerGuid → slot) populated from SEDP publication DATAs parsed by `RTPSSEDPPublicationParser`.

### Phase-3 bugs fixed during bring-up

1. **Monotonic SEDP-sub sequence numbers** — `_extraSubscriptionSeqNums[8] = {2,3,4,5,6,7,8,9}`. Without per-slot distinct SNs, FastDDS dropped the >=1 slot announcements as duplicates of the main-sub writer's SN space. Regression-guarded by the distinct-SN test landed in Slice 2.
2. **ACKNACK bitmap inversion** ([RTPSReaderRuntime.cpp](../components/RaftROS/RTPS/runtime/reliability/RTPSReaderRuntime.cpp)) — per RTPS §9.4.5.2, a `1` bit in an ACKNACK `readerSNState` bitmap means **NOT received / please retransmit**. We were inverting the sense, so FastDDS interpreted our ACKNACKs as positive ACKs and never retransmitted missed SEDP publications. Regression-guarded by assertions `d.ackNackBitmap == 0x1F` (5 missing) and `0x3` (2 missing) in `linux_unit_tests/main.cpp`.
3. **Inline-QoS skipping in DATA payload extraction** ([RTPSRxSubmessageRunner.h](../components/RaftROS/RTPS/runtime/receive/RTPSRxSubmessageRunner.h)) — DATA submessages with Q flag (bit 1 = 0x02) carry an inline-QoS ParameterList before the serialized payload. Propagated the DATA `flags` byte through `onData`, added `RTPSData_getSerializedPayload()` helper that scans past the 20-byte prefix and the inline-QoS list up to `PID_SENTINEL`. Regression-guarded by 9 new assertions covering Q=0, Q=1 sentinel-only, FastDDS dispose-style (`PID_STATUS_INFO` + `PID_KEY_HASH` + sentinel, no trailing payload), malformed parameter runs-off-end, and contentLen<20.

### Exit Criteria (met)

- `ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello slot1'}"` and `ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"` both deliver to their distinct `MainSysMod::chatter_in` / `chatter_in2` handlers within one publisher heartbeat.
- Log evidence (device-side, 2026-04-22):
  - `SEDP pub DATA received contentLen=468 flags=0x05`
  - `SEDP pub parsed topic='rt/chatter_in2' (14 chars), subReg.count=2`
  - `SEDP pub matched topic='rt/chatter_in2' -> slot=1 (new)`
  - `MainSysMod: chatter_in2 #1 writerEID=00000503 src=010FCCEF... "hello slot2" (11 chars)`
- Linux unit tests: **388 passed, 0 failed**.

### Remaining (Phase-3 polish)

- Reader-side ACKNACK consolidation: fold the last per-writer-kind bookkeeping (labels, FINAL-flag suppression nuances) fully behind `RTPSReaderRuntime`.

## Stage 7: DeviceManager Auto-Publishing (Phase 4) — DONE ✅

### Deliverables

- DeviceManager status-change callbacks instantiate/release dynamic writers.
- DeviceManager data callbacks feed decoded poll records into CDR serializers.
- DeviceTypeRecord → ROS 2 message type mapping (`clas` tag → `sensor_msgs/*`, `std_msgs/*`, `geometry_msgs/*`) is implemented by `RTPSAutoPubClassMap`.
- Dynamic SEDP publication ADD / DISPOSE announcements are emitted as devices appear/disappear.
- Per-class QoS profiles and SysTypes overrides are implemented.

### Exit Criteria

- Plugging an I2C sensor into a running `ExampleDiscoverable` board creates a new ROS 2 topic visible to `rclpy` graph discovery and `DemoSimple` within a few seconds.
- Example validation on 2026-04-27: VL6180 appears as `/raft/range_1_29` (`sensor_msgs/msg/Range`) and publishes live range samples.
- Note: `ros2 topic list` / `ros2 topic info` remain less reliable on WSL/Jazzy because of ros2cli daemon and `_NODE_NAME_UNKNOWN_` graph-attribution behavior. Prefer `DemoSimple`/`rclpy` for validation on that host.

## Historical RTPS Implementation Order

1. ~~Finish moving remaining wrapper-side writer-state/action-execution policy behind `RTPSReliabilityAndWriterStateRuntime`.~~ **DONE 2026-04-21** — shared `RTPSAckActionStandardCtx` + `_initStandardAckActionCtx`/`_standardExecuteAction`/`_standardGetChatterSeq` helpers in `RTPSRunnerAdapterHelpers`; both wrappers converted; Linux validated (108 passed). ESP on-device smoke test still pending.
2. ~~Add a focused unit test for VOLATILE `firstSN == currentSeq` HEARTBEAT invariant (regression guard for Stage 3 Fix 15).~~ **DONE 2026-04-21** — guard in `linux_unit_tests/main.cpp`.
3. ~~Add a focused unit test for "two DataWriter announcements on the same SEDP publications writer must use distinct sequence numbers" (regression guard for Stage 3 Fix 16).~~ **DONE 2026-04-21** — guard in `linux_unit_tests/main.cpp`.
4. Start Stage 6 (Phase 3 — subscribing) with a skeleton reader runtime behind new runner callbacks, mirroring the writer runner structure. **DONE 2026-04-22** — shared `RTPSReaderRuntime` (pure state + `evaluateIncomingDataDecision` / `applyAcceptedDataToReaderState` / `evaluateIncomingHeartbeatDecision` / `applyHeartbeatProcessedToReaderState`) landed with 27 unit-test assertions covering DATA accept/dedup/gap collapse and HEARTBEAT -> ACKNACK bitmap generation incl. FINAL-flag dedup and best-effort suppression. Shared `RTPSReaderRunner` (submessage parsers + `resolveState`/`sendAckNack`/`dispatchData` callbacks) landed with +34 assertions. `RTPSMessage::writeAcknackWithBitmap` bitmap-capable ACKNACK wire builder landed with +18 assertions (empty-bitmap equivalence with legacy builder, multi-word LE packing, FINAL flag, input validation). `RTPSRxSubmessageRunner` extended with opt-in `resolveReaderWriterState` callback that routes HEARTBEATs through the pure reader runtime decision + bitmap ACKNACK builder; when null, preserves legacy path byte-for-byte. Header-only `RTPSReaderStateMap` per-(guidPrefix,writerEID) state container added and wired into both wrappers (`RaftROS.cpp`, `raftros_standalone.cpp`); every incoming HEARTBEAT now consults shared reader runtime on both Linux and ESP build paths. `RTPSInitialAnnouncePlan` extended with opt-in `SedpChatterReader` action (new `ENTITYID_CHATTER_READER` / `CHATTER_IN_DDS_TOPIC`) and dedicated `sedpChatterReaderSeqNum` counter. Wrappers now opt in: both `RaftROS.cpp` and `raftros_standalone.cpp` set `sendSedpChatterReader=true` during initial announce and include the reader GID in `ros_discovery_info`, so ROS 2 peers should see a `rt/chatter_in` subscriber on this participant. Header-only `RTPSUserDispatch::decodeStdMsgsString` added and wired into both wrappers' user-data RX callback: non-ros_discovery_info user-topic DATA is decoded and logged, and `RaftROS::setStringMessageHandler` exposes the decoded message to user code. `RTPSWriterHeartbeatRunner` extended with a `SedpChatterSubscription` retransmit step (dedicated `chatterSedpSubSeqNum` counter on ESP / `sedpSubSeqNum+1` on Linux-flavor) so late joiners also learn the reader via periodic SEDP. **On-device confirmed 2026-04-22** on ESP32-S3 `192.168.1.173` ↔ ROS 2 Humble host `192.168.1.92` (FastDDS 2.6.11): `ros2 topic pub --once /chatter_in std_msgs/msg/String "{data: 'hello esp'}"` decoded to `UD user-topic ... "hello esp" (9 chars)`; chatter pub ACKNACK `base=566 numBits=0`. Slice 8 polish: added `setStringSubscription(topic,type,handler)` (runtime-configurable, slot-0 only), extended ACKNACK writer-kind table with `ParticipantMessage` + `ChatterReader` labels, liveliness HB firstSN=lastSN=seq (stops NACK storm), wired handler in `ExampleDiscoverable/MainSysMod`. Slice 9 (N-ary subscription registry): new shared `RTPSSubscriptionRegistry` (8-slot POD, deterministic per-slot entity ID allocator), wrapper `addStringSubscription(topic,type,handler)` appends extra slots, announce + heartbeat + `ros_discovery_info` all iterate every registered reader. Dispatch still single-handler; per-topic routing (remote SEDP-pub correlation) is the last remaining Phase 3 follow-up ("Phase 3.5: subscriber routing"). **Total 298 passed, 0 failed.** Next: per-topic dispatch via SEDP-pub correlation + reader-side ACKNACK consolidation.

   **Slice 12 (per-topic routing) DONE 2026-04-22:** `RTPSSEDPPublicationParser` extracts `PID_ENDPOINT_GUID` + `PID_TOPIC_NAME` from remote SEDP publication DATAs; `RTPSRemotePublicationMap` (16-entry fixed map) correlates remote writerGuid -> local slot; user-data DATAs dispatch via `_remotePubMap.findSlot()` to `_extraSubscriptionHandlers[slot]`. Three bugs found and fixed during bring-up: (1) non-monotonic `_extraSubscriptionSeqNums`, (2) ACKNACK bitmap sense inverted vs RTPS §9.4.5.2, (3) `onData` ignored DATA `Q` flag and always read `pContent + 20` as payload — broke FastDDS because dispose-style DATAs put PID_STATUS_INFO + PID_KEY_HASH + PID_SENTINEL inline before any (absent) payload. New helper `RTPSData_getSerializedPayload()` skips fixed prefix + inline-QoS ParameterList; `flags` now propagated through `onData`. On-device validated with both `/chatter_in` (slot 0) and `/chatter_in2` (slot 1). **Total 388 passed, 0 failed.**
5. Re-run `make -j$(nproc) all standalone && ./linux_unit_tests` after each extraction batch.

## Risks and Mitigations

- **Risk:** Refactor breaks currently working Linux or ESP32 path.
  - **Mitigation:** Keep each move small; run unit tests and docker smoke after every step; re-flash ESP32 and re-run `ros2 topic echo /chatter` after ACKNACK/writer-state changes.
- **Risk:** Reader-side reliability subtleties (missed heartbeats, out-of-order DATA) introduce the same kind of hard-to-diagnose bugs seen in the writer path.
  - **Mitigation:** Put reader decision logic in shared runtime from day one; keep wrappers to IO only; mirror the writer-side test pattern.
- **Risk:** Dynamic endpoint creation (Phase 4) conflicts with the static SEDP sequence numbering assumptions.
  - **Mitigation:** Done. Dynamic writers use registry-managed entity IDs and monotonically increasing SEDP publication sequence numbers, covered by Linux tests and on-device validation.
- **Risk:** `ros2` CLI graph attribution reports `_NODE_NAME_UNKNOWN_` on WSL/Jazzy, making demos look broken even when data subscriptions work.
  - **Mitigation:** Use `DemoSimple`/direct `rclpy` and Foxglove Bridge as the primary demo surfaces. Treat native Linux reproduction as the next discriminator before assuming a firmware protocol defect.

## Definition of Done for the Previous RTPS Milestone

- Phase 4 remains stable with hot-plug auto-published device topics.
- `DemoSimple` remains the canonical simple demo for dynamic topic discovery.
- Native Linux validation is run to confirm whether `_NODE_NAME_UNKNOWN_` is WSL/Jazzy-specific.
- Unit tests pass and devdocs reflect test commands, expected outputs, and known caveats.
