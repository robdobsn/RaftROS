# RaftROS Next Stages Implementation Plan

**Date:** 2026-04-21
**Scope:** Phase 2 (ESP32 topic publishing) is now complete and verified end-to-end. Remaining work: finish shared-runtime convergence, add Phase 3 topic subscribing, and Phase 4 DeviceManager integration.

## Status Summary

- Stage 1 (Lock in current behavior) — **done** for discovery and publish paths.
- Stage 2 (Extract shared runtime core) — **mostly done**; all four runtime flows (initial announce, ACKNACK, writer heartbeat, receive submessage dispatch) are now shared runners. A small amount of writer-state/action-execution policy still lives in wrappers.
- Stage 3 (ESP32 publishing completion) — **done and verified 2026-04-21**: `/chatter` publishes at 1 Hz, `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints every sample, RELIABLE + VOLATILE QoS honored, ACKNACK-driven retransmit proven to recover from initial LWIP send-queue drops.
- Stage 4 (Native Linux app path alignment) — **in progress**; most orchestration is shared; `raftros_standalone.cpp` is now mostly callbacks.
- Stage 5 (Hardening and test expansion) — **ongoing**.

## Objectives (forward)

1. Finish convergence of the remaining writer-state/action-execution policy under `RTPSReliabilityAndWriterStateRuntime`.
2. Add topic subscribing (Phase 3): SEDP reader announcement, reader-side ACKNACK, CDR deserialization, message dispatch hook.
3. Start Phase 4: auto-wire publishers from Raft DeviceManager data sources through StatePublisher into RaftROS as a CommsChannel.

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

## Stage 6: Topic Subscribing (Phase 3) — NEXT

### Deliverables

- ESP32 can subscribe to a ROS 2 topic (e.g., `/cmd`) and receive `std_msgs/msg/String` (or similar) from a ROS 2 publisher.
- Reader-side ACKNACK generation for RELIABLE topics.
- CDR deserialization of standard message types.
- Dispatch hook: user application SysMod receives a typed callback per incoming message.

### Tasks

- Add SEDP subscription announcement for the user topic (mirroring the existing chatter publication path).
- Update `ros_discovery_info` payload to include the new reader GID.
- Extend the RX submessage runner to route user-data DATA into a shared reader runtime.
- Implement reader-side ACKNACK (RELIABLE) with heartbeat-count tracking per remote writer.
- Add CDR decode path for the initial supported message types.

### Exit Criteria

- `ros2 topic pub /cmd std_msgs/msg/String "{data: hello}"` from the PC is received and logged by the ESP32 within one publisher heartbeat.

## Stage 7: DeviceManager Auto-Publishing (Phase 4) — FUTURE

### Deliverables

- `RaftROS` registers as a `CommsChannel` with `CommsCoreIF`.
- Configured `pubSources` wire StatePublisher subscriptions (e.g., `devjson`, `devbin`) into the RaftROS channel.
- DeviceTypeRecord → ROS 2 message type mapping (`clas` tag → `sensor_msgs/*`).
- Dynamic SEDP publication announcement as devices appear/disappear.

### Exit Criteria

- Plugging an I2C sensor into a running ExampleDiscoverable board creates a new ROS 2 topic visible in `ros2 topic list` within one SEDP heartbeat.

## Recommended Immediate Implementation Order

1. ~~Finish moving remaining wrapper-side writer-state/action-execution policy behind `RTPSReliabilityAndWriterStateRuntime`.~~ **DONE 2026-04-21** — shared `RTPSAckActionStandardCtx` + `_initStandardAckActionCtx`/`_standardExecuteAction`/`_standardGetChatterSeq` helpers in `RTPSRunnerAdapterHelpers`; both wrappers converted; Linux validated (108 passed). ESP on-device smoke test still pending.
2. ~~Add a focused unit test for VOLATILE `firstSN == currentSeq` HEARTBEAT invariant (regression guard for Stage 3 Fix 15).~~ **DONE 2026-04-21** — guard in `linux_unit_tests/main.cpp`.
3. ~~Add a focused unit test for "two DataWriter announcements on the same SEDP publications writer must use distinct sequence numbers" (regression guard for Stage 3 Fix 16).~~ **DONE 2026-04-21** — guard in `linux_unit_tests/main.cpp`.
4. Start Stage 6 (Phase 3 — subscribing) with a skeleton reader runtime behind new runner callbacks, mirroring the writer runner structure.
5. Re-run `make -j$(nproc) all standalone && ./linux_unit_tests` after each extraction batch.

## Risks and Mitigations

- **Risk:** Refactor breaks currently working Linux or ESP32 path.
  - **Mitigation:** Keep each move small; run unit tests and docker smoke after every step; re-flash ESP32 and re-run `ros2 topic echo /chatter` after ACKNACK/writer-state changes.
- **Risk:** Reader-side reliability subtleties (missed heartbeats, out-of-order DATA) introduce the same kind of hard-to-diagnose bugs seen in the writer path.
  - **Mitigation:** Put reader decision logic in shared runtime from day one; keep wrappers to IO only; mirror the writer-side test pattern.
- **Risk:** Dynamic endpoint creation (Phase 4) conflicts with the static SEDP sequence numbering assumptions.
  - **Mitigation:** Track a per-writer SEDP-pub sequence counter already centralized — just ensure new writers get a fresh unique SN; cover with unit tests before go-live.

## Definition of Done for the Next Milestone

- Phase 3: ESP32 receives a ROS 2 topic reliably, with shared reader runtime behind runner callbacks.
- Phase 2 regression tests in place (VOLATILE firstSN invariant; distinct SEDP-pub SNs).
- Linux and ESP32 use the same writer and reader orchestration logic in shared code.
- Unit tests pass and devdocs reflect test commands and expected outputs.
