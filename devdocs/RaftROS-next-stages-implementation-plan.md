# RaftROS Next Stages Implementation Plan

**Date:** 2026-04-20
**Scope:** Complete ESP32 publishing while converging Linux and ESP32 runtimes toward one shared core.

## Objectives

1. Get reliable ROS 2 topic publishing working on ESP32 in a repeatable testbed.
2. Reduce divergence between Linux and ESP32 runtime logic.
3. Establish a structure that supports both embedded and future native Linux applications.

## Guiding Principles

- Shared protocol behavior lives in common code.
- Platform wrappers stay thin (sockets, timers, startup/lifecycle).
- Add tests before and after moving logic to protect behavior.
- Preserve current externally visible behavior while refactoring.

## Stage 1: Lock In Current Behavior (Short)

### Deliverables

- Baseline protocol traces for:
  - discovery only
  - discovery + `/chatter` publish
  - ACKNACK-triggered retransmit paths
- Saved known-good command set for docker validation.

### Tasks

- Capture packet/log signatures for Linux standalone currently known to work.
- Add a short verification checklist in `devdocs` for quick regressions.
- Add unit tests for any ACKNACK branch not currently covered.

### Exit Criteria

- Existing Linux path still passes unit tests and reproduces expected pub/sub in docker.

## Stage 2: Extract Shared Runtime Core (Medium)

### Deliverables

- New platform-neutral runtime module under `components/RaftROS/` (for example `CoreRuntime`), containing:
  - participant state updates
  - heartbeat scheduling decisions
  - ACKNACK handling decisions/retransmit selection
  - endpoint sequence/heartbeat counters
- ESP32 and Linux wrappers calling the same runtime APIs.

### Tasks

- Introduce minimal interfaces:
  - `sendPacket(channel, bytes, len, remote)`
  - `nowMs()`
  - `getLocalParticipantInfo()`
  - `log(level, msg)`
- Move logic incrementally from:
  - `components/RaftROS/RaftROS.cpp`
  - `linux_unit_tests/raftros_standalone.cpp`
- Keep RTPS/CDR encoding where it already exists.

### Exit Criteria

- No functional behavior change in linux tests.
- Diff between ESP32 and Linux wrappers is mostly platform glue.

### Stage 2 Progress (Current)

- Completed shared runner-based orchestration for:
  - initial participant announce flow
  - ACKNACK parse/classify/dispatch flow
  - periodic writer-heartbeat resend flow
  - receive submessage dispatch + HEARTBEAT ACK response flow
- Linux validation baseline remains: `85 passed, 0 failed`.

## Stage 3: ESP32 Publishing Completion (Medium)

### Deliverables

- ESP32 advertises and publishes `/chatter` (or chosen first topic) reliably.
- `ros2 topic echo` receives periodic messages from ESP32.
- ACKNACK-driven retransmit confirmed for writer path.

### Tasks

- Ensure SEDP writer announcement for published topic is sent on participant discovery and heartbeat cycle.
- Ensure message payload generation and writer sequence handling are stable.
- Confirm `ros_discovery_info` entity list includes active writer GIDs.
- Validate with ROS 2 listener and topic introspection commands.

### Exit Criteria

- Stable publish observed for >= 2 minutes in testbed with no endpoint disappearance.

## Stage 4: Native Linux App Path Alignment (Medium)

### Deliverables

- Linux standalone switched from "prototype runtime" to shared runtime wrapper.
- Minimal API for embedding RaftROS runtime in future native Linux apps documented.

### Tasks

- Reduce `raftros_standalone.cpp` to setup + IO loop + callbacks.
- Document required integration hooks for non-Raft Linux apps.
- Keep docker validation script as a smoke test.

### Exit Criteria

- Linux standalone still works and code duplication is materially reduced.

## Stage 5: Hardening and Test Expansion (Ongoing)

### Deliverables

- Broader unit/integration coverage for reliability and edge cases.
- Regression checklist for both ESP32 and Linux.

### Suggested Tests

- ACKNACK base transitions (`base=1`, `base=2`, larger gaps)
- Participant lease expiry/rejoin
- Writer heartbeat cadence under packet loss
- Multiple discovered participants (port/address routing)

## Recommended Implementation Order (Immediate)

1. Batch-extract remaining shared participant-state/container logic (lookup, merge/update, purge triggers, and send-target lookup helpers) into RTPS shared modules.
2. Batch-extract wrapper callback-adapter boilerplate into reusable shared adapter helpers to reduce repeated lambda wiring.
3. Add/extend linux unit tests around receive callback behavior and participant routing decisions before/after extraction.
4. Re-run `make -j$(nproc) all standalone && ./linux_unit_tests` after each extraction batch.
5. Validate ESP32 publishing in the docker/WiFi testbed once the shared runtime convergence batch completes.

## Risks and Mitigations

- **Risk:** Refactor breaks currently working Linux path.
  - **Mitigation:** Keep each move small; run unit tests and docker smoke after every step.
- **Risk:** ESP32 timing/network behavior differs from Linux assumptions.
  - **Mitigation:** Keep timing and socket behavior behind platform callbacks; avoid Linux-specific defaults in shared core.
- **Risk:** Discovery/publishing drift reappears.
  - **Mitigation:** Shared runtime owns state machine; wrappers only adapt IO.

## Definition of Done for the Next Milestone

- ESP32 `/chatter` publisher works in ROS 2 testbed.
- Linux and ESP32 use the same ACKNACK and heartbeat orchestration logic in shared code.
- Unit tests pass and devdocs reflect test commands and expected outputs.
