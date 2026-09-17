# RTPS Modularization Investigation And Refactor Plan

**Date:** 2026-04-20
**Scope:** Analyze current RTPS helper/runner layout and propose a cleaner modular design.
**Constraint:** Planning only. No production code changes in this step.

## Scope Update: Zenoh Planning (2026-09-17)

This remains the RTPS-specific refactor history/plan. The forward transport
work is in [RaftROS-zenoh-implementation-plan.md](RaftROS-zenoh-implementation-plan.md).
Finish only the RTPS changes needed to establish its small backend boundary;
the broader cleanup below is not a prerequisite for the Zenoh feasibility
experiment.

"Shared runtime" here means RTPS behavior shared by Linux and ESP32, not
protocol behavior shared by DDS and Zenoh. Discovery, heartbeat/ACKNACK,
GUID/locator routing and SEDP belong behind the RTPS backend. Share the
DeviceManager lifecycle, semantic topic/type/QoS mapping and CDR pipeline
across backends, with opaque handles rather than RTPS writer IDs. Keep
platform I/O separate inside each backend, and verify RTPS regression and
single-backend source/dependency exclusion after each extraction.

## Executive Summary

The current RTPS runtime has improved DRY convergence, but it is now split across many very small helper files with top-level free functions. This was useful for low-risk incremental extraction, but it has reached a readability and navigation limit:

- behavior is spread across many files
- naming is inconsistent between policy, helper, plan, runner, adapter
- call chains are hard to trace end-to-end
- wrappers still carry a lot of callback wiring noise

A modular refactor is now beneficial. The best path is not to collapse everything into one large file, but to group by runtime domain and expose a few cohesive orchestrator interfaces.

## Terminology Layering (ROS 2 vs RTPS)

The user-facing ROS 2 concepts (publish/subscribe, services, actions) are higher-layer APIs.
The modules in this plan are mostly DDS/RTPS transport-runtime modules needed
by the DDS backend, not prerequisites for a native Zenoh backend.

- With a DDS RMW, ROS 2 publish/subscribe maps to DataWriter/DataReader behavior.
- Services/actions on that backend depend on its RTPS substrate; a Zenoh
	backend uses different graph and protocol primitives.

These modules implement ROS 2 concepts for RTPS, not a universal ROS 2
middleware substrate.

## What Exists Today

The RTPS folder currently contains a mixture of:

1. Packet/protocol primitives:
- `RTPSMessage`
- `RTPSParticipant`
- `SPDPHandler`
- `SEDPHandler`
- `RTPSTypes`

2. Micro-policy helpers (small free-function files):
- `RTPSReliabilityPolicy`
- `RTPSRuntimeSchedule`
- `RTPSParticipantLifecycle`
- `RTPSParticipantLeasePolicy`
- `RTPSParticipantSetPolicy`
- `RTPSBuiltinEndpointMap`
- `RTPSDiscoveredParticipantLookup`
- `RTPSDiscoveryPolicy`

3. Higher-order planners/runners:
- `RTPSInitialAnnouncePlan` + `RTPSInitialAnnounceRunner`
- `RTPSWriterHeartbeatRunner`
- `RTPSAckNack` + `RTPSAckNackRunner`
- `RTPSRxSubmessageRunner`
- `RTPSRunnerAdapterHelpers`

4. Wrapper glue:
- ESP wrapper in `components/RaftROS/RaftROS.cpp`
- Linux wrapper in `linux_unit_tests/raftros_standalone.cpp`

## Why The File Count Grew

This structure is a natural result of safe incremental refactoring:

- isolate one duplicated behavior
- extract to a small helper
- keep wrappers stable
- validate after each extraction

That strategy reduced regression risk and kept tests green. The downside is architectural drift toward many tiny files and free-function APIs that feel disconnected.

## Current Pain Points

1. Discoverability
- It is hard to know where to start for a behavior like ACKNACK, receive dispatch, or participant set updates.

2. Conceptual Overhead
- Many files each hold 1-3 functions, requiring frequent context switching.

3. Mixed Abstraction Levels
- very low-level checks and high-level orchestration appear as peers.

4. Callback Boilerplate
- Wrappers still define many lambda callbacks, obscuring intent.

5. Naming Inconsistency
- Terms like Policy, Plan, Runner, Helper are not yet standardized by domain.

## Refactor Goals

1. Keep protocol behavior shared and DRY.
2. Reduce file-count noise without creating monoliths.
3. Make end-to-end flows easy to follow.
4. Keep wrappers thin and focused on transport/lifecycle.
5. Preserve current behavior and validation baseline.

## Target Modular Shape

Use domain modules with cohesive public interfaces. Internals can still use helper functions, but hidden behind module boundaries.

### Module A: DiscoveryRuntime

**Owns:** participant merge, lookup, lease expiry, activation transitions.

Current pieces to absorb:
- `RTPSDiscoveryPolicy`
- `RTPSDiscoveredParticipantLookup`
- `RTPSParticipantLeasePolicy`
- `RTPSParticipantSetPolicy`
- `RTPSParticipantLifecycle`

Public interface example intent:
- merge/update participant
- resolve participant by guid/ip
- route metatraffic destination
- purge expired participants
- compute activation side effects

### Module B: RTPSReliabilityAndWriterStateRuntime

**Owns:** heartbeat response policy, ACKNACK parsing/classification/dispatch decisions.

Current pieces to absorb:
- `RTPSReliabilityPolicy`
- `RTPSAckNack`
- `RTPSAckNackRunner`
- `RTPSBuiltinEndpointMap`

Public interface example intent:
- process incoming ACKNACK into actions
- process heartbeat ACK policy decision
- classify writer kinds and retransmit gates

### Module C: BuiltinEndpointAnnouncementAndLivelinessRuntime

**Owns:** initial announce and periodic writer heartbeat orchestration.

Current pieces to absorb:
- `RTPSInitialAnnouncePlan`
- `RTPSInitialAnnounceRunner`
- `RTPSWriterHeartbeatRunner`
- (possibly) `RTPSRuntimeSchedule`

Public interface example intent:
- run initial announce bundle
- run periodic writer heartbeat bundle
- manage sequence/heartbeat counter mutation policy

### Module D: RTPSSubmessageDispatchRuntime

**Owns:** RTPS receive submessage walk and channel dispatch.

Current pieces to absorb:
- `RTPSRxSubmessageRunner`
- `RTPSRunnerAdapterHelpers` (or split into adapters namespace)

Public interface example intent:
- process metatraffic packet
- process user-data packet
- emit typed actions/callback hooks with minimal wrapper boilerplate

## API Style Recommendation

Use namespaced cohesive structs/classes per module instead of many standalone top-level functions.

Recommended style:
- `namespace RaftROS::RTPS::Runtime`
- module types like:
	- `DiscoveryRuntime`
	- `RTPSReliabilityAndWriterStateRuntime`
	- `BuiltinEndpointAnnouncementAndLivelinessRuntime`
	- `RTPSSubmessageDispatchRuntime`
- keep pure helper functions as `static` or internal namespace in module `.cpp`
- expose small config/state structs for deterministic behavior and testability

This keeps runtime logic modular while still C++-lightweight.

## Phased Implementation Plan

## Phase 0: Freeze And Baseline

1. Preserve current green baseline commands and outputs.
2. Capture current runtime call graph in this doc.
3. Add module-boundary comments in wrappers (no logic changes).

Exit criteria:
- No behavior changes.
- Team-agreed target module boundaries.

## Phase 1: DiscoveryRuntime Consolidation

1. Create `DiscoveryRuntime` module files.
2. Move discovery/lookup/lease/activation helpers behind this module.
3. Keep old helper files as thin forwarding shims temporarily.
4. Update wrappers and runners to call module interface only.

Exit criteria:
- Discovery-related calls originate from one module namespace.
- Tests and docker validation unchanged.

### Phase 1 Status (Started)

- Introduced new module entry point at:
	- `components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.h`
	- `components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.cpp`
- Began rewiring wrappers and adapter helpers to call the `DiscoveryRuntime` namespace API while preserving existing helper files as compatibility shims.

## Phase 2: RTPSReliabilityAndWriterStateRuntime Consolidation

1. Merge ACKNACK parse/classify/decision paths into `RTPSReliabilityAndWriterStateRuntime`.
2. Fold heartbeat-response and endpoint-map policies there.
3. Keep retransmit action mapping semantics identical.

Exit criteria:
- ACKNACK behavior traceable in one runtime module.
- No change in retransmit behavior.

## Phase 3: RTPSSubmessageDispatchRuntime Consolidation

1. Move receive runner + base adapter helpers under `RTPSSubmessageDispatchRuntime`.
2. Replace ad-hoc callback wiring with typed callback bundles per channel.
3. Standardize submessage log naming/policy in one place.

Exit criteria:
- Wrapper receive functions become short and declarative.
- Shared receive orchestration still fully reusable.

## Phase 4: BuiltinEndpointAnnouncementAndLivelinessRuntime Consolidation

1. Merge initial announce and writer heartbeat plan/runner internals under `BuiltinEndpointAnnouncementAndLivelinessRuntime`.
2. Normalize policy naming and file structure.
3. Keep sequence and heartbeat side-effect behavior unchanged.

Exit criteria:
- Announce and periodic resend orchestration are in one domain module.
- Wrapper send code remains transport-only.

## Phase 5: Cleanup And Rename Pass

1. Remove temporary forwarding shims.
2. Normalize naming conventions (Runtime, Policy, Action).
3. Update docs and architecture diagram.

Exit criteria:
- No orphan micro-helper files remain.
- Folder structure reflects clear domain ownership.

## Proposed File Layout (Illustrative)

`components/RaftROS/RTPS/runtime/discovery/`
- `DiscoveryRuntime.h/.cpp`

`components/RaftROS/RTPS/runtime/reliability/`
- `RTPSReliabilityAndWriterStateRuntime.h/.cpp`

`components/RaftROS/RTPS/runtime/announce/`
- `BuiltinEndpointAnnouncementAndLivelinessRuntime.h/.cpp`

`components/RaftROS/RTPS/runtime/receive/`
- `RTPSSubmessageDispatchRuntime.h/.cpp`

`components/RaftROS/RTPS/protocol/`
- existing protocol primitives (`RTPSMessage`, `RTPSParticipant`, `SPDPHandler`, `SEDPHandler`, `RTPSTypes`)

## Testing Strategy During Refactor

For each phase:

1. Build and run Linux unit tests:
- `cd linux_unit_tests && make -j$(nproc) all standalone && ./linux_unit_tests`

2. Docker runtime validation:
- Canonical rule for this repo/environment: run both the linux standalone publisher and ROS 2 validation commands inside the same docker container namespace.
- Build on host, then copy binary into container and execute there:
	- `cd linux_unit_tests && make -j$(nproc) all standalone`
	- `cd docker && docker compose up -d --build`
	- `docker cp linux_unit_tests/raftros_linux raftros-test:/workspace/raftros_linux`
	- `docker exec raftros-test chmod +x /workspace/raftros_linux`
	- `docker exec raftros-test bash -lc 'source /opt/ros/humble/setup.bash; /workspace/raftros_linux -i eth0 > /tmp/raftros_linux_in_container.out 2> /tmp/raftros_linux_in_container.err'`
- Validate from the same container:
	- node discovery visible in `ros2 node list`
	- `/chatter` visible in `ros2 topic list`
	- message received via `ros2 topic echo /chatter --once`
- Avoid using host-publisher + container-subscriber as the primary gate in this environment, because discovery can fail due to namespace/network separation rather than protocol regressions.

3. Compare key runtime logs for regressions:
- heartbeat resend
- ACKNACK retransmit triggers
- participant merge and lease behavior

## Risks And Mitigations

Risk: accidental behavior changes while moving many small helpers.
- Mitigation: phased moves, forwarding shims, frequent validation.

Risk: over-engineering abstractions.
- Mitigation: keep module interfaces small and procedural where possible.

Risk: longer transition with mixed APIs.
- Mitigation: strict per-phase completion and quick shim removal.

## Recommendation

Proceed with modular consolidation now, starting with DiscoveryRuntime. The current extraction has reached a point where readability and maintainability gains from domain modules outweigh the cost of keeping many tiny files.

This plan keeps the successful DRY refactor direction while improving navigability and long-term maintainability.
