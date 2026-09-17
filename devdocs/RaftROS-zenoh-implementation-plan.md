# RaftROS Zenoh Alternative: Design and Implementation Plan

**Date:** 2026-09-17
**Status:** Planned only; all Z-series slices below are not started.
**Scope:** A native ROS 2 Zenoh backend for Raft ESP32 firmware, preserving
DeviceManager-driven sensor auto-publishing and the existing RTPS backend.

This is the forward plan for the next transport milestone, ahead of services
and parameters. Phases 1-4 remain completed **RTPS** work, not evidence of Zenoh
support. See [development status](RaftROS-development-status.md), the
[overview](RaftROS-overview.md), and the implemented
[auto-publishing design](RaftROS-auto-publishing-design.md).

## 1. Decisions and Constraints

| Decision | Planned position |
| --- | --- |
| User-visible behavior | Auto-detected supported I2C sensors publish the same ROS topic names, standard types, values, units, frame IDs, and supported QoS profiles. Preserve composite mappings and detach/re-attach behavior. |
| Firmware | Raft + ESP-IDF/FreeRTOS/lwIP only under the existing dependency policy; no RCL, rclcpp, host RMW, micro-ROS, Rust runtime, or per-device generated ROS application on the ESP32. |
| ROS interoperability | Native `rmw_zenoh_cpp`, not arbitrary Zenoh pub/sub and not a DDS-to-Zenoh bridge protocol. |
| Build selection | Exactly one backend: proposed `RAFTROS_TRANSPORT=RTPS` (default) or `ZENOH`. Selection must exclude sources, state, headers, and dependencies of the other backend. |
| Runtime selection | Deferred. Network addresses and node/domain settings remain runtime configuration; choosing the middleware does not. |
| Standalone | No device agent or translation bridge. Also preserve the original no-separate-process goal: routerless operation is a required feasibility gate, not an assumed Zenoh capability. |
| Compatibility target | Start with native Linux, ROS 2 Jazzy and a pinned `rmw_zenoh_cpp`/Zenoh wire profile. ESP32-S3 follows, then the existing WSL2 demo environment. |

### 1.1 Dependency and licensing decision

The [overview's licensing policy](RaftROS-overview.md#71-open-source-rtpsdds-implementations)
excludes incorporating non-MIT implementation code. Interpret "Raft libraries
only" conservatively: **do not silently add zenoh-pico to firmware**.

- Baseline: implement a bounded, Raft-owned Zenoh subset from protocol
  documentation, with original MIT source, as was done for RTPS. Upstream
  implementations may be behavioral references and host-side test oracles,
  not copied or relicensed source. Establish sufficient protocol documentation
  and an acceptable implementation process in Z0 before committing to this.
- Effort-saving alternative: a separately packaged, pinned **zenoh-pico**
  dependency, which already targets ESP-IDF. This needs explicit maintainer
  approval to relax the dependency policy, plus review of the selected
  Apache-2.0/EPL-2.0 license and redistribution/notice obligations. Keeping
  Raft-authored code MIT does not make a third-party dependency MIT.
- Do not port the full desktop Zenoh stack. Do not estimate a Raft-owned
  protocol implementation as equivalent in effort to wrapping zenoh-pico.
  Z0 must compare flash/RAM, maintenance, interoperability, and implementation
  effort for both approaches before committing to a delivery estimate.

No dependency approval is assumed by this plan. If the strict option is not
feasible, report that result and obtain a decision rather than weakening the
goal or importing code implicitly.

### 1.2 Router versus agent

The normal `rmw_zenoh_cpp` topology uses `rmw_zenohd` for discovery/routing.
A router forwards native Zenoh traffic; it is not a micro-ROS agent and does
not implement the ESP32's ROS node on its behalf. Nevertheless it is an
external runtime dependency, so router-assisted success alone does not meet
the strict standalone goal.

Z0/Z1 must prove graph discovery and data exchange directly with a host RMW
session using explicitly configured reachable endpoints. Verify the selected
client/peer modes and discovery/liveliness behavior; zenoh-pico multicast
peer examples and desktop RMW peer examples do not establish mutual support.
One direct peer is the first discriminator; the release gate includes several
independent ROS processes and late joiners, without a router.

Use a router-assisted topology as a diagnostic control. If direct operation
cannot work within the chosen subset, stop at the gate. An optional or
required router deployment can be approved separately, but must be labeled
honestly and must not be substituted for the standalone acceptance test.

### 1.3 Interoperability limits

A Zenoh-only ESP32 does not communicate directly with Fast DDS or Cyclone DDS.
All participating host applications, CLI processes, dashboard, and Foxglove
Bridge must use `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. `zenoh-bridge-ros2dds` uses a
different mapping and is not an interchangeable route to `rmw_zenoh_cpp`.

Do not promise cross-distribution compatibility: upstream documents a Humble
versus Iron-and-newer type-hash incompatibility. Jazzy is the first target;
Humble, Kilted, and Rolling require separate recorded compatibility tests.

## 2. Current Code and the Extraction Boundary

| Current owner | Reuse or isolate |
| --- | --- |
| [RaftROS.cpp](../components/RaftROS/RaftROS.cpp): `setup`, `autoPubOnDeviceStatusChange`, attach/detach and `autoPubOnDeviceData` | Keep SysMod and DeviceManager integration shared. Separate decoding/serialization from endpoint creation and network sends. |
| [RaftROS.h](../components/RaftROS/RaftROS.h) | Currently includes RTPS types and embeds participants, sockets, registries, and sequence state. A CMake source switch alone cannot produce an independent Zenoh build. |
| [CDR](../components/RaftROS/CDR/CDREncoder.h) and [sensor serializer](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubCDRSerializer.h) | Reuse CDR algorithms and golden tests. Move transport-neutral serializer ownership out of the RTPS namespace/path incrementally. |
| [Class mapping](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubClassMap.h), [topic naming](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubTopicNaming.h), [QoS profiles](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubQoSProfile.h) | Share semantic mappings; separate DDS wire names/numeric constants from ROS names and QoS policy. |
| [Lifecycle](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubLifecycle.h) and [writer registry](../components/RaftROS/RTPS/runtime/autopub/RTPSDynamicWriterRegistry.h) | Reuse bounded ownership and device/sub-index identity patterns, not RTPS entity IDs, sequence numbers, or SEDP bookkeeping as a generic registry. |
| [RTPS runtime](../components/RaftROS/RTPS/RTPSTypes.h) | SPDP/SEDP, locators, ACKNACK, heartbeat, GUID routing, and `ros_discovery_info` stay entirely RTPS-specific. |
| [Root CMake](../CMakeLists.txt), [Linux Makefile](../linux_unit_tests/Makefile), [library manifest](../library.json) | Currently RTPS-oriented/unconditional; all supported entry points need explicit source and dependency isolation. |

The current data callback serializes the latest decoded record, then builds
RTPS user DATA and calls `sendto` directly for discovered peers. It does **not**
already provide a generic asynchronous transport queue. The planned boundary
must introduce and test buffer ownership/backpressure, not assume the older
design's history-cache description is the implemented behavior.

### 2.1 Minimal common contract

Keep one RaftROS SysMod and one DeviceManager registration path. Introduce a
small compile-selected backend interface, not an RMW framework or plugin loader:

- Lifecycle: start, service, stop, connection status and bounded diagnostics.
- Endpoints: create/destroy publisher using a ROS descriptor, returning an
  opaque bounded handle with a generation to reject stale callbacks.
- Data: publish serialized CDR with explicit length and timestamp metadata;
  return accepted, queue-full, disconnected, invalid-handle, or oversized.
- Subscriptions: create/destroy a bounded string subscription and deliver a
  neutral source identity plus decoded text on a documented callback context.

Descriptors own their strings and contain fully qualified ROS names (for
example `/raft/range_1_29`), ROS type identity, serialization kind, and semantic
QoS. Backends derive wire names and identities. Zenoh's selected profile may
also use the DDS-style type string; that does not justify leaking DDS endpoint
state into the common layer. Store verified ROS type hashes with the type
descriptor when required, with no runtime dependency on ROS type generation.

Use small common auto-publishing/CDR modules and separate RTPS and Zenoh
backend implementations. Prefer a compile-selected concrete implementation or
opaque implementation pointer; no dual-backend object, oversized union, or
global registry retaining both backends. Final filenames belong to Z2, not a
large prerequisite folder reorganization.

### 2.2 Existing subscription API

The public `StringMessageHandler` exposes a 4-byte RTPS writer entity ID and
12-byte GUID prefix; topic arguments also currently use DDS wire naming.
Preserve those APIs and behavior for RTPS applications. Add a transport-neutral
subscription API for portable applications and update ExampleDiscoverable to
use it during Z5. Do not manufacture RTPS GUIDs from Zenoh identities or
silently reinterpret existing callbacks. Clearly mark legacy APIs RTPS-only in
Zenoh builds and test both public-header/application compile paths.

## 3. ROS-on-Zenoh Compatibility Contract

Pin a compatibility profile before writing production protocol code: ROS
distribution, `rmw_zenoh_cpp` package version and commit, host Zenoh version,
protocol version, reference zenoh-pico version, ESP-IDF/Raft versions, and
host session/router configurations. The upstream references in section 9 are
moving branches, not this project's promised support matrix.

| Area | Required work and discriminating check |
| --- | --- |
| Session/link protocol | For a Raft-owned backend: bounded TCP framing, session handshake/identity, declarations, keepalive/lease, flow control, and fragment/reassembly behavior required by the pinned profile. Test arbitrary TCP segmentation, malformed lengths and reconnect against a real peer; delegate these to zenoh-pico only if dependency use is approved. |
| Data key expressions | Implement the pinned domain/topic/type/hash mapping. The current design uses `<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>`, with leading-slash normalization. Test nested namespaces, invalid/wildcard names, domain isolation, and matching name with wrong type/hash. Do not simply publish to `rt/...`. |
| Graph discovery | Declare ROS node (`NN`), publisher (`MP`), and subscription (`MS`) liveliness tokens under `@ros2_lv`, with session/node/entity identity, mangled names, enclave, type hash and encoded QoS as applicable. Test initial discovery, late-join liveliness queries, removal, lease expiry, and node-to-endpoint attribution. A generic Zenoh subscriber receiving bytes is insufficient. |
| Payload | Reuse standard ROS CDR, including encapsulation and alignment. Send serialized sample bytes only, not the RTPS DATA envelope or SEDP ParameterList. Compare with host serialization and typed deserialization for every currently supported mapped type and fallback. |
| Type metadata | Check in verified `RIHS01_...` values generated/read from the pinned ROS interface packages, including nested definitions. Test them against the host. An absent hash accepted by DDS is not a Zenoh compatibility strategy. |
| Attachments | Implement the pinned sequence/source-time/publisher-GID layout and identity derivation. The consulted Jazzy design specifies little-endian int64 sequence, int64 epoch nanoseconds, one GID-length byte, then 16 GID bytes. Verify against the selected release source and captures before fixing fixtures; do not assume all versions share this layout. |
| Time and identity | Separate poll-relative `Header.stamp` behavior from attachment source time. Never label uptime as Unix epoch nanoseconds. Specify unsynchronized-clock behavior, reboot/session identity and sequence reset rules from reference behavior; test wrap/reboot and duplicate device names. |

Keep wire encoders/parsers pure and Linux-testable. Implement only the
protocol/graph subset needed for this milestone, with explicit rejection of
unsupported features; do not attempt all ROS RMW APIs or copy RTPS discovery
machinery into Zenoh.

### 3.1 QoS scope

The first data proof is BEST_EFFORT + VOLATILE + KEEP_LAST with bounded depth.
This is an intermediate result, not automatic parity with every current
auto-publishing profile.

| Policy | Plan |
| --- | --- |
| BEST_EFFORT / VOLATILE | Latest-sample behavior with bounded queues. TCP may carry best-effort ROS samples; the label is not a demand for UDP. |
| RELIABLE / VOLATILE | Implement and test the pinned RMW's semantics, including bounded congestion handling. TCP alone does not justify claims of replay across disconnects or exactly-once delivery. |
| TRANSIENT_LOCAL | Existing `event` configurations require special attention. Implement the selected RMW cache/query or advanced pub/sub contract and late-join tests before advertising support; neither a liveliness token nor RELIABLE implies retained samples. |
| KEEP_LAST / depth | Bound payloads and history together and test depth limits. Validate offered QoS metadata matches actual behavior. |
| KEEP_ALL, manual liveliness, deadline/lifespan guarantees | Out of initial scope unless needed by an existing supported profile. Reject unsupported requests explicitly; never silently downgrade a configured profile. |

ROS QoS matching in Zenoh is not identical to DDS offered/requested matching.
Use the pinned RMW as the oracle. Record differences and test both a
best-effort and reliable host subscriber, rather than assuming the RTPS demo's
"default subscriber cannot match" rule applies. Complete the existing profile
matrix in Z5; any reduced-profile release needs an explicit scope decision.

## 4. Lifecycle, Scheduling and Resource Bounds

1. Device online: resolve mapping and reserve all required publisher slots,
   owned names, decode state, and buffers. Composite attachment must roll back
   cleanly on partial failure. Record desired endpoints even when offline.
2. Bus data: decode and serialize once per required message kind; copy into a
   bounded slot/mailbox or transfer clearly owned storage. Do not block the
   bus polling task on session opening, TCP writes, graph queries, or retries.
3. Transport service: one documented owner drains data and manages endpoints.
   If a worker is required, specify stack/priority, synchronization, queue
   limits and shutdown ordering. No concurrent use of shared scratch buffers.
4. Detach: unregister callbacks, invalidate the handle generation, discard
   queued samples, withdraw publisher tokens/declarations, then release owned
   memory when callbacks cannot reference it. Keep the board's node token.
5. Link loss: invalidate transport handles, keep bounded desired device state,
   reconnect with capped backoff, and recreate only still-online endpoints.
   Do not replay stale volatile samples. Devices removed while disconnected
   must not return as phantom publishers.
6. Shutdown/reboot: withdraw entities when possible; otherwise rely on tested
   session leases. A returning device keeps its topic name but gets valid
   current-session identities, without collisions or stale graph attribution.

Start with the current 16 dynamic publisher-slot limit, not 16 devices:
composites use multiple slots. Size Zenoh names/tokens separately from the
current 96-byte DDS topic/type buffers because hashes and graph metadata add
length. Bound peers, subscriptions, tokens, pending operations, sample size,
fragments, queue depth, and aggregate retained-history bytes at build time.
Oversize input and resource exhaustion must produce visible counters/errors,
not truncation, leaks, watchdog resets, or unbounded retry queues.

Z0 records comparable release-build RTPS measurements: ELF/app size, map file,
static RAM, minimum/largest free heap, task stacks, sockets, CPU/bus callback
latency, and network rate at idle and at the configured endpoint limit. Set
absolute Zenoh budgets against the actual board/partition, including OTA
headroom if applicable, before Z3. No PSRAM requirement by default; no claim
that Zenoh is smaller until measured. The old `0x1435c0` firmware figure is a
historical whole-application measurement, not a transport-only comparison.

Initial testing is on a trusted LAN. Domain IDs and key expressions are not
authentication. Keep listeners/firewall exposure explicit, test malformed
remote inputs, and document encryption/authentication as unsupported unless
deliberately implemented and budgeted. Do not present this as a secured WAN
deployment.

## 5. Build and Configuration Design

The following names are **proposed**, not functioning build switches today:

| Profile | Contents |
| --- | --- |
| `RAFTROS_TRANSPORT=RTPS` | Common SysMod/mapping/CDR plus current RTPS runtime only. Preserve no-option build behavior and existing SysTypes. No Zenoh download, configure step, headers, objects, or state. |
| `RAFTROS_TRANSPORT=ZENOH` | Same common pipeline plus Zenoh wire/session and ROS-compatibility adapter only. No RTPS runtime objects, includes, sockets, SPDP/SEDP timers, or discovery state. |

Use one validated CMake cache enum as the authority; generate the internal
compile definition/header from it. If a menuconfig choice is added, it must
feed that same choice, not create independently conflicting switches. Reject
unknown values and attempts to select both. Link-time garbage collection is
useful, but is not a substitute for conditional source/dependency selection.

Apply the selection to:

- Root component source lists and include/dependency lists.
- ExampleDiscoverable's existing Raft `features.cmake` integration, preserving
  existing defaults and documenting separate RTPS/Zenoh build directories.
- Linux unit/standalone targets, separating common, RTPS, and Zenoh test
  dependencies. Backend-specific object directories prevent stale reuse.
- PlatformIO/library packaging if advertised as supported. The current
  manifest's source discovery must not compile both backends; a Zenoh-only
  dependency must never become an unconditional manifest dependency.
- The ESP unit-test component build, so tests do not force RTPS into a
  Zenoh firmware build. Common tests must compile with neither wire backend.

If zenoh-pico is approved, fetch/pin it **only** for the Zenoh profile and
disable unused platform/link/features after confirming the graph/QoS subset
still works. Otherwise compile only the Raft-owned bounded implementation.
Avoid requiring two selectable Zenoh engines in a release; Z0 chooses one.

Retain shared enable/domain/node/namespace/autoPublish configuration. Proposed
Zenoh-specific settings cover remote endpoints, session mode, connect timeout,
lease and bounded reconnect policy, with validated limits. Do not add a JSON
`transport` selector that suggests a missing backend can be enabled at runtime.
Report compiled backend and compatibility profile through status diagnostics.
Wrong-backend protocol settings should produce a clear diagnostic.

### 5.1 Why not a dual-backend binary now?

A runtime selector still carries both code stacks, dependencies and static
storage, and doubles lifecycle/configuration interactions to test. There is
no demonstrated deployment need that offsets that cost. Revisit only after
single-backend measurements and a concrete use case; require an explicit
opt-in dual build and define reboot-only versus live switching separately.
Nothing in the small common API requires that feature now.

## 6. Implementation Slices and Gates

All statuses are **not started**. Work in order; keep each extraction small
and validate RTPS before continuing. Z0/Z1 can use a throwaway native harness
without restructuring production RaftROS.

| Slice | Deliverables | Exit gate |
| --- | --- | --- |
| **Z0: Baseline and feasibility** | Re-run current RTPS tests/demo; record versions/resources. Pin the ROS-on-Zenoh profile, check protocol documentation, compare Raft-owned and dependency-based effort, and document routerless topology and exact limits. | Maintainer-visible dependency/license decision, feasible standalone topology candidate, absolute resource budgets, no hidden router/agent assumption. If not feasible, stop and report options. |
| **Z1: Native vertical proof** | Implement the smallest session, node/publisher token, key/hash, attachment and CDR path for `/chatter` and one `Range` publisher in the Linux harness. Use host reference fixtures; router topology is a diagnostic control. | Unmodified pinned `rclpy` receives typed samples; graph associates endpoints with the named node; late join/remove works. Repeat directly without a router, including several ROS processes. Raw Zenoh delivery alone fails the gate. |
| **Z2: Common pipeline with RTPS adapter** | Extract semantic mapping/CDR and endpoint operations; isolate RTPS members from public headers. Add fake-backend tests for lifecycle, ownership, capacity, and bus callback handoff. Leave RTPS wire runners intact. | Current RTPS tests and ESP demo pass with unchanged user-facing behavior. Common code compiles without RTPS includes. Establish any legacy subscription API adapter before moving callers. |
| **Z3: Build profiles and ESP session** | Implement mutually exclusive source selection across supported builds; add the selected Zenoh engine and adapter to ESP32; implement connection servicing, status and bounds. | Fresh RTPS and Zenoh builds succeed; map/dependency checks prove exclusion. ESP32 directly exchanges static typed data with the selected host topology within Z0 budgets; repeated disconnects do not block the bus. |
| **Z4: Dynamic sensor auto-publishing** | Connect DeviceManager attach/data/detach to common descriptors and Zenoh publishers; all current mappings, composite endpoints, fallback and existing naming overrides use shared serializers. | VL6180 `Range`, a composite sensor, IMU, and fallback fixtures work; detach/replug and detach-during-outage recover without stale publishers. Host checks units, stamps, fields and publisher counts. |
| **Z5: Supported behavior parity** | Portable string subscriptions and ExampleDiscoverable integration; complete existing QoS-profile behavior, including bounded transient-local history where requested; test two boards and multiple subscribers. | `/chatter_in` and `/chatter_in2` route independently. Profile matrix passes or an explicit scope change is approved; no silent QoS downgrade or fabricated RTPS source identity. |
| **Z6: Hardening and release evidence** | Stress, malformed-input and outage tests; Linux/ESP CI build matrix; repeatable host instructions and backend-labeled dashboard/Foxglove demo; record resource comparison. | Section 8 acceptance passes for both isolated builds and devdocs distinguish measured support from deferred features. |

Do not block Z0/Z1 on the remaining RTPS modularization cleanup or Task D.
Their regression baselines remain relevant, but they are not Zenoh protocol
prerequisites. Services, parameters, actions, and live backend switching are
outside this milestone.

## 7. Validation Workflow

Reuse [Linux tests](../linux_unit_tests/main.cpp) and the model-publisher
approach in [the wire-diff guide](RaftROS-model-publisher-wire-diff.md).
Add a focused Zenoh fixture/test group alongside existing tests only when
needed to preserve dependency isolation, not a second copy of mapping tests.

Existing RTPS baseline command, from `linux_unit_tests` on Linux/WSL:

```bash
make -j"$(nproc)" all standalone && ./linux_unit_tests
```

Run clean or separate-profile builds when comparing backends. The Makefile
currently fetches RaftCore; pin/record its revision for repeatability. The
reported 912/912 result is historical until this command is rerun.

Host prerequisites for the **future** Zenoh integration tests:

```bash
source /opt/ros/jazzy/setup.bash
ros2 daemon stop
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=0
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset FASTDDS_BUILTIN_TRANSPORTS ROS_LOCALHOST_ONLY
```

Install the pinned RMW package beforehand. Configure `ZENOH_SESSION_CONFIG_URI`
for the Z1-verified direct topology in every host process, with reachable LAN
listen/connect addresses and no automatic router dependency. Do not claim a
universal routerless command before that experiment passes. For the separate
router control test, use a recorded `ZENOH_ROUTER_CONFIG_URI` and
`ros2 run rmw_zenoh_cpp rmw_zenohd`; make IPv4 reachability explicit for ESP32.
Never use an unpinned `latest` router image as release evidence.

Use a direct `rclpy` graph/data test first, then CLI with a daemon restarted
under the same RMW/domain/session configuration. Inspect publisher endpoint
counts and node attribution, not just topic names: a monitoring subscriber
can keep a topic in the graph after the last publisher disappears.

Extend capture tooling for the configured Zenoh endpoints and decoded
key/token/attachment/CDR records. Existing UDP 7400-7500 filters and SEDP PID
diffs do not observe Zenoh. Compare semantic records and normalized dynamic
identities/timestamps, not entire transport packets byte-for-byte.

## 8. Acceptance Matrix

Targets below are planned tests, not achieved measurements:

| Area | Required evidence |
| --- | --- |
| Isolation | Default RTPS builds without any Zenoh SDK present; Zenoh builds without compiling/linking RTPS. Check compile commands, dependency graph, ELF/map symbols, and traffic (no RaftROS RTPS UDP discovery in Zenoh mode). Common tests compile independently. |
| Graph and data | Node attribution and typed `/chatter` plus dynamic sensor delivery; late-started subscriber and late-started firmware both work, without a bridge or router. Test multiple independent host processes. |
| Hot plug | On a healthy link, publisher appears within 2 s of DeviceManager online callback and is withdrawn within 10 s of offline callback. Report scan/detection time separately. Replug preserves names, not stale handles. |
| Serialization | Golden CDR and native host deserialization for all mapped kinds, fallback and composites; test maximum payloads, insufficient buffers, hashes, units and timestamps. No per-device ROS host codegen. |
| QoS and input routing | Test all four existing auto-pub profiles, actual retained history/depth, late join, overflow reporting, and two independently routed string subscriptions. Unsupported requests fail explicitly. |
| Identity/isolation | Two boards with distinct configured namespaces, deliberate duplicate names, wrong domain/type/hash, and session restart. No cross-topic dispatch or stale endpoint ownership. |
| Recovery | At least 100 attach/detach cycles, at least 20 WiFi/session reconnect cycles, host restarts and a one-hour mixed-sensor soak. No monotonic heap loss, watchdog reset, stale publishers, or unbounded queues. Recovery deadlines are fixed from selected lease/backoff settings in Z0. |
| Resource limits | Test full publisher/subscription/peer capacity, composite partial-allocation rollback, slow consumers, oversize/malformed input and fragmentation boundaries. Record min heap/largest block and task high-water marks against Z0 budgets. |
| Regression/demo | RTPS Linux tests and ESP demo remain passing with their recorded Task D caveat. Zenoh graph failures are investigated independently, not waived as WSL/FastDDS behavior. Validate native Linux first, then WSL2 and Foxglove Bridge using the same RMW. |

## 9. Documentation and Upstream References

At each slice, update [development status](RaftROS-development-status.md) with
commands, platform/version tuple, outcome and unresolved gates. Keep this plan's
status table current. Update the overview, auto-publishing contract, demo plan
and wire-diff guide when their behavior changes. Preserve historical RTPS
findings in [the earlier stage plan](RaftROS-next-stages-implementation-plan.md)
and [RTPS modularization plan](RTPS-modularization-investigation-and-plan.md).
Implementation slices must also update the relevant example READMEs, build
instructions and root README; this planning change does not make their current
RTPS commands work for Zenoh.

Upstream material consulted 2026-09-17; pin immutable revisions in Z0:

- [rmw_zenoh README](https://github.com/ros2/rmw_zenoh): host configuration,
  non-RMW API compatibility obligations, DDS-bridge distinction, distro caveat.
- [Jazzy ROS-on-Zenoh design](https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md):
  key expressions, graph tokens, CDR, attachments and QoS behavior.
- [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico): constrained-device
  implementation, ESP-IDF support and limited peer examples. This is not proof
  of routerless ROS interoperability or permission to incorporate its source.

### Outstanding decisions before production implementation

1. Confirm feasibility under the preserved Raft-only/source-license policy,
   or explicitly approve a separate zenoh-pico dependency.
2. Demonstrate the strict routerless topology. Any relaxation of the
   no-separate-process requirement needs explicit approval.
3. Record the supported version tuple, absolute board budgets, supported link
   modes and reconnect/expiry deadlines from Z0/Z1 evidence.

Until those gates pass, RTPS remains the only supported backend.