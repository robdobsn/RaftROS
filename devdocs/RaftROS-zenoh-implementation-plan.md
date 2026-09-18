# RaftROS Zenoh Alternative: Design and Implementation Plan

**Resume note (2026-09-18):** The common sample runner/RTPS emitter slice is
reviewed and closed (see [Shared Sample Dispatch](#shared-sample-dispatch-2026-09-18)),
including the first ESP32-S3 firmware compile of the modified wrapper. The
[WSL agent handoff](RaftROS-WSL-agent-handoff.md) records the workspace move and next steps.

**Date:** 2026-09-18
**Status:** Z0/Z1 in progress; Z2 has shared sensor mapping/CDR and synchronous sample dispatch with an RTPS emission adapter. String/Range, lifecycle and bounded probe-storage proofs pass. The RTPS firmware compiles for ESP32-S3; DeviceManager lifecycle extraction, isolated backend builds and live device measurements remain pending.
**Scope:** A native ROS 2 Zenoh backend for Raft ESP32 firmware, preserving
DeviceManager-driven sensor auto-publishing and the existing RTPS backend.

This is the forward plan for the next transport milestone, ahead of services
and parameters. Phases 1-4 remain completed **RTPS** work, not evidence of Zenoh
support. See [development status](RaftROS-development-status.md), the
[overview](RaftROS-overview.md), and the implemented
[auto-publishing design](RaftROS-auto-publishing-design.md).

## Implementation Record (2026-09-17)

The metadata preparatory slices are implemented in
[ZenohROSCodec.h](../components/RaftROS/Zenoh/ZenohROSCodec.h), original
Raft-owned C++17 code with no RTPS, RaftCore, ESP-IDF or Zenoh SDK dependency:

- Encodes/decodes the pinned 33-byte publisher attachment, preserving signed
  sequence/source-time values and the 16-byte publisher GID.
- Formats ROS topic data keys from a fully qualified topic, DDS-style message
  type and `RIHS01_...` hash, without an RTPS `rt/` prefix.
- Formats node (`NN`) graph-liveliness tokens, including namespace/enclave
  mangling and the repeated node ID.
- Formats publisher (`MP`) and subscription (`MS`) tokens with distinct
  node/entity IDs, fully qualified topics, types/hashes and canonical QoS.
- Serializes a bounded QoS subset: RELIABLE/BEST_EFFORT, VOLATILE/TRANSIENT_LOCAL,
  KEEP_LAST with positive depth, AUTOMATIC liveliness and infinite durations.
  This describes metadata only, not implemented history or delivery guarantees.
- Uses caller-owned output storage, no dynamic allocation, and explicit input
  and capacity checks. The isolated `make zenoh-test` target builds only the
  codec tests, without fetching RaftCore or linking RTPS.

The next preparatory slice,
[ZenohROSIdentity.h](../components/RaftROS/Zenoh/ZenohROSIdentity.h), derives
publisher/subscription GIDs from our canonical endpoint tokens. Both generated
identities match the native RMW graph, and outgoing attachments no longer
take a graph-supplied GID as input.

The TCP publishing slice now connects the metadata/identity code and existing
Raft CDR code to a Raft-owned POSIX socket. Native ROS discovers
`/raft_test/raft_fixture` and receives either `std_msgs/msg/String` on
`/raft_test/chatter` or `sensor_msgs/msg/Range` on `/raft_test/range`, in
separate probe modes, then observes token withdrawal. A second ROS process
started after publishing begins also discovers and receives live samples.
Range reuses the existing auto-publishing serializer with synthetic decoded
poll records; it is not a hardware sensor test. The earlier host-library
metadata control remains separate. Other sensor types, subscriptions,
reconnection, firmware integration and the full Z0/Z1
acceptance gates remain open; no RTPS extraction or dependency-policy change
was needed for those host proofs. The shared sensor extraction below now
reuses the same implementation across both paths without changing wire logic
or the dependency policy.

### Shared Sensor Mapping and CDR (2026-09-17)

The first preparatory Z2 extraction is implemented:

- [AutoPubClassMap.h](../components/RaftROS/AutoPub/AutoPubClassMap.h) owns
  message kinds, wire type names and class/composite/override/fallback mapping
  in `RaftRuntime::AutoPub`.
- [AutoPubCDRSerializer.h](../components/RaftROS/AutoPub/AutoPubCDRSerializer.h)
  and [AutoPubCDRSerializer.cpp](../components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp)
  own descriptor/context types, field readers, unit scaling and CDR encoding.
  The implementation was moved mechanically; mapping order, enum values,
  descriptor layouts, default arguments and serialization behavior are unchanged.
- The old RTPS headers retain type aliases and inline forwarding functions.
  Existing callers keep their source API; this is not precompiled C++ ABI
  compatibility. Explicit external source lists must replace the old serializer
  `.cpp` with the common one. Root ESP-IDF CMake, Linux tests/standalone, and
  Docker recipes now compile only the common implementation. ESP unit tests
  consume the root component through `REQUIRES RaftROS`.
- The Zenoh Range probe uses only neutral sensor names/includes. Its Docker
  build copies no RTPS headers. The common serializer also passes strict C++17
  syntax compilation without any project include paths.
- RTPS standalone builds and **929** Linux assertions pass (921 previous
  checks plus eight shared-API compatibility checks and five compile-time type
  identity assertions). Native typed/raw Range, late observation and withdrawal
  pass under ASan/UBSan; Docker session tests remain **1245/1245** and storage
  allocation-failure checks pass.
- Repeated `-Os` resource reports still show text+data **41023 B RTPS /
  33008 B Zenoh**, Zenoh main frame **400 B** and owned storage **19192 B**.
  ELF files are now 71448/49480 B respectively; renamed symbol metadata is not
  a firmware flash saving. Reporting requires the neutral serializer symbol
  and clears generated `.su` records before rebuilding to exclude stale paths.

This does not extract DeviceManager lifecycle/ownership, endpoint registries,
topic construction, QoS policy or the transport interface. RTPS wire runners
and production callbacks are unchanged. Z2's complete RTPS adapter and target
regression gates remain open; Z3 build selection is not implemented.

### Shared Sample Dispatch (2026-09-18)

- [AutoPubSampleRunner.h](../components/RaftROS/AutoPub/AutoPubSampleRunner.h)
  is a header-only, transport-neutral runner. It validates a borrowed decoded
  batch (`recordCount <= capacity / recordSize` before pointer arithmetic),
  selects the latest record, reads its leading `timeMs`, serializes up to two
  caller-owned outputs (primary then secondary) and calls a synchronous
  publish callable per non-empty payload. A failed output does not suppress
  the other. `run()` returning true means a valid batch, not delivery.
- `AutoPubPublishResult` reports `NotAttempted`, `Accepted`, `QueueFull`,
  `Disconnected`, `InvalidHandle`, `Oversized` or `SendFailed`. These describe
  one payload's outcome; no queue or QoS guarantee is implemented behind them.
- Ownership: all storage is borrowed for the duration of `run()`. The callable
  must send or copy the payload before returning and must not mutate the batch
  or outputs. There is no allocation, worker, locking or lifetime enforcement.
- [RTPSAutoPubSampleEmitter.h](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubSampleEmitter.h)
  adapts one payload to a VOLATILE best-effort RTPS writer: one sequence number
  per non-empty sample (even with no peers or failed sends), sent to every
  peer; any accepted peer makes the aggregate `Accepted`, otherwise the last
  failure, or `Disconnected` with no peers. A null/released entry sends nothing
  (`InvalidHandle`). The entry pointer is **not** a generational handle.
- `RaftROS::autoPubOnDeviceData` delegates to the runner and emitter. Padding,
  generated decoding, pre-checks, per-device buffers, synchronous `sendto`, packet
  arguments and primary-before-secondary order are unchanged; logs add numeric
  publish outcomes, and an invalid decoded batch logs a rate-limited warning.
- The Linux Zenoh probe's `serializeRangeSample` reuses the runner for one
  synthetic record. That proves runner/CDR reuse, not a common Zenoh adapter.
- Evidence: **979** Linux assertions pass (929 plus 33 runner and 17 emitter
  checks), also under ASan/UBSan. Docker session **1245/1245**, allocation-failure
  and native sanitized Range checks passed on 2026-09-17. Host text+data is
  **41023 B RTPS / 33062 B Zenoh** (+54 B for the runner); owner storage and
  main frame are unchanged at 19192 B / 400 B.
- **First firmware compile (2026-09-18):** `examples/ExampleDiscoverable` built for
  ESP32-S3 with `raft build --no-docker -e ~/esp/esp-idf-v6.0.2`, compiling the
  modified `RaftROS.cpp` with no RaftROS warnings. RaftCore `feb4f77`, RaftSysMods
  `a1af7f1`, RaftWebServer `0f04a21`, RaftI2C `0792b2f` (all fetched `@main`);
  IDF 6.0.2 resolved mdns 1.13.1 and littlefs 1.22.3 (the committed lock was
  left at 6.0.0 versions). App image 1257619 B (`0x133100` padded), 29% free in
  the `0x1b0000` slot; DIRAM static 95279 B; `libRaftROS.a` 40042 B, of which
  39584 B flash code and 38 B static RAM. Compile only: not flashed or run on a board.

The DeviceManager callback/detach ownership, shared send buffer and
registry/peer concurrency concerns are unchanged; fake-backend tests do not
prove them safe. Raw decoding, decoder allocation, attach/detach and transport
lifecycle still live in RaftROS.

### Pinned Reference and Observed Baseline

| Item | Evidence |
| --- | --- |
| Metadata reference | `rmw_zenoh_cpp` Jazzy 0.2.11, commit `8c1fe8ef412bca5e6ac64f320468c70dcb03fc52`; [immutable design document](https://github.com/ros2/rmw_zenoh/blob/8c1fe8ef412bca5e6ac64f320468c70dcb03fc52/docs/design.md). The profile is also identified in the codec header. |
| Fixture provenance | Topic/node/endpoint examples from the pinned design, independently specified attachment bytes, and QoS behavior verified against the pinned RMW's `liveliness_utils.cpp` and `qos.cpp`. Original implementations, no upstream source copied into firmware. Unit fixtures are separate from the native host control below. |
| Compiler/environment | Ubuntu WSL, g++ 13.3.0 (`13.3.0-6ubuntu2~24.04.1`), CMake 3.28.3. Native Jazzy RMW/Zenoh vendor/FastDDS/sensor-msgs packages were not found by `dpkg-query`. |
| Zenoh unit checks | 2218 passed, 0 failed with C++17, `-Wall -Wextra -Werror -pedantic`, and again with AddressSanitizer/UndefinedBehaviorSanitizer. Includes fixed upstream GID vectors, descriptor-field changes and invalid/maximum input checks in addition to codec tests. |
| GID reference comparisons | Docker's reference-enabled test binary passes 3222 checks: the normal suite plus 1004 direct hash comparisons with the pinned RMW helper, covering shortest tokens and the 128/129, 240/241, 64-byte stripe and 1024-byte block boundaries. |
| RTPS unit baseline | Rebuilt and ran successfully: 921 passed, 0 failed. RaftCore revision used: `feb4f77f1778be04fbf1789bdaa6c84ec4e8fe5c`. |
| RTPS standalone baseline | Initial build exposed stale includes and runtime API references in `raftros_standalone.cpp`. Resolved in the 2026-09-17 follow-up: current runtime include paths, discovery namespace, ACKNACK declarations and DATA flags/payload helper. Standalone compilation/linking and the 921 RTPS tests pass; live ROS smoke testing remains unverified. |
| Native metadata/identity control | Passed in Docker against pinned `rmw_zenoh_cpp` 0.2.11 and Zenoh Python 1.8.0: graph attribution/QoS, independently derived publisher/subscriber GID equality, bidirectional typed String data with Raft-generated attachment identity, endpoint withdrawal and node removal, using direct loopback with no router. |
| Session/network unit checks | 1245 passed with C++17 strict warnings and again with AddressSanitizer/UndefinedBehaviorSanitizer. Includes nine new same-object reset checks following removal of a measured large stack temporary, in addition to wire/handshake/sequence coverage. |
| Native TCP control | Raft-owned client completed INIT/OPEN with the pinned RMW peer, negotiated 4096-byte batches and survived 12000 ms with four-second leases. Received 12 peer keepalives; final run sent 112 bytes and received 118 bytes. No upstream Zenoh library is linked into the C++ probe. |
| Native TCP publication | Passed, also with the publisher built under ASan/UBSan: named ROS publisher, correct type/QoS/GID, typed Raft-CDR String delivery with increasing attachment sequence, a second late ROS process, and publisher/node withdrawal. Final sanitized run queued 22 samples, sent 4534 bytes and received 118 bytes over 12 seconds. |
| Native TCP Range publication | Passed, including an ASan/UBSan build of the probe and reused serializer: verified Jazzy Range type hash; six deterministic CDR fixtures versus native serialization; typed/raw live samples; all fields, scaling, timestamps, QoS and GID; late ROS process and withdrawal. Final sanitized run queued 22 samples, sent 5282 bytes and received 118 bytes over 12 seconds. |
| Scripted wire lifecycle | Passed locally and with an ASan/UBSan probe in Docker: correlated current/current-future replies, exact/prefix/no-match, non-token/future-only interests, cancellation, four-entry capacity/reuse and absence after withdrawal. Separate EOF, silence, scope/wildcard rejection, overflow and CLOSE cases terminate with expected diagnostics. |
| Native explicit restarts | Three Range rounds pass against pinned RMW, also with a sanitized probe: abrupt publisher kill removes publisher/node entries, ROS peer shutdown terminates the probe, then a new peer/probe resumes with a new GID and sample sequence one. This is process restart, not automatic reconnect. |
| Host release baseline | Matching `-Os`/section-GC builds: RTPS text+data 41023 bytes, Zenoh probe 33008 bytes after ownership changes. Zenoh main frame falls from 19568 to 400 bytes; 19192 bytes of persistent storage move into one checked allocation, not out of RAM. These are non-equivalent Linux programs, not firmware footprints. |
| Still unmeasured | Other sensor serializers, live DeviceManager/hot-plug, subscriptions, general upstream discovery-interest patterns, automatic reconnect, multi-peer resilience and ESP32 resources/hardware. Provisional firmware targets below are not measured results or a completed Z0 gate. |

### Initial Codec Contract

The formatters accept string views valid for the duration of the call; inputs
must not overlap output storage. Names are a deliberately bounded subset:
fully qualified ROS paths with identifier segments, at most 255 bytes, and
message wire types of the form `package::msg::dds_::Message_` at most 128 bytes.
Empty/root node namespace and enclave map to `%`; root is not a valid topic.
Session IDs are nonzero lowercase hex strings of 1-32 characters. Type hashes
must have the `RIHS01_` prefix and 64 lowercase hex digits; syntax validation
does not establish that a hash matches a message definition. No type-hash
registry or Humble compatibility is supplied yet.

On string-format failure a non-null, nonempty output is set to an empty
string; no truncated key/token may be used. Attachment encode returns 33 on
success or zero without writing on failure. Decode requires exactly 33 bytes
and GID length 16 and leaves its result unchanged on failure; trailing bytes
and future profile extensions are not accepted implicitly. The codec preserves
integer values but supplies no timestamp clock, sequence policy or session-ID
generator. Those remain session/ROS-adapter responsibilities; endpoint GID
derivation from supplied identities is now implemented separately below.

`NodeIdentity` and `Endpoint` borrow string views for the formatting call;
they are not a persistent device registry. `formatEndpointToken` supports
only `Publisher`/`Subscription` kinds and validates the same name/hash rules
as the node and topic-key helpers. Endpoint IDs are supplied by the caller;
uniqueness and lifecycle remain the future registry's responsibility.

`QoS` is a metadata-only subset with fixed KEEP_LAST, AUTOMATIC liveliness,
and infinite deadline/lifespan/lease values. Defaults are RELIABLE, VOLATILE,
depth 42, as verified in the pinned RMW's actual `qos.cpp`; its general design
prose should not override those source defaults. `formatQoS` retains all six
colon-separated groups and their comma separators, omitting only default
values. ROS reliability numbers are 1=RELIABLE and 2=BEST_EFFORT, not the
opposite Zenoh transport enum order. Depth zero and invalid enum values fail;
system-default resolution, KEEP_ALL and finite durations are not accepted by
this API. The canonical default is `::,:,:,:,,`. Golden tests cover default,
sensor and transient-local forms, including exact separator placement. Use
`formatQoS`, not hand-edited strings, for production.

### Raft-Owned Endpoint GIDs

`ZenohROSIdentity::deriveEndpointGid(node, endpoint, output)` first validates
and formats the complete endpoint liveliness token using `ZenohROSCodec`.
It hashes those bytes, excluding the trailing NUL, with unseeded XXH3-128 and
the algorithm's default 192-byte secret. The 16 output bytes are the low
64-bit result followed by the high result, **each little-endian**, matching
the pinned RMW on our little-endian Linux/ESP32 targets. This is not the
big-endian hexadecimal representation used by generic xxHash tools, a DDS
GUID, or the ROS message-type hash.

The original scalar implementation follows the
[published xxHash algorithm specification](https://github.com/Cyan4973/xxHash/blob/v0.8.2/doc/xxhash_spec.md).
Algorithm constants are fixed inputs. Upstream's BSD-licensed implementation
is not incorporated into Raft source; its unchanged helper is compiled only
in the Docker reference test. The implementation uses 32-bit limb products
and unsigned 64-bit arithmetic, with no `__int128`, SIMD, dynamic allocation,
custom seed, or external hash library. Only input lengths reachable through
validated endpoint descriptors are exposed; this is not a generic xxHash API.

The token buffer has a compile-time bound derived from the codec's name/type/
QoS limits (currently 1347 bytes, covering a maximum 1345-byte token plus NUL).
Medium-input and long-input paths cover the entire descriptor range,
including the final overlapping stripe and block scrambling. Invalid inputs
leave the caller's GID unchanged; a truncated token is never hashed. Stack
and performance measurements on ESP32 remain pending, despite using portable
arithmetic and fixed storage.

The native test compares both endpoint GIDs with the graph; the separate
reference test compares 1004 valid tokens directly with
[`simplified_XXH3_128bits` at the pinned RMW revision](https://github.com/ros2/rmw_zenoh/blob/8c1fe8ef412bca5e6ac64f320468c70dcb03fc52/rmw_zenoh_cpp/src/detail/simplified_xxhash3.cpp).
Four captured vectors remain in the dependency-free local suite: minimum
token, fixed publisher, fixed subscription and maximum token. Tests also
check changes to every node/endpoint descriptor field and unchanged output
on invalid input. The default local test does not compile the reference
helper; only `RAFTROS_ZENOH_GID_REFERENCE` in the Docker test enables it.

This is a non-cryptographic identity hash, not authentication or a guarantee
against intentional collisions. The session-ID generator and entity-ID
allocator must still ensure different live entities receive distinct input
identities. Reusing the same entire token intentionally produces the same
GID. Reboot, detach/re-attach and reconnect identity policy remain session/
registry work, not something hashing alone solves.

### Native Host Metadata Control

[Dockerfile.zenoh](../linux_unit_tests/Dockerfile.zenoh) builds a host-only
reference environment. It pins the ROS base image by digest, the RMW by
commit, and `eclipse-zenoh==1.8.0`. The pinned RMW binary package was not
available in the configured apt repository, so the RMW is built from source.
Its vendor build uses apt Cargo 1.75's compatibility branch; Zenoh Python is
built using its required Rust 1.93.0. The image is a test tool, not firmware.
Initial builds require network access, substantial Docker disk space and
several minutes; subsequent source/test edits reuse the toolchain layers.

Observed host versions: `ros-jazzy-rclpy` 7.1.12-1noble.20260902.053513 and
`ros-jazzy-std-msgs` 5.3.8-1noble.20260902.022418. The pinned RMW vendor selects
zenoh-c `b31348fa7f94f44f1f7b049c111a710e970a2725` (Zenoh
`2687c51352121f006e3a603ce07925a8ad0b295c`) with the older Cargo and zenoh-cpp
`af381b420cc8837ac7da42c9984594ef8f110e90`. Apt/rosdep and transitive Python
build dependencies are not a fully frozen package snapshot; the test checks
the RMW version at runtime and records the tested tuple rather than claiming
bit-for-bit reproducible images.

[zenoh_metadata_interop.py](../linux_unit_tests/zenoh_metadata_interop.py)
starts a native ROS RMW peer on `tcp/127.0.0.1:17447`, with multicast scouting
disabled and no router connection, and attaches an upstream Python Zenoh
client directly. `zenoh_codec_tests --fixture <session-id>` provides the
actual C++ node/publisher/subscription tokens and data key; the Python test
also receives both Raft-derived endpoint GIDs and does not independently
rebuild those strings or identities. `--attachment <session-id> <sequence>`
derives the fixture publisher's identity again inside C++; it no longer
accepts a GID returned by the host graph. Domain 23 and `/raft_test` keep
the fixture identifiable. Docker's `--network none` confines the test to its
own loopback, with no host ROS daemon or external router required.

Passing checks:

- Native RMW attributes both endpoint kinds to `/raft_test/raft_fixture` and
  decodes `std_msgs/msg/String`, BEST_EFFORT, VOLATILE, KEEP_LAST depth 5.
- Raft-derived publisher and subscription GIDs match the native graph's
  endpoint GIDs. The graph is an assertion oracle only, not a source of IDs
  fed back into attachment generation.
- A typed ROS subscription receives data with our key and C++-encoded
  attachment using the Raft-generated GID. Source timestamp zero and
  publication sequence are checked.
- A native ROS publisher delivers a typed String to the Python subscriber
  using the same C++ key.
- Undeclaring endpoint tokens removes the fixture's publishers/subscriptions
  even while a monitoring subscription remains. Removing the node token
  removes its named node from the graph.

Limits: CDR is generated/deserialized by host ROS libraries. GID derivation is
now verified against both the pinned helper and native graph. This rclpy
callback exposes sequence/timestamps as a dictionary but no publisher GID,
so callback-level GID equality is not asserted. Only live volatile
String traffic and one native RMW context are exercised; retained history,
late-started independent processes, all sensor serializers, raw Zenoh framing,
and ESP32 operation remain separate gates. No claim of Z1 completion follows.

Run from the repository root (Docker Desktop/Linux engine required):

```bash
docker build --progress=plain -f linux_unit_tests/Dockerfile.zenoh -t raftros-zenoh-metadata-test .
docker run --rm --init --network none raftros-zenoh-metadata-test timeout 90s /bin/bash -c 'source /reference/install/setup.bash; exec /opt/zenoh-test/bin/python /test/zenoh_metadata_interop.py'
```

The container exits nonzero on an unmet assertion or timeout and is removed.
No workstation ROS installation, firmware SDK, or source bind mount is needed.

### Raft-Owned TCP Handshake Control

Implemented on 2026-09-17:

- [ZenohStreamFramer.h](../components/RaftROS/Zenoh/ZenohStreamFramer.h):
  fixed-capacity receiver for TCP's two-byte little-endian batch length.
  Handles split length headers, partial bodies and consecutive batches;
  zero/oversize lengths fail until explicitly reset.
- [ZenohTCPSession.h](../components/RaftROS/Zenoh/ZenohTCPSession.h):
  socket-independent client state machine for INIT/OPEN, opaque cookie echo,
  negotiated limits, KEEPALIVE and incoming CLOSE. State/errors are observable.
- [zenoh_session_probe.cpp](../linux_unit_tests/zenoh_session_probe.cpp):
  Linux nonblocking IPv4 socket adapter using `poll`, `getrandom`, monotonic
  time, partial-send accounting and EOF/error handling. It closes its socket
  on every exit and has no reconnect loop. No ROS, RTPS, zenoh-pico or desktop
  Zenoh dependency is linked into this executable.
- [zenoh_session_tests.cpp](../linux_unit_tests/zenoh_session_tests.cpp):
  isolated, hardware-free framing and control-state regression tests.

Wire facts are pinned to Zenoh
`2687c51352121f006e3a603ce07925a8ad0b295c`, using its documented
[INIT](https://github.com/eclipse-zenoh/zenoh/blob/2687c51352121f006e3a603ce07925a8ad0b295c/commons/zenoh-protocol/src/transport/init.rs),
[OPEN](https://github.com/eclipse-zenoh/zenoh/blob/2687c51352121f006e3a603ce07925a8ad0b295c/commons/zenoh-protocol/src/transport/open.rs)
and transport formats as behavioral references, not copied implementation.

| Setting | Current bounded prototype |
| --- | --- |
| Protocol/mode | Version 9, client mode, one TCP link; no multicast scouting. |
| Identity | Caller supplies a nonzero 16-byte Zenoh ID; probe uses OS entropy. Rejects zero peer IDs and self-connection, including equivalent shorter peer-ID encoding. This is not cryptographic authentication. |
| Batch | Offer 4096 bytes; accept peer reduction to 64-4096. An INIT ACK omitting size fields decodes as the protocol defaults, not acceptance of the smaller offered buffer. Unsupported larger batches fail. |
| Resolutions | Offer 32-bit frame sequence and request IDs; accept 8/16/32-bit reductions. Validate peer OPEN sequence fits the agreed resolution. Outgoing reliable frames start at zero; RX tracks reliable/best-effort spaces independently from peer OPEN, with wraparound at the negotiated width. |
| Cookie | At most 1024 bytes; copied unchanged into OPEN output. Its size plus OPEN fields must fit the negotiated batch. No cookie interpretation or logging. |
| Extensions | Bound chains to 16 entries; skip well-formed unknown optional unit/integer/buffer extensions. Reject mandatory extensions, malformed encoding and unrequested QoS-channel, shared-memory, authentication, multilink, low-latency, compression or nonzero patch negotiation. |
| Timing | Five-second timeout per handshake stage; local lease 4000 ms and keepalive every 1000 ms when idle. Accept remote lease 1-120000 ms and support wire seconds/milliseconds. Late receive cannot revive an expired session. |
| TX ownership | One fixed 4098-byte output buffer holds a batch plus TCP prefix. `sendNetworkMessage` copies a complete message into a reliable FRAME only when output is free and the envelope fits the negotiated batch. Adapter calls `consumeOutput` for actual writes; sequence advances only on accepted enqueue. A stalled batch fails after four seconds. |

Call `start` on a new/failed/closed session only; it resets buffered state.
Use a single owner for `receive`, `service` and `consumeOutput`, with a
nondecreasing millisecond clock. `service` must run regularly even without
socket traffic; `receive` also enforces deadlines before processing input.
On EOF or unrecoverable socket errors call `linkLost`, and close the socket
on failed/closed state. Successful frame storage is borrowed only until the
next batch starts or the framer is reset. No per-batch allocation is used;
actual ESP32 stack/heap and CPU costs have not been measured.

**Current limits:** INIT ACK and OPEN ACK are accepted as individual
handshake batches. After establishment the former FRAME-discard path has
been removed: discovery INTEREST/DECLARE messages are decoded individually,
then parsing resumes at the following network or transport message in the
batch. Empty frames, bad sequence continuity and unsupported incoming
network kinds fail closed. This is still a bounded publication prototype,
not general Zenoh receive support: inbound PUSH/REQUEST/RESPONSE, fragmented
messages, peer key-table resolution and general discovery routing are not
implemented. There is no automatic reconnect, graceful outbound CLOSE,
retained history, encryption or authentication.

The live `--tcp-session` test starts only a native ROS RMW peer and runs the
C++ probe as a separate process. It does **not** open a Python Zenoh client.
Both endpoints use container loopback with no router; host configuration
adds `transport/link/tx/lease=4000` and `transport/link/tx/keep_alive=4` in
both TCP test modes. With the original 60000-ms host lease, negotiation succeeded
but the probe's bounded two-keepalive criterion was not met; the explicit
short lease makes bidirectional maintenance observable. Final run: three
local lease periods, 12 peer keepalives, no ignored application frames.
The probe's normal duration is 12000 ms (maximum 120000), plus bounded
connect/handshake time; the harness and outer command add process deadlines.

Local build/checks, from `linux_unit_tests`:

```bash
make zenoh-session-test zenoh-session-probe
make zenoh-session-test BUILD_DIR=build/zenoh-session-sanitize \
  ZENOH_TEST_CFLAGS='-std=c++17 -Wall -Wextra -Werror -pedantic -g -fsanitize=address,undefined -fno-omit-frame-pointer'
```

Native handshake check, from repository root using the image build above:

```bash
docker run --rm --init --network none raftros-zenoh-metadata-test timeout 45s /bin/bash -c 'source /reference/install/setup.bash; exec /opt/zenoh-test/bin/python /test/zenoh_metadata_interop.py --tcp-session'
```

The original metadata/identity and handshake-only controls also passed again
after the publication changes. Keep all three modes separate: default tests
metadata with upstream session/CDR; `--tcp-session` tests only the own-socket
handshake; `--tcp-publish` exercises integrated String publication and
`--tcp-range` exercises the existing Range serializer. Default RTPS
firmware/build source selection is unchanged.

### Raft-Owned Discovery and String Publishing

[ZenohNetworkMessage.h](../components/RaftROS/Zenoh/ZenohNetworkMessage.h)
adds original bounded codecs based on the pinned protocol's
[DECLARE](https://github.com/eclipse-zenoh/zenoh/blob/2687c51352121f006e3a603ce07925a8ad0b295c/commons/zenoh-protocol/src/network/declare.rs),
[INTEREST](https://github.com/eclipse-zenoh/zenoh/blob/2687c51352121f006e3a603ce07925a8ad0b295c/commons/zenoh-protocol/src/network/interest.rs)
and [PUT](https://github.com/eclipse-zenoh/zenoh/blob/2687c51352121f006e3a603ce07925a8ad0b295c/commons/zenoh-protocol/src/zenoh/put.rs)
wire definitions. It is not an upstream library wrapper.

- Outbound: liveliness token declare/undeclare, declaration-final (optionally
  correlated with an interest ID), and PUSH/PUT with a raw attachment and
  serialized payload. Full scope-zero named keys avoid a transmit key table.
- Bounds: concrete keys up to 1536 bytes, payloads up to 2048, attachments up
  to 256. Builders return zero on invalid input or insufficient space;
  output may be partially written on failure and must not be sent. Lengths
  are checked before copying. These limits do not imply fragmentation support.
- Inbound: `readDiscovery` returns the exact consumed size and borrowed
  message fields for INTEREST or supported DECLARE bodies. Key scope/mapping
  is exposed but not resolved. Invalid input leaves the output event unchanged.
  Unknown optional extensions are skipped within length/chain bounds;
  mandatory extensions are rejected except a network node-ID extension of
  zero. Mandatory wire-expression extensions on undeclarations remain unsupported.
- Session: `sendNetworkMessage` is a low-level enqueue of one already encoded
  message, not semantic validation or remote acknowledgment. Buffer-busy or
  oversize sends return false; callers keep pending work. RX invokes an
  optional synchronous discovery callback whose string views expire with
  the received buffer. Copy required state and schedule replies later; do
  not recursively service the session from the callback. Callback rejection
  fails the session, providing bounded backpressure on discovery state.
- Sequences: outgoing reliable frames use one counter; incoming reliable
  and best-effort frames use separate exact-next counters. Duplicate/gap
  rejection and 8-bit wraparound are covered by tests. This strict TCP
  prototype does not implement gap recovery or relaxed best-effort gap handling.

The probe's `--publish` mode creates one node token (token ID 1) and one
publisher token (token ID 2), with ROS entity ID 1. It uses the OS-generated
session ID, canonical ROS strings and Raft-derived GID. It encodes
`from_raft_tcp` using the existing `CDREncoder`, includes the GID and
monotonically increasing sample sequence in each attachment, and publishes
at a nominal 250-ms cadence until 6.5 seconds after establishment. It then
withdraws publisher and node tokens and keeps the transport serviced until
the 12-second test finishes. There is no host-side payload or GID input to
this path. Source timestamp zero explicitly remains the prototype's
unsynchronized-time value, not an epoch conversion implementation.

The probe also has a four-entry current-interest reply queue. It can return
its live tokens and a correlated final for unrestricted, exact-key or simple
`prefix/**` token interests; unsupported scope/patterns or queue exhaustion
fail explicitly. Future changes are sent as unsolicited declarations and
interest-final cancels pending replies. This is test-probe policy, not a
general-purpose dynamic device/interest registry. **The observed native
topology sent no incoming frames**. The scripted peer test below now drives
correlated replies directly through the actual socket and responder, including
capacity/cancellation checks. That is not evidence that every upstream peer's
interest/scoped-key behavior is supported or that all responder races are covered.

`--tcp-publish` starts a native ROS observer before the probe, checks publisher
name/namespace, String type, BEST_EFFORT/VOLATILE/KEEP_LAST depth 5, receives
at least five samples and checks values plus attachment sequence/source time.
After this begins, it starts a second independent ROS process in client mode
connected to the first native peer. That process discovers the cached live
publisher and receives at least two new samples. The first observer then
verifies token withdrawal despite its own remaining subscription, and compares
the generated GID reported by the probe with the graph's GID.

Both ROS processes and the Raft probe are inside an isolated Docker network
namespace using loopback; no `rmw_zenohd`, translation bridge or Python Zenoh
client supplies the publishing connection. The first ROS application peer
does provide routing/cache access for the later ROS client, so losing that
peer, connecting directly to multiple peers, LAN/WiFi and ESP32 behavior
remain separate tests. ROS-to-Raft subscriptions are not implemented by this
publication proof. The independent metadata control still tests that direction
using upstream libraries only.

Run from repository root after the existing Docker build command:

```bash
docker run --rm --init --network none raftros-zenoh-metadata-test timeout 45s /bin/bash -c 'source /reference/install/setup.bash; exec /opt/zenoh-test/bin/python /test/zenoh_metadata_interop.py --tcp-publish'
```

The integrated probe also passed after recompiling it with
`-fsanitize=address,undefined -fno-omit-frame-pointer` inside the container.
Local `make zenoh-session-test` passes 1236 assertions; all earlier codec/GID
and standalone RTPS baselines remain separately recorded, not combined into
one misleading count. No production RTPS files were changed in this slice.

### Raft-Owned Range Publishing

The probe's `--publish-range` mode reuses
[AutoPubCDRSerializer](../components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp)
and `RTPSAutoPubClassMap_typeName(Range)` without modifying either. Their
current RTPS-named path/namespace is retained until the planned common-layer
extraction. Only mapping/serialization code is linked into the probe, not
SPDP, SEDP, RTPS wire/reliability or RaftCore. This remains an isolated Linux
test mode, not the firmware build-time transport selector.

The type hash was read from the installed `sensor_msgs` **5.3.8** generated
`msg/Range.json` in the pinned ROS test image:

```text
sensor_msgs/msg/Range
RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a
```

This describes the Jazzy layout including `float32 variance`, not Humble's
older Range definition. The probe includes this hash in the topic key and
publisher token and reports it alongside its generated GID. The native
harness independently compares it with the installed type-description
metadata, rather than trusting a second hand-written hash constant.

One node and one publisher are declared per run with the same bounded
lifecycle as String mode. The Range topic is `/raft_test/range`, the node is
`/raft_test/raft_fixture`, and QoS is BEST_EFFORT/VOLATILE/KEEP_LAST depth 5.
Synthetic input deliberately exercises the existing serializer boundary:

| Fixture input | Expected native Range |
| --- | --- |
| `dist` uint16 raw values cycling 0, 368, 2000, 4000; attribute divisor 2, addend 0 | Millimetres 0, 184, 1000, 2000 converted to metres 0.0, 0.184, 1.0, 2.0. |
| `timeMs = 1234 + (sequence - 1) * 250` | Header seconds/nanoseconds, including a seconds rollover; independent of attachment source timestamp zero. |
| Frame ID `raft_range_1_29` | Exact string plus valid CDR NUL/length encoding. |
| Current serializer defaults | INFRARED radiation, FOV 0, minimum 0 m, maximum 2 m and variance 0. |

The C++ fixture accepts sequences 1-64 and uses the same serialization
function for CLI fixtures and network samples. `--range-payload <sequence>`
prints CDR hex without opening sockets. Tests cover sequences 1, 2, 3, 4, 5
and 64 and reject zero, out-of-range, negative, overflowing and malformed
sequence inputs. No bus bytes are decoded in this test; `PollRecord` represents
the already-decoded DeviceManager record consumed by the existing serializer.

`--tcp-range` first compares those six fixture payloads with native ROS
serialization, then starts the probe and typed plus raw ROS subscriptions.
It checks at least five received samples, validates all collected typed
messages and raw bytes against their attachment sequence, and starts a late
independent Range observer that validates at least two subsequent samples.
Publisher/node withdrawal, correct type/QoS and graph GID equality are checked
as before. One publisher is tested at a time; simultaneous String and Range,
composite devices and physical sensor attach/detach are not claimed.

**CDR comparison detail:** the 56-byte fixture contains three alignment bytes
at offsets 33-35 (zero-based), after `radiation_type`. Native ROS serialization
left these bytes nonzero in one diagnostic run; their values are not message
fields. The test normalizes only those three native-reference padding bytes,
then requires Raft's payload to match every other byte and the exact length,
with zeroed padding. Typed-message comparisons normalize the same reference
padding. Negative checks verify that field-byte corruption, truncation and
trailing data still fail. No serializer change was needed; do not mask other
bytes or treat the fixture-specific normalization as a generic CDR parser.

The complete Range run passed with the probe **and reused sensor serializer**
compiled under AddressSanitizer/UndefinedBehaviorSanitizer. The String run
also passed after the shared harness changes. The existing session suite
continues to pass its 1236 checks in the Docker build; no new C++ assertion
count is claimed for the native Python integration checks. No production
RTPS code or serializer was modified in this slice.

Run from the repository root after rebuilding the existing test image:

```bash
docker build --progress=plain -f linux_unit_tests/Dockerfile.zenoh -t raftros-zenoh-metadata-test .
docker run --rm --init --network none raftros-zenoh-metadata-test timeout 45s /bin/bash -c 'source /reference/install/setup.bash; exec /opt/zenoh-test/bin/python /test/zenoh_metadata_interop.py --tcp-range'
```

Local builds still use `make zenoh-session-probe`; they now compile the
existing sensor serializer as well as CDR. For sanitizer reruns, both source
files must be linked (see the probe's Makefile/Dockerfile commands). All host
toolchains and dependencies remain confined to the test setup.

### Direct Discovery and Restart Evidence

[zenoh_wire_interop.py](../linux_unit_tests/zenoh_wire_interop.py) is a
standard-library-only scripted TCP peer for the real C++ probe. It accepts the
probe's INIT/OPEN, checks the negotiated profile and cookie echo, sends exact
INTEREST wire messages, and independently decodes returned token/final/PUT
envelopes. It checks frame sequence continuity and complete message consumption.
This is a deliberately small protocol test peer, not another production Zenoh
implementation or a native RMW test. Listeners bind to ephemeral loopback
ports; socket, reply and process deadlines bound every scenario.

Passing scenarios, including with the C++ probe under ASan/UBSan:

- Current and current-future interests return the expected live node/publisher
  tokens followed by one matching correlated final. Exact publisher/node keys,
  `@ros2_lv/23/**`, a nonmatching domain prefix and non-token interests are tested.
- Future-only interests produce no correlated current reply. Interest-final
  cancels a queued request in the same incoming frame before output is serviced.
- Four simultaneous current replies succeed. Cancelling one frees a slot for
  a replacement in the same incoming frame. Five requests fail explicitly
  with discovery rejection, rather than growing storage or dropping silently.
- After explicit publisher/node withdrawal, a new current interest returns
  an empty final and no token or sample reappears before session shutdown.
- Separate fresh-process cases verify TCP EOF (`LinkLost`), peer silence
  (`LeaseExpired`), unsupported scoped/wildcard requests and queue overflow
  (`DiscoveryRejected`), and remote CLOSE with its reason. Failure cases must
  exit within six seconds with the expected status, not merely any nonzero
  result. Successive handshakes use different session IDs and start outgoing
  frame sequences at zero.

The successful direct-interest run received nine FRAMEs, unlike the existing
native cached-graph topology. Observed output was 6442 transmitted/506 received
bytes over its 12-second run, with 23 scripted keepalives; small byte-count
changes from random session-ID text length are not failures. No C++ protocol
repair was required to pass these scenarios. More general key expressions,
upstream scoped-interest behavior, arbitrary fragmentation, concurrent device
mutation, and cancellation after partially sent replies remain outside this
test's evidence.

The separate native `--tcp-restart` test reuses `rmw_zenoh_cpp` 0.2.11 and
Range validation from the normal publication proof. It performs three rounds:

1. Receive at least three Range samples, kill the publisher with SIGKILL before
   its normal token withdrawals, and verify the remote publisher and node
   disappear while the observer subscription remains.
2. Start a fresh publisher on the same topic, verify a distinct graph GID and
   sequence one, then explicitly shut down the ROS peer's node/context.
   The native peer sends CLOSE; the probe terminates with a session failure
   within six seconds instead of hanging.
3. Recreate the native peer and start another fresh probe. Validate the same
   topic/type hash, a third GID, sequence one and correct Range fields, then
   repeat abrupt publisher-loss graph cleanup.

Each native ROS context owns a matching executor. The default `rclpy` global
executor cannot service these explicitly initialized contexts; it is not
initialized here. Executors/nodes are disposed before their contexts, and
all child processes are reaped on success or failure.

Both suites pass with the C++ probe and sensor serializer instrumented by
AddressSanitizer/UndefinedBehaviorSanitizer. Native SIGKILL scenarios cannot
prove leak-free graceful teardown of the killed process; normal teardown is
covered separately. The regular typed/raw Range and late-observer test was
rerun successfully after extending the harness. Existing 1236 session unit
checks pass unchanged; the new scenario checks are not folded into that count.

Local direct-wire test, from `linux_unit_tests`:

```bash
make zenoh-wire-test
```

Docker tests, from repository root after rebuilding the existing image:

```bash
docker run --rm --init --network none raftros-zenoh-metadata-test timeout 45s /opt/zenoh-test/bin/python /test/zenoh_wire_interop.py --probe /test/zenoh_session_probe
docker run --rm --init --network none raftros-zenoh-metadata-test timeout 60s /bin/bash -c 'source /reference/install/setup.bash; exec /opt/zenoh-test/bin/python /test/zenoh_metadata_interop.py --tcp-restart'
```

**Remaining gates:** recovery is explicit process restart, not reconnection
of the same session object, endpoint replay, or DeviceManager recovery. Peer
shutdown is orderly; blackholed traffic is exercised only by the scripted
peer's silence case. No LAN/WiFi, hardware hot-plug, full QoS, long-run leak
or multi-peer failover claim is made. The host release baseline and provisional
firmware budgets are now recorded below. Broader discovery, automatic
reconnect, subscriptions and firmware selection remain planned work.

### Release Resource Baseline

Measured 2026-09-17 on Ubuntu WSL with g++ 13.3.0
(`13.3.0-6ubuntu2~24.04.1`), target `x86_64-linux-gnu`, GNU binutils 2.42,
ELF64 little-endian. Both programs use these compile flags and link with
`--gc-sections` plus a per-program map file:

```text
-std=c++20 -Wall -Os -DNDEBUG -ffunction-sections -fdata-sections -fstack-usage
-DRAFT_CORE -DRAFTROS_ACK_HEX_DUMP_ENABLE=0 -DRAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE=0
```

`make resource-baseline` in [linux_unit_tests/Makefile](../linux_unit_tests/Makefile)
forces separate builds under `build/resources/rtps` and
`build/resources/zenoh`, without overwriting normal debug executables. It
runs the normal RaftCore dependency step, then
[resource_report.py](../linux_unit_tests/resource_report.py) writes
`build/resources/report.json`. The report records flags/toolchain, repository
revision and dirty status, RaftCore revision, artifact SHA-256s, GNU size
sections, map paths, compiler stack records and known backend-symbol checks.
The measured source base is `ac23400f7a5202153b17a1de420fd091d577885f` plus
the recorded worktree edits; RaftCore is
`feb4f77f1778be04fbf1789bdaa6c84ec4e8fe5c`. Pin dependencies when comparing
later measurements: the existing fetch target may update RaftCore on a
future invocation. No fully frozen build environment is implied.

| Linux artifact | RTPS standalone | Zenoh String/Range probe |
| --- | ---: | ---: |
| GNU size `text` (including read-only data) | 40023 B | 32071 B |
| Initialized `data` | 1000 B | 937 B |
| Zero-initialized `bss` | 3240 B | 48 B |
| `text + data` | 41023 B | 33008 B |
| `data + bss` | 4240 B | 985 B |
| ELF file size (not flash size) | 71456 B | 49696 B |
| Compiler `main` frame | 768 B | 400 B |

The latest Zenoh values include the owned-storage change below. Before that
change the probe measured text+data 32117 B, ELF 49208 B and a 19568-byte main
frame. The additional code/data covers ownership, allocation failure and
resource diagnostics; no claim of reduced total RAM follows from the stack
reduction. The latest report records the precise build/artifact hashes.

**Not a backend efficiency claim:** the RTPS standalone and Zenoh probe do
not implement identical behavior. The RTPS executable includes its existing
DDS discovery/reliability wrapper; its unused sensor serializer is discarded.
The Zenoh probe includes String/Range modes, CLI/test logic, the sensor
serializer and only its supported protocol subset. Neither represents full
16-device firmware or completed subscription/QoS parity. Log/CLI code remains
in both artifacts. Shared-library code/memory, runtime heap, thread stacks,
kernel socket buffers and runtime CPU/RSS are not measured by GNU size.
Different compiler/platforms and future functionality will change these values.

The symbol check finds no `RaftRuntime::Zenoh` definitions in the RTPS
artifact and no known RTPS participant/SPDP/SEDP/wire definitions in the Zenoh
artifact. This checks these executables, not complete firmware dependency
isolation or the future `RAFTROS_TRANSPORT` build contract. Compiler `.su`
records include functions later removed by the linker; they are per-function
frames, not a measured maximum nested call chain or live task high-water mark.

The probe's non-networking `--resource-info` mode reports fixed C++ storage:

| Host storage | Bytes | Ownership / interpretation |
| --- | ---: | --- |
| `ZenohTCPSession` | 8344 | Includes RX/TX batch buffers, counters and framing state. |
| `PublicationProbe` | 8800 | Two graph tokens, topic key, builder storage, four replies and fixture state. Not a per-sensor estimate. |
| Socket read scratch | 2048 | Array inside the owned workspace, not in main's frame. |
| Complete `ProbeStorage` owner | 19192 | One startup heap allocation containing the three items above; limit 24576 B. Do not add the components again. |
| GID token scratch capacity | 1347 | Temporary inside identity derivation, not another persistent endpoint buffer. |

The first three items total 19192 bytes. Originally they occupied the main
frame; they now reside in a single owned allocation, separately from the
400-byte main frame and its nested calls. RTPS keeps much of its wrapper
state in static storage. Comparing BSS alone would therefore still hide most
of the Zenoh probe's memory use. Host pointers are eight bytes; these `sizeof`
values exclude allocator overhead and are not ESP32 heap measurements.

The measurement exposed `*this = ZenohTCPSession{}` materializing an additional
8400-byte reset frame under this release build. `start()` now resets framing,
state, counters, sequences and pending-output lengths **in place**, without
constructing a full temporary. After that change `start` is inlined and has
no separate `.su` frame. The later owner relocation reduced main from 19568
to 400 bytes. This does not mean zero stack use or a solved firmware stack
budget. Prior receive/transmit buffer
contents become inaccessible through reset lengths rather than being wiped;
this is not secure memory erasure. Tests cover partially received/sent old
connections, new identities, peer-close/error clearing and RX/TX sequence reset.

Validation for this slice: 1245 session checks pass normally and with
ASan/UBSan; the exact measured release Zenoh executable passes the scripted
wire/lifecycle suite; RTPS release `--help` starts successfully. A Docker-built
probe with the same release optimization policy also passes native Range
CDR/type/GID/graph/late-observer/withdrawal tests. The existing 921-test RTPS
baseline is historical for this slice; no new RTPS wire code was changed.

Reproduce from `linux_unit_tests`:

```bash
make -j4 resource-baseline
python3 zenoh_wire_interop.py --probe build/resources/zenoh/zenoh/zenoh_session_probe
```

Keep the report, map and stack files with a comparison run; they are generated
build outputs, not source files to commit. The reporting script does not run
network workloads or claim heap/CPU results.

### Owned Probe Storage

The non-copyable `ProbeStorage` in
[zenoh_session_probe.cpp](../linux_unit_tests/zenoh_session_probe.cpp) owns
`ZenohTCPSession`, `PublicationProbe` and socket receive scratch. A single
`std::unique_ptr` creates it directly with `new (std::nothrow)` after CLI/address
validation and before randomness, socket creation or connection. There is no
whole-workspace temporary, singleton, or per-sample allocation. A compile-time
check caps the owner at 24 KiB, with the current x86-64 allocation at 19192 B.
Constructing it has an 8-byte compiler frame under the measured flags.

Allocation failure returns exit code 1 with the requested byte count and no
network connection. Resource queries and the offline Range fixture return
before allocation. References into the owner remain stable for the entire
session; copying/moving the owner is disabled. On normal return or a handled
error, the socket is destroyed before the owner, since it is declared later.
No callbacks are asynchronous in the current probe. Forced process death does
not run C++ destructors and is not evidence of RAII cleanup.

The resource report now verifies off-stack ownership, component accounting and
the declared storage limit. It also rejects a missing, unbounded or larger-than-
4096-byte compiler main frame; `--max-probe-main-frame` can explicitly set a
different host review limit. A deliberately one-byte limit was tested and
rejected. This is a host regression guard, **not** a limit on nested calls or
an ESP32 task high-water mark: GID derivation still has a 1408-byte frame and
endpoint-token formatting a 1040-byte frame on the measured host.

`make zenoh-storage-test` builds a separate executable linked with
[zenoh_probe_allocation_failure.cpp](../linux_unit_tests/zenoh_probe_allocation_failure.cpp),
whose replacement nothrow allocator returns null. The ordinary probe does
not link that file. The scripted test verifies identical offline output and
early allocation failure with no accepted connection in handshake, String
and Range modes. The same gate runs when building the Docker test image.
There is no runtime failure-injection flag in the normal executable.

Validation after relocation: native Range/late-observer/withdrawal, direct
scripted interests and error exits, and three native explicit restart rounds
all passed with ASan/UBSan builds of the owned-storage probe. Resource builds
pass the new guards. The unchanged session unit suite still passes its 1245
checks in Docker; the allocation/lifecycle scenarios have separate results.

```bash
make zenoh-storage-test
make -j4 resource-baseline
```

This is bounded **probe** ownership, not yet a reusable firmware backend.
The 19192-byte request still needs a contiguous heap block, which is more
than the provisional 16 KiB minimum largest-block target at peak load below.
Firmware startup/reconnect design must either retain the owner for its whole
lifetime, demonstrate adequate allocation headroom, or choose a target-sized
static/pool allocation. Allocator overhead, other framework allocations,
multi-endpoint scaling, heap fragmentation and live task-stack margins remain
unmeasured. The storage relocation does not reduce the total RAM budget or
authorize increasing firmware stacks/using PSRAM.

### Provisional Firmware Budgets

These are initial engineering **review targets**, not measured ESP32 results,
new default configuration, or permission to change the existing constraints.
Validate/revise them against an actual release build and live device before
declaring Z0/Z3 complete. They apply to an initial one-session, bounded
volatile sensor profile; retained history and broader peer/QoS behavior need
explicit additional budgeting without silently downgrading configured QoS.

The checked-in [partition table](../examples/ExampleDiscoverable/systypes/SysTypeMain/partitions.csv)
has two OTA app slots of `0x1b0000` bytes each (1769472 B, 1728 KiB) on 4 MiB
flash. [sdkconfig.defaults](../examples/ExampleDiscoverable/systypes/SysTypeMain/sdkconfig.defaults)
selects size optimization and a 10000-byte main task stack. The example's
Compose command references an absent `build.sh`, and a Docker build that mounts
only the example folder cannot reach its relative RaftROS component path
(`raft build --docker` was not tried).
The working route is a local IDF build (`raft build --no-docker -e <idf path>`;
`-i` looks for an exact `6.0` directory). The 2026-09-18 RTPS-only image is
1257619 B, under the 1536 KiB target below (see
[Shared Sample Dispatch](#shared-sample-dispatch-2026-09-18)). Heap, stack and
runtime measurements still require a connected board. No flash, partition or SDK
setting was changed here.

| Area | Initial review target | Required evidence |
| --- | --- | --- |
| Whole firmware app image | At most `0x180000` B (1536 KiB), leaving `0x30000` B (192 KiB) in **each** existing OTA slot. Never remove the second OTA slot to make a backend fit. | Same board/Raft/IDF versions, features, optimization and partition table for each backend; actual app binary and IDF size/map report. Bootloader/filesystem fit remains separate. |
| Zenoh-owned session and shared wire workspace | At most 24 KiB direct RAM, including RX/TX, IO scratch, shared key/CDR builders and bounded discovery queues. | Target `sizeof`/static allocation inventory and live allocation deltas; count shared scratch once, not per endpoint. Do not clone the entire probe object into each writer. |
| Common auto-pub state | At most 32 KiB direct RAM for 16 publisher slots in the initial volatile profile; composites consume multiple slots. | Count owned names, metadata, decoder state, serialization/mailbox buffers and in-flight samples at full occupancy. Existing non-ROS DeviceManager storage and any new allocations must be distinguished. |
| Added worker stack, if introduced | At most 8 KiB allocated; at least 2 KiB measured unused at the worst observed workload. Without a worker, retain the existing 10000-byte main stack and demonstrate the same spare margin. | Target compiler call-chain review plus FreeRTOS stack high-water marks during GID generation, attach/detach, reconnect and maximum messages. Large persistent arrays must be object/pool-owned outside the task stack. |
| Combined new RaftROS-owned RAM | At most 64 KiB for the initial profile (24 + 32 + 8 KiB), excluding pre-existing framework/network storage but including all RaftROS allocations and any new task stack. | Static + heap + stack accounting with no double counting; separately record lwIP/WiFi socket allocation overhead and minimum/largest free heap. No PSRAM dependency. |
| Headroom | Target at least 64 KiB free internal heap and a 16 KiB largest internal block at peak load. | Live board measurement, not Linux RSS. If the shared application cannot meet it, review limits before proceeding rather than silently enabling PSRAM. |
| Socket/task count | One Zenoh TCP socket for the first profile; no duplicate RTPS sockets/tasks in a Zenoh-only build. | Runtime inventory and compile/link dependency check. Extra peer links require an explicit revised bound. |
| Bus callback | No blocking network IO, session open, retry or unbounded allocation. | Measure enqueue/callback latency and poll jitter under slow/no peer and full-queue conditions; numerical CPU/latency ceilings await the actual board baseline. |

The earlier 19568-byte Linux main frame was a porting blocker. The owned
probe now measures 400 bytes in main, with persistent buffers outside the
stack. The firmware ownership design must still account for target heap/pool
capacity and nested serializer/GID scratch, not simply copy the Linux
allocation strategy. Budget exhaustion must return a visible error and clean
up partial endpoint allocations; no firmware stack setting changed here.

Next: use this baseline to design the minimal common ownership/backend
boundary, starting with sensor/CDR separation and a behavior-preserving RTPS
adapter. Keep target firmware measurement and live heap/stack validation as
unmet gates; the host table does not authorize a firmware readiness claim.

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
| [CDR](../components/RaftROS/CDR/CDREncoder.h) and [sensor serializer](../components/RaftROS/AutoPub/AutoPubCDRSerializer.h) | Shared mapping/CDR ownership now lives in `RaftRuntime::AutoPub`, with legacy RTPS forwarding headers. Reuse existing algorithms and tests while extracting the remaining lifecycle/transport boundary. |
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

**Z0 is in progress.** The isolated metadata preparatory slice above is
complete, and Z1 now has passing Raft-owned String and Range graph/data/
late-join proofs, scripted direct-interest coverage and native explicit restart
coverage, as well as the separate host-library control. Host release resources
are measured and provisional firmware review budgets are recorded. Actual
firmware measurements, broader discovery/topology and automatic reconnect
remain open. Z2 has started with shared sensor mapping/CDR only; its endpoint
operations, RTPS adapter and firmware regression gates remain open. Z3-Z6
remain unstarted.
Work in order; keep each extraction
small and validate RTPS before continuing. Z0/Z1 can use a native harness
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

Implemented metadata-codec checks, from `linux_unit_tests` on Linux/WSL:

```bash
make zenoh-test
make zenoh-test BUILD_DIR=build/zenoh-sanitize \
  ZENOH_TEST_CFLAGS='-std=c++17 -Wall -Wextra -Werror -pedantic -g -fsanitize=address,undefined -fno-omit-frame-pointer'
```

Separate build directories prevent a previously built non-sanitized binary
from satisfying the sanitizer target. `make clean` removes both with the
default build directory. Neither target implements `RAFTROS_TRANSPORT` or
changes the default RTPS source selection.

Historical combined RTPS baseline command, from `linux_unit_tests` on Linux/WSL:

```bash
make -j"$(nproc)" all standalone && ./linux_unit_tests
```

Run clean or separate-profile builds when comparing backends. The Makefile
currently fetches RaftCore; pin/record its revision for repeatability. The
unit rerun passed 921/921 on 2026-09-17. The initial standalone include/API
failure has been resolved; the standalone executable now compiles and links.

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

Upstream material consulted 2026-09-17. The metadata design revision is pinned
in the implementation record above; the remaining runtime/link versions still
need pinning in Z0:

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