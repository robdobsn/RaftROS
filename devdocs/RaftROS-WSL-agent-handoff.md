# RaftROS WSL Agent Handoff

**Recorded:** 2026-09-17, after the 20:05 UTC resource build.
**Reason:** The user paused implementation to move this agent session from a
Windows mapped-folder workspace into a native WSL workspace.
**Resume point (updated 2026-09-21):** Checklist items 1–5 are done. The
bus→loop mailbox handoff is implemented and validated on hardware against
native ROS 2 Jazzy (see
[development status](RaftROS-development-status.md)). Continue at item 6, or
at the open issues listed under [Current Open Issues](#current-open-issues-2026-09-21).

## Current Open Issues (2026-09-21)

1. **`_NODE_NAME_UNKNOWN_` (O6) reproduces on native Linux.** `ros2 node list`
   is empty and `ros2 topic info -v` shows no node name, though the topic,
   type, hash, QoS and data are all correct. Previously assumed to be a
   WSL/Jazzy artefact; it is not. This is the next functional bug to chase.
2. **Rare ~101-176 ms loop stall** around SPDP/heartbeat sends, seen a handful
   of times per hour. Sockets are already non-blocking, so the suspicion is
   the lwIP TCP/IP task; needs per-call timing to confirm.
3. **Logging blocks the loop task when the USB console has no reader** -
   RaftCore's logger uses a 100 ms write timeout twice per line, so ~200 ms
   per line with USB unplugged; that lost ~4.5% of auto-published samples.
   The example now overrides `RAFT_LOGGER_USB_JTAG_WRITE_TIMEOUT_MS=10`;
   a RaftCore default change is proposed.
4. **Test hardware moved** to an Adafruit ESP32-S3 TFT Feather (COM18): I2C
   SDA 42 / SCL 41, and GPIO 21 must be driven high to power the STEMMA QT
   connector (done in the example's `main.cpp`). The UM ProS3 previously used
   has an RF fault - it receives fine but never completes WiFi authentication,
   confirmed against a Xiao ESP32-S3 running the identical stock IDF test.
5. **ROS 2 test host:** Ubuntu 24.04 at `rob@192.168.86.192` (SSH keys set up,
   ROS 2 Jazzy installed natively). The Feather must be on `rdint01` to share
   that subnet; `rdiot` is a different subnet.

## Resumed in WSL (2026-09-18)

- Worktree verified intact at `ac23400`, RaftCore `feb4f77`.
- Review of runner, emitter and `autoPubOnDeviceData` complete: behavior
  preserved (released/`0xFF` slots now report `InvalidHandle` instead of 0 peers),
  log arguments match. Fixes: invalid-batch warning rate-limited, nested lambda
  indentation, restored `heartbeatCount`/`firstSN` labels and QoS/composite
  comment, contract documentation added to both new headers.
- 979/979 passes normally and under ASan/UBSan, zero warnings.
- **ESP build route found:** `cd examples/ExampleDiscoverable && raft build --no-docker -e /home/rob/esp/esp-idf-v6.0.2 .`
  (`-i` fails: it wants an exact `6.0` directory). It compiles the modified
  `RaftROS.cpp`; image 1257619 B. The build rewrites the tracked
  `dependencies.lock` for IDF 6.0.2. Restore it with `git restore` unless
  intentionally upgrading. Not flashed; no board test.
- Devdocs synchronized (status, Zenoh plan, overview, next stages, auto-publishing design).
  Docker/native oracle and resource-baseline were not rerun: only doc comments,
  logging and formatting changed since their 2026-09-17 runs.

## Start Here

Open the existing checkout in VS Code Remote - WSL (Ubuntu):

```bash
cd /home/rob/rdev/raft/RaftROS
code .
git --no-pager status --short
```

This is the same checkout previously accessed on Windows as
`Z:\home\rob\rdev\raft\RaftROS`. Do not replace it with a clean clone: much
of the implementation is modified or untracked, not committed. The agent has
not committed, staged, reset, or created a branch. Preserve all existing work;
the user may have committed earlier slices between turns.

Recorded HEAD: `ac23400f7a5202153b17a1de420fd091d577885f`.
The Linux test dependency checkout at `linux_unit_tests/RaftCore` was at
`feb4f77f1778be04fbf1789bdaa6c84ec4e8fe5c`.

Read this note first, then the
[Zenoh implementation plan](RaftROS-zenoh-implementation-plan.md) and
[development status](RaftROS-development-status.md). Their earlier **929-test**
and **33008-byte Zenoh text+data** records describe the preceding serializer
extraction. This handoff records the newer, interrupted runner slice. Its full
documentation synchronization and final production-wrapper review are pending.
No prior agent memory or transcript is required to use this handoff.

## User Goal and Constraints

- Preserve native ROS 2 sensor auto-publishing from DeviceManager-detected I2C
  devices in Raft ESP32 firmware, adding native Zenoh as an alternative to RTPS.
- Ultimately build exactly one transport: RTPS by default or Zenoh, excluding
  the unused transport's sources, dependencies and storage. No runtime switch
  or combined dual-backend binary is requested.
- Preserve the standalone goal: no micro-ROS/device agent or DDS translation
  bridge. Routerless feasibility remains a gate, not a waived requirement.
- Keep the Raft-only/original-source policy. No upstream Zenoh implementation
  was copied into firmware. Host reference libraries are test oracles only;
  incorporating zenoh-pico or changing dependency policy needs explicit approval.
- Keep devdocs current and RTPS APIs/behavior compatible. Do not invent RTPS
  GUIDs for Zenoh or silently downgrade supported QoS.
- No services, parameters, actions, production subscriptions over Zenoh, or
  automatic reconnect have been implemented by these slices.
- RTPS remains the only firmware backend. Host proofs are not ESP32 support.
  Z0/Z1 have substantial evidence but unmet gates; Z2 is partial; Z3-Z6 are pending.

## Current Slice: Shared Sample Dispatch

### Implemented but Not Fully Closed Out

1. [AutoPubSampleRunner.h](../components/RaftROS/AutoPub/AutoPubSampleRunner.h)
   is a new header-only common runner in `RaftRuntime::AutoPub`.
   It accepts a borrowed decoded batch and one or two caller-owned output
   buffers, checks `recordCount <= capacity / recordSize` before pointer
   arithmetic, selects the last decoded record, reads its leading `timeMs`,
   serializes primary then secondary, and invokes a synchronous publish callable.
   It adds no allocation, worker, queue, endpoint registry or transport identity.
2. `AutoPubDecodedBatch` describes data/capacity/recordSize/recordCount,
   field descriptors/count and frame ID. `AutoPubSampleOutput` describes kind,
   buffer and capacity. `AutoPubSampleResult` separates serialization status,
   bytes written and backend outcome. `run()` returning true means a valid batch,
   not that every output serialized or was delivered.
3. `AutoPubPublishResult` currently has `NotAttempted`, `Accepted`, `QueueFull`,
   `Disconnected`, `InvalidHandle`, `Oversized`, and `SendFailed`, in that order.
   Disabled/failed outputs are not published. A primary serialization failure
   does not suppress an otherwise valid secondary. Callback outcomes are kept
   independently. These result values do not implement queueing or QoS guarantees.
4. [RTPSAutoPubSampleEmitter.h](../components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubSampleEmitter.h)
   is a new RTPS-only adapter around an existing `RTPSDynamicWriterEntry*`.
   It increments the writer sequence once per nonempty sample, even with no
   peers or failed sends, and attempts every peer with the same sequence.
   Any successful peer send makes the aggregate accepted; if none succeeds,
   it returns the last peer failure, or disconnected for an empty peer list.
   A missing/released entry does not send. This pointer is NOT a generational
   handle and does not protect against a stale callback after slot reuse.
5. [RaftROS.cpp](../components/RaftROS/RaftROS.cpp),
   `RaftROS::autoPubOnDeviceData`, now builds the common batch/output descriptors
   and delegates to the runner/emitter. It still pads raw input, calls the
   generated decoder, retains its initial no-decode/no-serialize checks, uses
   existing per-device buffers, and performs synchronous RTPS `sendto` calls.
   Packet arguments remain writer entity ID, sequence, heartbeat count zero,
   and firstSN equal to sequence. Logs now include numeric publish outcomes;
   invalid decoded batches log a warning instead of forming an unchecked pointer.
6. [zenoh_session_probe.cpp](../linux_unit_tests/zenoh_session_probe.cpp),
   `serializeRangeSample`, uses the same runner for a single synthetic record.
   Its callback records the output length; the probe's existing network stage
   sends later. This proves runner/CDR reuse, not a complete common Zenoh adapter.
7. [main.cpp](../linux_unit_tests/main.cpp) reuses the existing test harness:
   33 new runner assertions plus 17 RTPS emitter assertions bring 929 to **979**.
   Coverage includes composite order, latest timestamp/bytes, copied payload
   survival after caller-buffer reuse, independent failures, invalid count/stride/
   pointer/descriptor/output bounds, sequence preservation, partial success,
   no peers, empty payloads and released writers.
8. [Makefile](../linux_unit_tests/Makefile) tracks the runner header as a probe
   dependency; [Dockerfile.zenoh](../linux_unit_tests/Dockerfile.zenoh) copies it
   into the isolated test build. Neither new header needs a source registration.

### Ownership Contract and Remaining Risks

The runner is synchronous and borrows all storage. Its caller must keep batch,
descriptors, output arrays and result arrays valid for the complete call. The
publish callable must consume or copy sample bytes before returning if it will
use them later. It must not invalidate the batch or mutate subsequent output
descriptors during dispatch. There is no lifetime enforcement or synchronization.

Finish documenting/reviewing this contract before extending it. The old
DeviceManager callback/detach ownership, shared send buffer and registry/peer
concurrency concerns are unchanged and not proven safe by the fake-backend tests.
The runner validates the declared batch capacity, not the decoder's internal
writes or the truth of caller-provided buffer sizes. Generated decoder behavior
and attach-time allocation contracts still belong to the wrapper.

The production callback patch had passed editor diagnostics, but final diff
review was interrupted by the workspace-move request. Inspect the nested
publish/send lambda formatting, log argument correspondence, and unchanged
primary/secondary behavior. Do not treat this note as a completed code review.

## Latest Verification

These results are from the working tree containing the new runner/emitter:

| Check | Result |
| --- | --- |
| Linux RTPS/common unit suite | **979 passed, 0 failed** |
| Same full suite under ASan/UBSan | **979 passed, 0 failed** |
| RTPS standalone target | Builds; release target rebuilt during resource measurement |
| Isolated Docker build | Pass; no RTPS headers copied for Zenoh probe |
| Docker session unit suite | **1245 passed, 0 failed** |
| Allocation-failure checks | Pass locally and in Docker; offline modes work, all three network modes fail before connecting |
| Native ROS Range with ASan/UBSan probe | Pass: native CDR field/length comparison, typed/raw samples, hash, graph, late process, withdrawal |
| Native Range runtime | 22 samples, 12 seconds, 12 received keepalives, 5282 TX bytes, 118 RX bytes, zero incoming network frames |
| Resource target | Pass; updated values below |
| Editor diagnostics | No errors in runner, emitter, probe, main tests or RaftROS.cpp; not a substitute for compiling firmware |

**Important:** the Linux Makefile builds protocol/common code and
[raftros_standalone.cpp](../linux_unit_tests/raftros_standalone.cpp), NOT the
ESP-IDF SysMod translation unit [RaftROS.cpp](../components/RaftROS/RaftROS.cpp).
Therefore the modified production wrapper has not been compiled for ESP32 or
tested on a board. `which idf.py raft` found neither command on the current
Ubuntu PATH; that does not prove no SDK exists elsewhere.

Earlier slices passed metadata/identity **2218** assertions and **3222** with
the pinned upstream GID oracle (1004 additional comparisons), scripted direct
discovery/error tests, native String, and three explicit native restart rounds,
including sanitizer runs. Those complete wire/restart suites were not rerun
after this runner change; the fresh native test was Range. No fresh LAN test,
hardware smoke test, complete firmware compile, or final runner-slice diff check
had been performed when implementation was paused.

### Current Host Resources

Generated report: `linux_unit_tests/build/resources/report.json`, measured
`2026-09-17T20:05:33.276511+00:00`, g++ 13.3.0, binutils 2.42, x86_64 Linux,
C++20 `-Os -DNDEBUG` with function/data sections, linker GC and stack-usage files.

| Metric | RTPS standalone | Zenoh probe |
| --- | ---: | ---: |
| text + data | 41023 B | **33062 B** |
| ELF file size | 71448 B | 49480 B |
| main frame | 768 B | 400 B |

The runner adds **54 B** to the prior Zenoh text+data result of 33008 B.
Persistent `ProbeStorage` remains one checked **19192 B** heap allocation,
compile-time capped at 24576 B: session 8344 + publication state 8800 + socket
receive scratch 2048. No payload allocation was added. Largest reported Zenoh
frames remain GID derivation 1408, token formatting 1040, publication init 928,
serializer 624. These are per-function host frames, not nested stack maxima.

The resource report checks common serializer linkage, absence of RTPS wire
symbols in the Zenoh probe, bounded storage and a 4096 B main-frame limit.
The target deletes generated `.su` files before its forced builds so relocated
sources cannot leave obsolete stack records. Never double-count the owner as
stack/BSS. Host executables are not feature-equivalent firmware measurements;
the RTPS standalone does not measure the modified bus callback.

Provisional firmware budgets remain only review targets: 1.5 MiB app in each
existing 0x1b0000-byte OTA slot, initial direct RAM <=64 KiB, session/workspace
<=24 KiB, common 16-publisher state <=32 KiB, added worker <=8 KiB. Actual heap,
largest block, stack high-water and image measurements are pending. The 19 KiB
contiguous owner allocation exceeds the provisional 16 KiB largest-block floor;
target startup/restart allocation policy must address this. No PSRAM or partition
changes were authorized.

## Earlier Work Already Present

- Common [mapping](../components/RaftROS/AutoPub/AutoPubClassMap.h),
  [serializer header](../components/RaftROS/AutoPub/AutoPubCDRSerializer.h), and
  [serializer implementation](../components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp)
  were mechanically extracted from RTPS with encoding behavior unchanged.
  Legacy RTPS headers alias types and inline-forward functions: source API
  compatibility, not precompiled ABI compatibility. The old RTPS serializer
  `.cpp` is intentionally deleted; compile only the common implementation.
- Root [CMakeLists.txt](../CMakeLists.txt), Linux and Docker source lists were
  updated. ESP unit tests use the root component via `REQUIRES RaftROS`.
  [library.json](../library.json) discovers sources below components/RaftROS.
  There is still no transport build selector.
- [ZenohROSCodec.h](../components/RaftROS/Zenoh/ZenohROSCodec.h) formats native
  RMW ROS graph tokens, data keys, QoS and 33-byte attachments.
  [ZenohROSIdentity.h](../components/RaftROS/Zenoh/ZenohROSIdentity.h) derives
  endpoint GIDs using original bounded XXH3-128 code, tested against upstream.
- [ZenohStreamFramer.h](../components/RaftROS/Zenoh/ZenohStreamFramer.h),
  [ZenohTCPSession.h](../components/RaftROS/Zenoh/ZenohTCPSession.h), and
  [ZenohNetworkMessage.h](../components/RaftROS/Zenoh/ZenohNetworkMessage.h)
  implement bounded own-wire framing, INIT/OPEN, keepalives, output consumption,
  declarations, PUT and a limited inbound discovery parser.
- The session has one 4098-byte TX buffer, 4096-byte batch bound, 1024-byte
  cookie bound, 5-second handshake timeout and 4-second local lease. Pending
  output cannot be overwritten. Unsupported negotiation/messages fail explicitly.
  General scoped keys, fragmentation, subscription data and automatic reconnect
  are not implemented. Discovery exact-next sequence checks are deliberately strict.
- The probe has one non-copyable off-stack owner allocated before entropy/socket
  setup. Earlier main-frame 19568 B and reset temporary 8400 B were removed.
  [zenoh_probe_allocation_failure.cpp](../linux_unit_tests/zenoh_probe_allocation_failure.cpp)
  is a separately linked test allocator override: never link it into normal code.
- [zenoh_wire_interop.py](../linux_unit_tests/zenoh_wire_interop.py) provides a
  scripted peer for current/current-future interests, matching/cancellation,
  four-entry reply capacity, withdrawal and six bounded failure/restart cases.
  [zenoh_metadata_interop.py](../linux_unit_tests/zenoh_metadata_interop.py)
  provides the independent native RMW graph/data oracle and explicit restarts.

## WSL Commands

Run commands below in a WSL Bash terminal, not Windows PowerShell. No `wsl -d`
prefix is needed after reopening the folder remotely. Use `git --no-pager`:
the last production diff command entered a pager; it was closed with `q` and
the terminal execution completed. No test/server needs to remain running.

### Local Regression

```bash
set -e
cd /home/rob/rdev/raft/RaftROS/linux_unit_tests
make -B -o raft_core -j4 all standalone zenoh-storage-test \
  BUILD_DIR=build/shared-extraction \
  OUTPUT=build/shared-extraction/linux_unit_tests \
  STANDALONE_OUT=build/shared-extraction/raftros_linux
./build/shared-extraction/linux_unit_tests
```

`-o raft_core` avoids an automatic fetch/update of the already-present dependency.
Verify that checkout exists before using it. `-B` is intentional for a fresh
baseline: the ordinary RTPS object rules track `.cpp` files but do not generate
complete header dependencies. A header-only edit can otherwise leave stale tests.

```bash
set -e
cd /home/rob/rdev/raft/RaftROS/linux_unit_tests
make -B -o raft_core -j4 all \
  BUILD_DIR=build/sample-runner-sanitize \
  OUTPUT=build/sample-runner-sanitize/linux_unit_tests \
  CFLAGS='-Wall -std=c++20 -g -DRAFT_CORE -DRAFTROS_ACK_HEX_DUMP_ENABLE=1 -DRAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE=1 -fsanitize=address,undefined -fno-omit-frame-pointer'
./build/sample-runner-sanitize/linux_unit_tests
```

Other focused checks, when relevant to the next edit:

```bash
cd /home/rob/rdev/raft/RaftROS/linux_unit_tests
make zenoh-test zenoh-session-test
make zenoh-wire-test BUILD_DIR=build/shared-extraction
make -o raft_core -j4 resource-baseline
```

### Native ROS Oracle

Docker Desktop's Linux engine was used from Windows. In the new WSL workspace,
check `docker version` and Desktop's Ubuntu WSL integration first. Do not assume
the Linux CLI/daemon connection is configured just because Windows Docker worked.
The existing image tag is `raftros-zenoh-metadata-test`. Rebuild the cached final
COPY/compile layers rather than relying on Windows mapped-path bind mounts,
which failed in the old workspace. The initial RMW/Rust build is expensive.

```bash
cd /home/rob/rdev/raft/RaftROS
docker build --progress=plain -f linux_unit_tests/Dockerfile.zenoh \
  -t raftros-zenoh-metadata-test .
docker run --rm --init --network none raftros-zenoh-metadata-test \
  timeout 60s /bin/bash -c '
    set -e
    source /reference/install/setup.bash
    g++ -std=c++17 -Wall -Wextra -Werror -pedantic -g \
      -fsanitize=address,undefined -fno-omit-frame-pointer \
      -Iinclude -Iinclude/CDR zenoh_session_probe.cpp \
      include/CDR/CDREncoder.cpp include/AutoPub/AutoPubCDRSerializer.cpp \
      -o zenoh_session_probe
    exec /opt/zenoh-test/bin/python /test/zenoh_metadata_interop.py --tcp-range
  '
```

For normal probe tests, omit the compile step. Harness modes include
`--tcp-session`, `--tcp-publish` (String), `--tcp-range`, and `--tcp-restart`.
Default mode is a host-library metadata control, NOT an own-socket proof.
The test container uses internal loopback, domain 23, port 17447, and no
external network. Do not remove network isolation to solve an unrelated failure.

## Interoperability Facts Not to Rediscover

- Pinned `rmw_zenoh_cpp` Jazzy **0.2.11**, commit
  `8c1fe8ef412bca5e6ac64f320468c70dcb03fc52`; Zenoh core reference
  `2687c51352121f006e3a603ce07925a8ad0b295c`, protocol version 9.
  Docker builds the pinned RMW source because its matching apt binary was absent.
- Python `eclipse-zenoh==1.8.0` is host-only; its source build needs Rust 1.93.0.
  Apt Cargo 1.75 could build the pinned RMW vendor but not the Python Cargo.lock.
  Do not merge those toolchain steps without checking compatibility.
- Native sensor_msgs/std_msgs are 5.3.8; rclpy is 7.1.12. The Dockerfile pins
  the ROS base image digest and the plan records detailed upstream versions.
- rclpy MessageInfo is a dict with timestamps and sequence fields, but **no
  publisher_gid**. Compare derived GIDs to graph endpoint_gid; do not reintroduce
  the failed callback-GID assertion.
- An explicit rclpy Context needs `SingleThreadedExecutor(context=context)`.
  Using the global executor broke native restart tests; the harness now handles it.
- Synthetic Range raw uint16 distances 0/368/2000/4000 with divisor 2 encode
  0/0.184/1/2 metres. Stamp is 1234 + (sequence-1)*250 ms; frame ID is
  `raft_range_1_29`. Jazzy Range includes variance. Its verified type hash is
  `RIHS01_b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a`.
- That Range fixture is 56 bytes. Native serialization can contain nonzero
  unspecified padding at offsets 33..35. Normalize only those native-reference
  bytes; Raft output must still have zero padding. Keep field corruption,
  truncation and extra-byte negative checks. Do not change production CDR for
  this test-oracle issue or claim all message types have this layout.
- Late ROS observers in the proof connect via the first ROS application peer,
  which routes/caches declarations. Native tests normally receive zero inbound
  network frames at the probe, so scripted-interest tests are separate evidence,
  not proof of arbitrary native discovery compatibility.
- Restart tests recreate processes/contexts, GIDs and sequence state. They do
  not implement same-process reconnect or desired-endpoint replay.

## Resume Checklist

1. Inspect current tracked AND untracked files in the same checkout. Do not
   discard the deleted legacy serializer path or omit new headers from review.
2. Finish local review of `AutoPubSampleRunner::run`,
   `RTPSAutoPubSampleEmitter_emit`, and the modified `autoPubOnDeviceData`.
   Make ownership, synchronous callback, return-value and failure semantics
   explicit. The original primary-before-secondary order is intentional.
3. Obtain an actual ESP build route before claiming wrapper validation.
   [ExampleDiscoverable/CMakeLists.txt](../examples/ExampleDiscoverable/CMakeLists.txt)
   uses RaftBootstrap v1.37.1; follow the project's Raft build instructions.
   The example Compose entry historically refers to absent `build.sh`; do not
   assume `docker compose up` is a working firmware build. Do not invent SDK
   stubs and present their compilation as an ESP result.
4. Close out this slice's devdocs updates: common sample dispatch now exists,
   but raw decoding, allocation, attach/detach and transport lifecycle are still
   in RaftROS. Record 979 tests and current resources, retaining historical counts.
   Run focused checks after any corrections and `git --no-pager diff --check`.
5. Then continue Z2 locally from attach/detach and `DynamicWriterCtx` in
   [RaftROS.h](../components/RaftROS/RaftROS.h). Plan generation-safe publisher
   handles, owned descriptors and bounded handoff with fake-backend lifecycle
   tests. Check the actual DeviceManager unregister/callback concurrency contract
   before freeing state or introducing asynchronous use. Keep 16 publisher slots
   total, not 16 devices; composites consume multiple slots.
6. Only after a usable backend boundary exists, implement mutually exclusive
   firmware builds and ESP Zenoh integration. Do not let a CMake flag claim
   isolation while the public header still embeds RTPS types/state.

## Worktree Preservation and Tool Lessons

At handoff, modified tracked files include root CMake, the two legacy RTPS
mapping/serializer headers, RaftROS.cpp, Linux Makefile/main/Dockerfile,
zenoh_metadata_interop.py and six devdocs. The old RTPS serializer `.cpp` is
deleted. Important untracked files are:

```text
components/RaftROS/AutoPub/AutoPubClassMap.h
components/RaftROS/AutoPub/AutoPubCDRSerializer.h
components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp
components/RaftROS/AutoPub/AutoPubSampleRunner.h
components/RaftROS/RTPS/runtime/autopub/RTPSAutoPubSampleEmitter.h
components/RaftROS/Zenoh/ZenohNetworkMessage.h
components/RaftROS/Zenoh/ZenohStreamFramer.h
components/RaftROS/Zenoh/ZenohTCPSession.h
linux_unit_tests/resource_report.py
linux_unit_tests/zenoh_probe_allocation_failure.cpp
linux_unit_tests/zenoh_session_probe.cpp
linux_unit_tests/zenoh_session_tests.cpp
linux_unit_tests/zenoh_wire_interop.py
devdocs/RaftROS-WSL-agent-handoff.md
```

Generated build directories and the dependency checkout may be ignored by Git.
Reopening the existing WSL folder preserves them; a patch containing only tracked
changes does not preserve this whole implementation. Recheck status after moving.

Windows Git tools reported dubious ownership for the mapped WSL checkout; native
WSL Git worked. No global safe.directory workaround was installed. Avoid line
ending churn; earlier diff checks only warned that some CRLF files normalize to LF.

In the mapped workspace, patch deletion twice reported success without deleting
the old serializer on disk; an Add File after a mechanical move also appended to
a stale editor buffer. These were repaired and tested. The final old-source
removal was gated on exact mechanically transformed equality with the new source.
Use normal patch edits in WSL, but verify actual relocation state if tools disagree.

The Docker editor sometimes reports duplicate CMD despite one CMD and successful
builds. Treat that as a known diagnostic anomaly, not justification for changing
working Docker behavior. Terminal completion notifications can replay old session
scrollback, including already-fixed errors; use the newest command results and
this handoff rather than reopening historical failures.