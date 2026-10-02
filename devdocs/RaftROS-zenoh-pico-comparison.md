# RaftROS: zenoh-pico as an Alternative Zenoh Session - Measured Comparison

**Date:** 2026-10-01. **Branch:** `zenoh-pico-backend` (experimental; not merged).
**Status:** complete - load, robustness, hot-plug, tuning and a 12-hour soak.

**Question:** what would it cost (flash, RAM, loop time) and what would it gain
to use Eclipse zenoh-pico for the Zenoh session instead of Raft's own
clean-room implementation - and is there anything to learn for the clean-room
code?

## What was built

A third, opt-in build option, `CONFIG_RAFTROS_BACKEND_ZENOH_PICO`. The
default stays `CONFIG_RAFTROS_BACKEND_ZENOH` (clean-room).

- zenoh-pico **1.10.1** (`e1ab223a`), the version rmw_zenoh_pico pairs with
  Jazzy, is fetched by `tools/fetch_zenoh_pico.sh` into the git-ignored
  `third_party/`. It is not in the repository and the default and RTPS
  builds never read it.
- `components/RaftROS/ZenohPico/RaftROSZenohPico.{h,cpp}` is a separate
  SysMod with the same public API. It reuses the ROS layer unchanged:
  `ZenohROSCodec` (keys, liveliness tokens, QoS strings, attachments),
  `ZenohROSIdentity` (GIDs), CDR, `AutoPubDeviceSource`,
  `AutoPubServiceRegistry` and `AutoPubParameterStore`. zenoh-pico replaces
  only the session: TCP, INIT/OPEN, framing, keep-alives, interests and
  reconnection.
- zenoh-pico is configured as a TCP client with only what RaftROS uses
  (publication, subscription, queryable, liveliness, interest, fragmentation,
  auto-reconnect; no query, scouting, UDP, multicast, serial, peer mode or
  admin space). See the `CONFIG_RAFTROS_BACKEND_ZENOH_PICO` block of
  `CMakeLists.txt`.
- `CONFIG_RAFTROS_ZENOH_PICO_TUNED` (only selectable with zenoh-pico) shrinks
  buffers (1 kB batch, 2 kB reassembly, 8 runtime tasks instead of 64), runs
  zenoh-pico's executor at the loop's priority, and pins the helper tasks to
  CPU0.

**Effect on the default build: none.** The default configuration was built
from `main` and from this branch. ESP-IDF's size reports are identical, and
the two `.bin` files differ only in build metadata (version string, build
time, ELF hash, image checksum).

## Threading: the structural difference

On ESP-IDF, zenoh-pico's client socket is **blocking with no receive
timeout** (`src/link/transport/tcp/tcp_esp32.c`; only the peer-mode path
switches to non-blocking), and its connect ignores the timeout argument. So
its single-threaded executor (`zp_spin_once`) cannot be stepped from the
Raft loop: an idle read would block the loop until the router sent
something. It runs as designed instead:

- zenoh-pico's executor is one FreeRTOS task (5120 B stack), by default at
  **priority 12 and unpinned**. It can run on the loop's core and preempt it
  (the loop is priority 1).
- A helper task runs `z_open`, which blocks on connect and handshake
  (27-2,200 ms measured). Another helper drops a session, which joins the
  executor and took about 10 s.
- Subscriber and queryable callbacks copy into two FreeRTOS queues (depth 4)
  that the loop drains; a query is cloned and held until its reply.
- Declarations, publishing and replies stay on the loop, but each one takes
  zenoh-pico's transmit lock, which its own task also holds while sending.

## Functional result

It worked first time with the unmodified ROS layer. That is independent
confirmation that the clean-room ROS mapping (tokens, keys, type hashes,
attachments, CDR) is right, because only the session changed. Node, topics,
10 services (including the deferred `/raft_esp32/range`), parameters
(list/get/set/describe/dump, refusals) and inbound `/chatter_in` all behaved
as on the clean-room build.

## Method

Same board (Adafruit ESP32-S3 TFT Feather, VL6180 at 0x29 polled every
200 ms), same router (`rmw_zenohd`, ros-jazzy-rmw-zenoh-cpp 0.2.10), same day.

**Load runs (unattended, two per configuration).** No serial monitor: each
console line costs about 10 ms when nothing reads the USB-Serial-JTAG port,
and both builds must pay that equally. Logs went to Raft's `RaftRemote` TCP
logger instead, and SysMan's statistics window was set to 660 s
(`/api/sysman?interval=660`), so one heartbeat covers each whole run. The
load (`~/ab_load.sh` on the ROS box) was 600 s of a `ros2 topic hz` subscriber
on `/raft/range_1_29`, plus every ~5 s a deferred `/raft_esp32/range` call, a
`param get` and a `param set`. The TCP logger allocates a 16 kB buffer once a
client connects, on both builds, so the heap figures in the load table are
16 kB lower than usual.

**Robustness (scripted, `robust.sh`):** raw zenoh queries (proper, empty,
900 B, 3000 B), a 3000-byte string to `/chatter_in`, a burst of 8 concurrent
range calls, a router restart after 15 s down, a `routerHost` change through
the parameter service, and a boot with no router for 60 s.

**Limits of the method.** SysMan names only the two slowest modules in a
window, so RaftROS's worst pass per window is sometimes known only as a bound.
`rosstat`'s per-phase maxima run from boot, so they include start-up. They
are directly comparable between builds, because both had the same boot and
the same two runs.

## Results

### Flash and static RAM

| | Clean-room (default) | zenoh-pico | Difference |
| --- | --- | --- | --- |
| Total image | 1,278,743 B | 1,328,015 B | **+49,272 B** |
| Flash code (`.text`) | 990,344 B | 1,041,148 B | +50,804 B |
| Flash data (`.rodata`) | 198,840 B | 197,256 B | −1,584 B |
| Static RAM (`.data` + `.bss`) | 39,948 B | 39,980 B | +32 B |
| libRaftROS.a | 44,919 B | 94,513 B | +49,594 B |
| ... the SysMod | 39,641 B (session included) | 32,776 B | |
| ... zenoh-pico (56 objects) | - | 56,459 B | |
| ... shared AutoPub/CDR | 5,278 B | 5,278 B | |

### Load runs (unattended)

| | Clean-room | zenoh-pico | zenoh-pico tuned |
| --- | --- | --- | --- |
| Calls / failures (2 runs) | 870 / **0** | 825 / **0** | 849 / **0** |
| Sessions during runs | 1 | 1 | 1 |
| Range at the host | 4.89 Hz | 4.90-4.92 Hz | 4.90-4.91 Hz |
| Free heap under load (incl. 16 kB logger) | 127.6-129.6 kB | 89.4-90.5 kB (**−38.5 kB**) | 92.9-94.6 kB (−34.5 kB) |
| Minimum free heap since boot | 115.6-117.6 kB | 61.5 kB | 53.3 kB |
| Largest free block | not reported | 49.2 kB | 31.7 kB |
| Loop average (660 s window) | 0.74 ms | 0.59 ms | 0.56 ms |
| RaftROS worst pass in a window | 3.7 ms; < 14.3 ms | < 9.5 ms; < 9.8 ms | ≤ 11.3 ms; 9.8 ms |
| Worst publishing (sample drain) pass since boot | **1.4 ms** | 5.8-8.3 ms | 8.3 ms |
| Worst send-side pass since boot | 12.3 ms | 19.4 ms | **43.0 ms** |
| Worst single publish / reply | ~0.8-1.8 ms per datagram (earlier measurement) | 4.0 / 5.3 ms | 4.7 / 5.5 ms |

Heap for one session, from boots without a router: free heap with no session
was 160.9 kB on zenoh-pico and 146.8 kB on the clean-room build, against
105-107 kB and 144-147 kB with a session up. So **a zenoh-pico session costs
about 54 kB of heap, and a clean-room session about 2.5 kB**. Why the
zenoh-pico image has 14 kB more free before any session was not
investigated.

### Robustness

| Test | Clean-room | zenoh-pico |
| --- | --- | --- |
| Raw query, proper / empty | OK / `ERR bad request` | OK / `ERR bad request` |
| Raw query 900 B / 3000 B | OK / `ERR bad request` | "no reply" / "no reply" (the zenoh-pico glue drops requests over 512 B instead of refusing them) |
| 3000-byte string to `/chatter_in` | dropped and counted, session kept | dropped and counted, session kept |
| 8 concurrent range calls | 1 OK, the rest refused, service still answers | 2 OK, 6 refused (1 by the 4-deep queue), service still answers |
| Router stopped 15 s then restarted | reports `disconnected`; data again **12 s** after restart | reports `ready` throughout (internal reconnect invisible); data again **1 s** after restart |
| `routerHost` change (×3) | 3 clean reconnects in < 1 s; worst pass 13.2 ms | **first version crashed** (see below); after the fix, 3 clean reopens, each taking **about 11 s**; worst publishing pass 23 ms during the reopen |
| Boot with no router for 60 s | `disconnected`, 5 failures, worst pass 12.6 ms; ready 9 s after router start | `disconnected`, 5 failures (`z_open` −102), worst pass 10.7 ms; ready 6 s after router start |

**The crash** (`LoadProhibited` in `z_liveliness_declare_token` →
`_z_transport_tx_mutex_lock`). The first version of the zenoh-pico glue
opened the replacement session while the old one was still closing, with the
same session id. The new session's transport went away, and zenoh-pico
dereferenced the missing transport on the next declaration instead of
returning an error. Fixed in the glue: a new session opens only after the old
one has closed. The clean-room build never hits this, because it closes its
socket before reconnecting.

**Preemption.** During zenoh-pico's session reopen, a publishing pass took
23 ms while the publishes themselves took 0.4 ms. The difference is the
helper task (priority 5) and executor (priority 12) running on the loop's
core.

### Hot-plug and graph visibility (2026-10-01 evening)

| Test | zenoh-pico |
| --- | --- |
| Sensor unplugged | The device withdrew the range publisher within one 2 s sample (2 publishers → 1); re-declared within one sample of replugging, nothing left pending |
| Deferred range call with the sensor unplugged | Answered with an error at the registry timeout (`svcTimedOut` +1), as on the clean-room build |
| Router's view after two unplug cycles | **Three** range publisher tokens from the same session (entity ids 4, 5, 6): two withdrawals never reached the router. That build released the token with `z_drop`, which discards the result. |
| After switching to explicit `z_liveliness_undeclare_token` with the result counted | One cycle: the withdrawal succeeded (0 failures) and the router held exactly the new token. The earlier failure was intermittent; its cause is not established. A replug that bounces may be involved (entity 6 was one attach more than the replugs). |

**Router-side visibility of the device's endpoints.** A raw liveliness query
for the range publisher, once a second for 40 s:

| | Clean-room | zenoh-pico |
| --- | --- | --- |
| Queries in which the publisher was missing | **0 / 40** | **10 / 40**, in bursts of about 3 s roughly every 13 s |

So fresh ROS tools (each `ros2` command is a new node querying the graph)
intermittently do not see the zenoh-pico node's publisher, although it is
declared throughout. This explains the flicker seen in `ros2 topic list`.
The regular period suggests something cyclic in zenoh-pico's session (lease
or keep-alive handling), but the cause has not been established.

### Tuning (`CONFIG_RAFTROS_ZENOH_PICO_TUNED`)

Smaller buffers and 8 runtime tasks recover **about 4 kB** of the 38 kB
heap gap and leave the heap more fragmented (largest block 31.7 kB against
49.2 kB). Dropping the executor to the loop's priority made the worst
send-side pass **worse** (43 ms against 19 ms): at equal priority, the loop
can wait a time slice for the lock the executor holds. Most of zenoh-pico's
memory cost is not configuration. The loop interference comes from the
shared transmit lock; priorities only move it around.

### Soak (12 h)

Untuned zenoh-pico, 2026-10-01 21:35 to 2026-10-02 09:35, with the same driver
as the clean-room soak (`~/soak2` scripts copied to `~/soak3`): a subscriber
counting range and `/chatter` per minute; every minute the devices, range and
ping calls, a parameter get and a set; a parameter dump every 10 minutes;
`rosstat` every minute. The settings overlay was empty, as for the clean-room
soak.

| 12 h soak | Clean-room (2026-09-29/30) | zenoh-pico (2026-10-01/02) |
| --- | --- | --- |
| Service/parameter calls | 3,600, **0 failures**; 72 dumps, 0 failures | 3,600, **0 failures**; 72 dumps, 0 failures |
| Requests accepted / completed / timed out | 3,744 / 3,744 / 0 | 3,744 / 3,744 / 0 |
| Sessions | 1 | 1 |
| Free heap, first-hour / last-hour mean | 146,971 / 146,797 B (−174 B) | 107,273 / 107,309 B (+36 B) |
| Minimum free heap | 114.3 kB (reached by minute 20) | 71.2 kB (reached in minute 1) |
| Worst loop pass | 13.2 ms | 20.8 ms |
| Stack headroom | 5,564 B | 5,580 B |
| Range / `/chatter` per minute at the host | 286-299 / 58-67 | 285-298 / 59-67 |
| Samples published | 256,985 | 257,738 |

Both are leak-free and fully reliable over 12 h. The differences are the ones
the shorter tests showed: about 40 kB less heap and a higher worst pass for
zenoh-pico.

## Not tested

- The cause of zenoh-pico's intermittent failed token withdrawal, and of
  its periodic absence from liveliness queries.
- **Higher data rates**, for example an IMU at full rate. Lock contention may
  grow with load; only about 6 messages a second were tested.
- **Heap against ROS graph size.** A zenoh-pico session records the router's
  declarations for the rest of the graph, so its heap may grow with the number
  of ROS nodes on the network. This is a hypothesis; only a small graph was
  used here.
- Peer (routerless) mode.

## Conclusions

1. **Cost:** about **49 kB of flash** and **35-39 kB of heap** more than the
   clean-room session (about 54 kB for a zenoh-pico session against 2.5 kB),
   plus a FreeRTOS task with its own stack, cross-task queues, and
   preemption of the loop. Tuning recovers about 4 kB.
2. **Loop behaviour:** a lower average (receive work is no longer charged to
   the loop), but worse worst cases. Publishing passes reach 8 ms against
   1.4 ms, and send-side passes 19-43 ms against 12 ms. All stayed within the
   50 ms contract on this load, but with less margin.
3. **Robustness:** both completed a 12-hour soak with 0 failures in 3,600
   calls and flat heap. Equal on malformed and oversized traffic. zenoh-pico
   recovered faster from a router restart (1 s against the clean-room's 12 s
   at the time; the clean-room build now takes 0.55 s), but hides the outage,
   and reopening a session is slow (about 11 s). It also needed a workaround
   for a crash in zenoh-pico 1.10.1 when declaring on a session whose
   transport has gone.
   **Graph visibility is worse.** zenoh-pico's publisher was missing from 10
   of 40 router liveliness queries (none of 40 for the clean-room build), so
   fresh ROS tools intermittently do not see it. In early hot-plug cycles,
   withdrawn publishers stayed in the graph (cause not established).
4. **For the clean-room code:**
   - **Router-restart recovery was slow, and is now fixed.** The reconnect
     backoff doubled up to 30 s during an outage, so recovery took 12 s. Now
     (2026-10-01) a lost live session is retried after the minimum delay,
     and refused connects back off only to 4 s; a host that does not answer
     still backs off to 30 s. Measured: session ready **0.55 s** after the
     router was listening again (two runs, 15 s outages), worst pass
     14.5 ms.
   - Keeping all session work on the loop, with one datagram out per pass and
     no locks, gives the better worst case; this comparison confirms the
     design.
   - The shared ROS layer is validated independently of the clean-room
     session.

**Recommendation:** keep the clean-room session as the default. Keep this
branch as a measured reference. zenoh-pico is only worth revisiting if
routerless peer mode becomes a requirement.

## Reproducing

```bash
tools/fetch_zenoh_pico.sh
# examples/ExampleDiscoverable/systypes/SysTypeMain/sdkconfig.defaults:
#   CONFIG_RAFTROS_BACKEND_ZENOH_PICO=y        (optionally CONFIG_RAFTROS_ZENOH_PICO_TUNED=y)
rm -f examples/ExampleDiscoverable/build/SysTypeMain/raft/sdkconfig
cd examples/ExampleDiscoverable && raft build --no-docker -e ~/esp/esp-idf-v6.0.2 .
```

`raft build` regenerates the build's sdkconfig from `sdkconfig.defaults`, so
the defaults line must still be present when building. Restoring the file
before a rebuild silently builds the clean-room image.

Build notes:
- zenoh-pico's ESP-IDF header includes `driver/uart.h` even with serial links
  off. A component dependency cannot be made conditional on Kconfig, because
  ESP-IDF resolves dependencies before the configuration is loaded, so the
  UART header paths are added to this build only.
- `zenoh-pico.h` must not be wrapped in `extern "C"`, because it supplies C++
  overloads of `z_loan`/`z_move`/`z_drop`.
- Its headers need `-Wno-missing-field-initializers` under this component's
  C++ warnings.

Test-harness notes: the `ros2` CLI ignores SIGTERM, so use
`timeout -s KILL`. A refused service call never returns (rmw_zenoh logs
`z_reply_is_ok returned False` and keeps waiting). When killing processes
over ssh, use bracketed `pgrep -f "[p]attern"`, or the pattern matches the ssh
command itself.
