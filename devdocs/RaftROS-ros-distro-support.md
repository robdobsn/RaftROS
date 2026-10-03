# RaftROS: ROS 2 Distribution Support (Jazzy, Kilted, Lyrical)

**Date:** 2026-10-02/03. **Firmware:** the default Zenoh build, unchanged across
all three distributions: one image serves Jazzy, Kilted and Lyrical.
**Status:** Zenoh functional, robustness, load and hot-plug tests passed on all
three. 12-hour soaks on Kilted and Lyrical are **running** (results pending).

## Why one image works

What ties RaftROS to a ROS distribution is the type hashes it compiles in, plus
`rmw_zenoh`'s wire format. Both were checked before testing:

- **Type hashes:** all 23 types RaftROS carries (the `sensor_msgs`, `std_msgs`,
  `geometry_msgs`, `std_srvs` and `rcl_interfaces` types it publishes or serves)
  have identical `RIHS01_` hashes in Jazzy, Kilted and Lyrical. They were read
  from each distribution's installed `share/<pkg>/{msg,srv}/*.json`. Between the
  three, upstream changed only interfaces RaftROS does not use (`LoadMap`,
  `Pose2D`, `PointField`, `TransitionEvent`, some test messages).
- **`rmw_zenoh` wire format:** liveliness-token, attachment and QoS code is
  identical between Jazzy (0.2.11 branch) and Kilted (0.6.x). Lyrical (0.10.x)
  adds an *optional* trailing "backends" field to tokens, for buffer message
  types only. Tokens without it, like RaftROS's, remain valid. The maintainers
  have committed to wire compatibility from Kilted onwards.

## Test setup

Each distribution runs in a container built from `docker/distros/Dockerfile`
(`ros:<distro>-ros-base` plus `rmw_zenoh`, CycloneDDS and zenoh-python), with
host networking on the ROS host (base8ubuntu, Ubuntu 26.04). Its router
therefore sits at the address the board already uses, and only one
distribution's router runs at a time. Board: ESP32-S3 TFT Feather with a VL6180.

| | Jazzy | Kilted | Lyrical |
| --- | --- | --- | --- |
| Base image | Ubuntu 24.04 | Ubuntu 24.04 | Ubuntu 26.04 |
| `rmw_zenoh_cpp` | 0.2.10 | 0.6.8 (zenoh-c 1.8.0) | 0.10.6 |
| CycloneDDS | 2.2.4 | 4.0.3 | 4.1.5 |

Scripts (in `docker/distros/`):
- `run_suite.sh`: bring-up with a tshark capture, functional checks,
  robustness, graph visibility, functional checks again, 10-minute load.
- `run_hotplug.sh`: the router's tokens beside the device's view, once a second.
- `run_soak.sh`: 12 h with the earlier soaks' traffic.
- `run_rtps.sh`: RTPS-build checks.

Results are written to `~/distro-tests/<distro>/` on the ROS host.

## Results: Zenoh build

| | Jazzy | Kilted | Lyrical |
| --- | --- | --- | --- |
| Functional checks (before / after robustness) | 19/19, 19/19 | 19/19, 19/19 | 19/19, 19/19 |
| Raw queries: proper / empty / 900 B / 3000 B | OK / bad request / OK / bad request | same | same |
| 3000-byte string to `/chatter_in` | dropped, session kept | same | same |
| 8 concurrent deferred calls | 1 served, rest refused, service still answers | same | same |
| Graph visibility (router liveliness, 40 queries) | 40/40 | 40/40 | 40/40 |
| Router restart, 15 s down: session back after | 0.82 s, 1.07 s | 0.83 s, 0.82 s | 0.83 s, 0.30 s |
| `routerHost` set via parameter: reconnected after | 0.32 s | 0.32 s | 0.32 s |
| Boot with no router: ready after router start | 4.1 s | 0.57 s | 0.31 s |
| Hot-plug: token withdrawn / re-declared | - (verified 2026-09-27) | same second / new token, no stale | same second / new token, no stale |
| 10-min load: calls / failures / dumps | 600 / 0 / 12 | 600 / 0 / 12 | 600 / 0 / 12 |
| Load: heap drift / minimum | −62 B / 115.9 kB | −263 B / 131.5 kB | −310 B / 124.8 kB |
| Load: worst loop pass | 14.0 ms | 14.2 ms | 13.0 ms |
| Load: range / chatter per minute at host | 293-295 / 63-64 | 292-296 / 63-64 | 292-296 / 63-64 |
| 12 h soak | (2026-09-29/30: 0 failures in 3,600 calls) | **pending** | **pending** |

The functional checks cover:
- `node list` and `node info`: publishers, subscriptions, 10 services
- `topic info -v`: node attribution, type hash, QoS
- echo of range and `/chatter`
- inbound `/chatter_in`, counted on the device
- `ping` (Empty), deferred `range` (Trigger) and `chatter_enable` (SetBool)
- parameter list, get, set (refused and accepted), describe and dump

The boot-without-router times differ only because of where the device's retry
back-off was when the router started.

Session bring-up captures (`session.pcapng`, 539-556 packets from a device
reset) are kept per distribution. On Ubuntu 26.04, tshark's AppArmor profile
cannot read files outside `/tmp`, so copy them there first.

## Results: RTPS build (for reference only)

CycloneDDS: 9/9 checks on all three distributions (node in the graph, type
hash, QoS, echo, and inbound confirmed in the device log). Fast DDS: data
flows both ways on all three, but node attribution fails
(`_NODE_NAME_UNKNOWN_`) on all three, Jazzy included. It worked natively on
2026-09-27, so the cause is either the container environment or an RTPS
change since then. Not investigated; the current focus is Zenoh.

## Not covered

- Humble (no type hashes, and a different `rmw_zenoh` key format; it would
  need its own mode).
- Mixed distributions in one graph. Not supported by `rmw_zenoh` before
  Kilted, and not tested.
