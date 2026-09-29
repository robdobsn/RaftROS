# RaftROS Parameters: Implementation Plan

Written 2026-09-28, after the services plan
([RaftROS-services-implementation-plan.md](RaftROS-services-implementation-plan.md))
completed S0-S5. ROS 2 parameters are six standard services per node, so
this plan is a layer on that work: the same session, registry, staging and
loop rules, plus a codec for `rcl_interfaces` and a small parameter store.

## 0. Decisions

| Question | Decision | Why |
| --- | --- | --- |
| Transport | Zenoh only (the user ruled RTPS services out on 2026-09-28) | One queryable and one token per service; six services cost the same on the loop as one. |
| Which services | All six: `list_parameters`, `get_parameters`, `get_parameter_types`, `set_parameters`, `set_parameters_atomically`, `describe_parameters` | `ros2 param list/get/set/describe/dump` between them use all six; a node missing one looks broken to the tools. |
| Types | `bool`, `int64`, `double`, `string`. Arrays and byte arrays are described (type reported) but never held; a set with one is refused with a reason. | Covers what a Raft SysMod configures; array handling would double the codec for no user. |
| Which parameters | The SysMod's own (`chatterEnable`, `routerHost`) and any the application declares with `declareParameter(name, value, onSet)`. `use_sim_time` is declared `false`, read-only. | `use_sim_time` is on every ROS 2 node and the tools expect it; the rest is the actual use. |
| Persistence | A set changes the running value; the declaring code's `onSet` callback may persist it. `routerHost` persists through the same overlay `/api/postsettings` writes, so `ros2 param set /raft_esp32 routerHost <ip>` survives a reboot. | Persistence is a policy of each parameter, not of the transport. |
| `/parameter_events` | Not published. | Nothing in the CLI needs it; it would be a permanent publisher on every set for no reader. |
| Bounds | 16 parameters; names up to 48 characters; string values up to 64; replies up to 1024 bytes (`REPLY_MAX`) so `ros2 param dump` on a full table fits in one reply. | Static tables, as everything else in the SysMod. |

## 1. The Wire Contract

The services are `rcl_interfaces::srv::dds_::<Name>_` at
`<ns>/<node>/<name>`, with the Jazzy type hashes (from
`/opt/ros/jazzy/share/rcl_interfaces/srv/*.json`):

| Service | Request | Response | RIHS01 |
| --- | --- | --- | --- |
| ListParameters | `string[] prefixes; uint64 depth` | `ListParametersResult result` (`string[] names; string[] prefixes`) | `3e6062bf...` |
| GetParameters | `string[] names` | `ParameterValue[] values` | `bf9803d5...` |
| GetParameterTypes | `string[] names` | `uint8[] types` | `da199c87...` |
| SetParameters | `Parameter[] parameters` | `SetParametersResult[] results` | `56eed9a6...` |
| SetParametersAtomically | `Parameter[] parameters` | `SetParametersResult result` | `0e192ef2...` |
| DescribeParameters | `string[] names` | `ParameterDescriptor[] descriptors` | `845b484d...` |

`ParameterValue` is every field serialised in order (`uint8 type`, `bool`,
`int64`, `float64`, `string`, then five sequences, empty unless used), so a
`double` parameter still carries ~30 bytes. `ParameterDescriptor` carries
name, type, description, additional_constraints, two bools and two
bounded sequences (`FloatingPointRange[<=1]`, `IntegerRange[<=1]`) which we
always send empty. Everything is CDR 1.0 LE with the 4-byte header the
existing codec already writes; the only alignment cases beyond `std_srvs`
are `int64`/`float64` at 8 and sequences of structs. The exact bytes for
each exchange come from P0 and become fixtures, as in the services plan.

## 2. Loop, Memory and Failure Bounds

Every parameter service answers from the store on the handler's pass -
nothing defers, nothing touches a bus - so the cost is one reply build (a
few hundred bytes of CDR) plus the existing two send passes. `REPLY_MAX`
rising from 256 to 1024 costs 3 kB of in-flight buffers (4 x 768 B more);
the store is 16 x ~140 B. The service table grows from 4 to 12 slots (six
parameter services plus up to six of the application's). A set that fails
validation is answered `successful=false` with the reason, never `ERR`;
a request naming an unknown parameter gets a `PARAMETER_NOT_SET` value, as
rclcpp does. Unknown or array types in a set are refused per parameter.

## 3. Slices and Gates

**P0 - Capture and contract.** A tiny `rclpy` node with one parameter of
each supported type, in client mode so its traffic crosses the router on
`lo`; `ros2 param list`, `get` (each type), `set` (each type, one with the
wrong type, one unknown name), `describe`, `dump` under `tshark`. Decode
each request and response; record them as fixtures.
*Gate:* every exchange decodes completely by hand; the fixtures cover all
six services and the two failure results.
**Met 2026-09-28.** 1148 TCP segments; every request and response the
`ros2 param` tools sent decoded completely (a Python decoder that reads
only request/response batches and names each request by the response with
the same id). What the CLI actually does: `list` is one `ListParameters`
with no prefixes and depth 0; `get` is one `GetParameters` with one name;
`set` is one `SetParameters` with no `describe` before it; `describe` is
one `DescribeParameters`; `dump` is `ListParameters` then one
`GetParameters` naming every parameter, sorted. `GetParameterTypes` and
`SetParametersAtomically` are not used by the CLI at all (their codecs
follow the definitions; no fixture). Failure texts a Jazzy node returns:
`Wrong parameter type, expected 'Type.INTEGER' got 'Type.STRING'`,
`Trying to set a read-only parameter: routerHost.`, `Invalid access to
undeclared parameter(s): []`. Sizes: a scalar `ParameterValue` is 52 bytes
of CDR body, a descriptor ~76, a six-parameter `dump` reply 312 bytes - so
`REPLY_MAX` 1024 covers a 16-parameter table. A stock Jazzy node also lists
`start_type_description_service` and serves `get_type_description`; we do
neither. Fixtures: 14 files `linux_unit_tests/fixtures/zenoh_param_*.hex`.

**P1 - `rcl_interfaces` codec and the raw service kind.**
`AutoPub/AutoPubParamCodec.h`: readers for `string[]`, `Parameter[]`
(with `ParameterValue`), `prefixes+depth`; writers for `ParameterValue[]`,
`uint8[]`, `SetParametersResult[]`/single, `ListParametersResult`,
`ParameterDescriptor[]`. The registry gains a *raw* service kind whose
handler receives the request CDR and writes the response CDR itself, so
the six services do not each need a `AutoPubServiceKind`.
*Gate:* fixtures re-encode byte for byte; truncations of every request
rejected; mutation checks on the writers.
**Met 2026-09-29.** `AutoPub/AutoPubParamCodec.h`; every captured response
re-encodes byte for byte (the two multi-string ones equal but for padding
bytes, which FastCDR leaves uninitialised - the test masks exactly those);
every captured request decodes; each truncation of a set request is
rejected; array values are recognised and stepped over. The registry's Raw
kind hands the request CDR to the handler and copies the response CDR.

**P2 - Parameter store.** `AutoPub/AutoPubParameterStore.h`: declare
(name, type, value, description, read-only, on-set callback), get, set
with type and read-only checks, list with prefixes and depth, describe.
*Gate:* host tests for each service's semantics including the recursive
and prefixed list cases and the atomic set (all or nothing).
**Met 2026-09-29.** `AutoPub/AutoPubParameterStore.h`: declare (with
description, read-only flag, owner callback), list with prefixes and depth
and the derived prefixes, get/types/describe with `NotSet` for unknown
names, set with per-parameter results in the Jazzy node's own words, atomic
set (the store's checks all pass before any callback runs; a refusing
callback stops the rest - what earlier callbacks applied stays, which the
header says). The tests caught one bug on the way: formatted refusal
reasons shared one buffer, so a request refusing two parameters reported
the last reason twice. Unit suite 1123 passed (+59 over P0).

**P3 - SysMod integration (first flash).** RaftROS declares the six
services for its node at setup, maps `chatterEnable` and `routerHost`
(with persistence), declares `use_sim_time`, and offers
`declareParameter()` to the application. `rosstat` gains `params`.
*Gate, on hardware:* `ros2 param list /raft_esp32` shows the parameters;
`get`/`set`/`describe`/`dump` work for each type; a wrong-type set and a
read-only set report their reasons; `ros2 param set /raft_esp32
chatterEnable false` silences `/chatter`; `routerHost` set survives a
reboot; loop maxima unchanged; image and heap deltas recorded.
**Met 2026-09-29** on the ProS3 against `rmw_zenohd` 0.2.10: `ros2 param
list/get/describe/dump` for every parameter; sets of each type; refusals
for wrong type, read-only, undeclared name, an invalid `routerHost` and an
out-of-range `chatterPeriodMs`, each with its reason; `chatterEnable false`
silences `/chatter`, `chatterPeriodMs 250` gives 4.02 Hz on `ros2 topic
hz`; a `routerHost` set persists to the overlay (`routerSource` becomes
`config`), keeps an unrelated posted section intact, and the node
reconnects once the reply is out. Five defects found and fixed on the way:
(1) the reply and its `RESPONSE_FINAL` went on separate passes and the
client logged "ResponseFinal for unknown Request" - they now share one
frame, as a real server sends them; (2) the store's per-request arrays sat
on the loop task's stack (headroom 5540 -> 4516 B) - now members, headroom
5544 B; (3) reusing those arrays leaked a previous refusal's reason into a
later success; (4) the overlay merge read the chained config and froze the
whole base `RaftROS` section into NVS - it now merges the overlay document
alone; (5) RaftCore's `RaftJson` loses every key after an object holding an
escaped quote, so the merge rebuilds from leaves, and a RaftCore patch is
in `devdocs/patches/raftcore-json-escaped-quote.patch` (not applied).
Worst pass 14.5 ms unattended (a `routerHost` set: the NVS write); image
1272 kB (28% of the slot free); free heap 145.8 kB against 172 kB before
services (12 service slots with 1 kB replies, the parameter table and its
working space).

**P4 - Example and documentation.** The example declares one or two
parameters of its own with an `onSet` (e.g. the chatter period), the README
gets a *Parameters* section, the results document a row.
**Met 2026-09-29.** The SysMod adds `chatterPeriodMs` (validated
100-60000); the example declares `rangeOffsetMm` and reads it where it is
used - `/raft_esp32/range` answered "range 259.5 mm (offset 4.5)" after a
set. README *Parameters* section; results row added.

## 4. Validation Workflow

As for services: host suite on every slice, mutations on the writers, the
board from P3 with `rmw_zenohd` 0.2.10 and the `ros2 param` tools, `rosstat`
read unattended for loop figures, a router restart for re-declaration.

## 5. Acceptance Matrix

| | Required | How it is shown |
| --- | --- | --- |
| All six services in the graph | yes | `ros2 service list -t \| grep raft_esp32` |
| `ros2 param list/get/set/describe/dump` | yes | each against the board |
| Type safety | yes | wrong-type set refused with reason; read-only refused |
| Atomic set | yes | one bad parameter fails the whole set, nothing changes |
| Persistence | `routerHost` | set, reboot, `rosstat` shows the new router |
| Loop and memory | within budget | maxima unchanged; deltas in the results doc |

## 6. Risks

- The CLI may use services or fields not obvious from the definitions
  (`ros2 param set` may call `describe` first, or send `ListParameters`
  with `depth` non-zero). P0 exists to see this before any code.
- `ParameterDescriptor`'s bounded sequences: rmw serialises them as plain
  sequences (length 0); if the capture shows otherwise the writer follows
  the capture.
- Persisting `routerHost` uses RaftCore's settings overlay; the exact call
  the `/api/postsettings` handler makes is what P3 reuses, not a new path.
