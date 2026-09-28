# RaftROS Services: Assessment (not yet a plan)

Written 2026-09-28, after the initial Zenoh milestone. This records what exists
and what a services implementation would need, so a plan can be written against
facts. It is deliberately not the plan; the plan is
[RaftROS-services-implementation-plan.md](RaftROS-services-implementation-plan.md).

## What exists today

- In the docs: one bullet - "ROS 2 actions / service servers" - under *Phase 5:
  Integration with Raft* in the status document, and the Zenoh plan explicitly
  scoping out inbound `PUSH/REQUEST/RESPONSE`.
- In the code: nothing request/reply-shaped on either transport. The RTPS
  runtime has no `rq/`/`rr/` topics, no SampleIdentity handling and no
  related-sample inline QoS; the Zenoh wire layer has no queryable declaration,
  no REQUEST or RESPONSE message and no reply encoding.

So a plan has to be written before code, and the first design question is
which transport, or both.

## What a service needs, per transport

**Shared, transport-neutral core** (one copy, as `AutoPubDeviceSource` is):
a service registry (name, request and response type names and hashes, a
handler), CDR decode of the request and encode of the reply, and the same
discipline the publish and subscribe paths already follow - resolved at
announce time, handled on the loop task, never blocking.

**Zenoh** (the smaller half). A ROS 2 service over `rmw_zenoh` is a *queryable*
on `<domain>/<service>/<type>/<hash>` with an `SS` liveliness token. A request
arrives as a REQUEST network message carrying the query and an attachment; the
server answers with a RESPONSE (reply, then final). Needed:

- `declareQueryable` / `undeclareQueryable` (declaration ids 4/5 in the
  numbering the token code already uses).
- REQUEST parsing and RESPONSE encoding in `ZenohNetworkMessage`, and the
  session delivering requests to a handler the way it delivers samples.
- Method that worked for `Put`: capture the bytes from a real `rmw_zenoh`
  client with `tshark` against the router, read `zenoh-codec` for the layout,
  keep the capture as a test.
- Proof: `ros2 service call` against the device.

Estimate: days, including the real-tool proof.

**RTPS** (the larger half). DDS-RPC: a request topic `rq/<service>Request` and
a reply topic `rr/<service>Reply`, so each service is two endpoints - a
reliable reader for requests and a reliable writer for replies - announced
through SEDP like today's endpoints, with the reply carrying the request's
SampleIdentity in inline QoS (`PID_RELATED_SAMPLE_IDENTITY`) so the client can
correlate. All of that is new; the announce and heartbeat sequences grow by two
steps per service, and reliability on the request reader is what the
subscription path deliberately kept simple. Proof: `ros2 service call` under
CycloneDDS and FastDDS.

Estimate: a couple of weeks, and where the risk lives.

## Recommended order

1. Shared core and Zenoh first. It proves the application API and the demo on
   the transport that is now the default, and it is contained.
2. RTPS second, later, or - a decision worth making explicitly - not at all for
   a first release, given the measured loop-cost gap between the transports
   (see [RaftROS-zenoh-milestone-results.md](RaftROS-zenoh-milestone-results.md)).

## Interaction with a running soak

Design, host code, unit tests and firmware *builds* do not touch a soaking
device. Only a flash does: it reboots the device, resets `heapMinB` and adds a
reconnect, and the run restarts from there.
