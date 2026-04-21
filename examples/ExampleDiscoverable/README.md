# ExampleDiscoverable — RaftROS Phase 1 + Phase 2 Demonstrator

A minimal Raft ESP32 application that brings up the RaftROS SysMod so the ESP32 becomes a native ROS 2 node. Generated originally by `raft new` and extended with the RaftROS SysMod.

## What it does

When connected to WiFi and booted:

- Registers as DDS participant `/raft_esp32` on domain 0.
- Responds to ROS 2 SPDP/SEDP discovery so `ros2 node list` and `ros2 topic list` see it as a first-class node.
- Publishes `std_msgs/msg/String` on `/chatter` at 1 Hz with a payload like `Hello from raft_esp32 [N]` (RELIABLE + VOLATILE QoS).
- Handles incoming HEARTBEAT / ACKNACK and retransmits lost SEDP and user-data DATA submessages.

## Build and flash

The simplest way to build this application is to [use the raft command line interface](https://github.com/robdobsn/RaftCLI):

```bash
raft run
```

Once flashed, configure WiFi via the serial console the first time:

```
w/<SSID>/<password>
```

## Verify from a ROS 2 host

On a Linux host on the same network with ROS 2 Humble installed:

```bash
env -u PYTHONPATH PYTHONNOUSERSITE=1 bash -lc '
  source /opt/ros/humble/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0

  ros2 node list --no-daemon
  # expect: /raft_esp32

  ros2 topic list --no-daemon
  # expect to include: /chatter

  ros2 topic info /chatter --no-daemon -v
  # expect: one publisher, RELIABLE + VOLATILE, from node raft_esp32

  ros2 topic echo /chatter std_msgs/msg/String --no-daemon
  # expect: "data: Hello from raft_esp32 [N]" once per second
'
```

The `env -u PYTHONPATH PYTHONNOUSERSITE=1` and `--no-daemon` parts avoid a common user-site numpy collision and a ROS 2 daemon XMLRPC timeout when running against a non-local DDS participant.

## Configuration

The RaftROS SysMod and the overall system configuration live in `systypes/SysTypeMain/SysTypes.json`. The relevant RaftROS fields are the domain ID, the ROS 2 node name (`raft_esp32` by default), the SPDP announce interval, and the participant lease duration.

## Further reading

- [RaftROS overview and design](../../devdocs/RaftROS-overview.md)
- [RaftROS development status](../../devdocs/RaftROS-development-status.md)
- [RaftROS next stages plan](../../devdocs/RaftROS-next-stages-implementation-plan.md)
- [Raft command line documentation](https://github.com/robdobsn/RaftCLI)
- [Raft Framework documentation](https://github.com/robdobsn/RaftCore/wiki)
