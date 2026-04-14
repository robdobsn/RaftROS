# RaftROS

Native ROS 2 node functionality for ESP32 via the Raft framework.

RaftROS enables ESP32 firmware built on the Raft framework to function as a native ROS 2 node, participating directly in DDS discovery and data exchange without relying on micro-ROS or any external agent/bridge process.

## Features

- Native RTPS wire protocol implementation (clean-room, MIT licensed)
- CDR serialization for standard ROS 2 message types
- SPDP/SEDP discovery — ESP32 appears as a first-class DDS participant
- Automatic mapping of Raft device data to ROS 2 topics via DeviceTypeRecords
- Automatic generation of ROS 2 services from device actions
- Integrates with Raft's StatePublisher for rate-limited, change-triggered publishing

## Dependencies

- [RaftCore](https://github.com/robdobsn/RaftCore)
- [RaftSysMods](https://github.com/robdobsn/RaftSysMods) (StatePublisher, NetworkManager)

## License

MIT — see [LICENSE](LICENSE)
