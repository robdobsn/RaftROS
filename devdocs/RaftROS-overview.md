# RaftROS — Native ROS 2 Node Functionality for ESP32 via the Raft Framework

## 1. Introduction and Motivation

### 1.1 Goal

RaftROS aims to make ESP32 firmware, built on the Raft framework, function as a **native ROS 2 node** — participating directly in ROS 2 discovery and data exchange without relying on micro-ROS or any external agent/bridge process.

### 1.2 Why Not micro-ROS?

micro-ROS is the standard approach for bringing ROS 2 to microcontrollers. It uses the DDS-XRCE (eXtremely Resource Constrained Environments) protocol, which requires a **host-side agent** process to bridge between the microcontroller and the full DDS network. This introduces several limitations:

| Limitation | Impact |
|-----------|--------|
| **Agent dependency** | A separate process must run on a host machine at all times. If the agent crashes or loses connectivity, the MCU is disconnected from ROS |
| **Bridged architecture** | The MCU is not a first-class DDS participant — it cannot be discovered by other nodes without the agent intermediary |
| **Serial/UDP transport only** | XRCE-DDS typically communicates over UART or UDP to the agent, not directly on the DDS multicast network |
| **Limited expressiveness** | The MCU must pre-declare its topics/services; dynamic creation and discovery are constrained |
| **No auto-configuration** | micro-ROS cannot leverage Raft's dynamic device detection and self-describing data model — the mapping from devices to ROS topics must be manually coded |

### 1.3 The RaftROS Vision

RaftROS would allow an ESP32 running Raft firmware to:

1. **Appear as a native ROS 2 node** on the DDS network (discoverable by `ros2 node list`, `ros2 topic list`, etc.)
2. **Auto-generate ROS 2 publishers** from dynamically detected devices (I2C sensors, BLE peripherals) using DeviceTypeRecords metadata
3. **Auto-generate ROS 2 services** from device actions (servo control, LED configuration, etc.)
4. **Subscribe to ROS 2 topics** to receive commands from the ROS ecosystem
5. **Self-describe** its capabilities so that ROS tools can introspect the ESP32's device graph without prior configuration

---

## 2. Architecture Context

### 2.1 ROS 2 / DDS Architecture (What We Need to Interoperate With)

ROS 2 is built on top of DDS (Data Distribution Service), using the RTPS (Real-Time Publish-Subscribe) wire protocol. The key architectural layers are:

```
┌─────────────────────────────────────────────────┐
│  ROS 2 Application (Nodes, Executors)           │
├─────────────────────────────────────────────────┤
│  RCL  (ROS Client Library)                      │
├─────────────────────────────────────────────────┤
│  RMW  (ROS Middleware Interface)                │
├─────────────────────────────────────────────────┤
│  DDS Implementation                             │
│   (Fast DDS / Cyclone DDS / Connext / Zenoh)    │
├─────────────────────────────────────────────────┤
│  RTPS Wire Protocol (DDSI-RTPS)                 │
│   - UDP multicast discovery (SPDP/SEDP)         │
│   - UDP unicast data exchange                   │
│   - CDR serialization                           │
└─────────────────────────────────────────────────┘
```

**Key RTPS concepts RaftROS must implement or interface with:**

| Concept | Description |
|---------|-------------|
| **Participant** | A node on the DDS network. Each participant has a unique GUID and announces itself via SPDP |
| **SPDP** (Simple Participant Discovery Protocol) | Periodic multicast announcements on a well-known port so participants find each other |
| **SEDP** (Simple Endpoint Discovery Protocol) | After participants discover each other, they exchange information about their publishers/subscribers/services |
| **DataWriter / DataReader** | The actual pub/sub endpoints that exchange topic data |
| **CDR Serialization** | Common Data Representation — the standard wire format for DDS messages |
| **QoS** (Quality of Service) | Reliability, durability, history depth, deadline, liveliness — these must be compatible between endpoints for communication to occur |
| **Domain ID** | A logical partition of the DDS network. Nodes on different domains don't communicate. Maps to specific UDP port ranges |

### 2.2 Raft Framework Architecture (What We're Building On)

Raft is an opinionated embedded framework for ESP32. Its architecture maps well to ROS concepts:

```
┌───────────────────────────────────────────────────────┐
│  Raft Application                                     │
├───────────────────────────────────────────────────────┤
│  SysManager (orchestrator)                            │
│   ├── NetworkManager (WiFi/Ethernet)                  │
│   ├── BLEManager                                      │
│   ├── DeviceManager                                   │
│   │    ├── RaftI2C Bus Task (priority scanning)       │
│   │    ├── DeviceTypeRecords (self-describing data)   │
│   │    └── Device instances (auto-detected)           │
│   ├── StatePublisher (hash-based change detection)    │
│   │    └── CommsCoreIF (transport-agnostic channels)  │
│   ├── MQTTManager (MQTT pub/sub — existing pattern)   │
│   ├── CommandSocket (HTTP/WebSocket)                  │
│   ├── CommandSerial (UART framed protocol)            │
│   └── [RaftROS SysMod — NEW]                          │
├───────────────────────────────────────────────────────┤
│  REST API / Subscriptions (HTTP, BLE, Serial, WS)     │
└───────────────────────────────────────────────────────┘
```

**Raft features directly relevant to ROS integration:**

| Raft Feature | ROS 2 Parallel | Mapping Potential |
|-------------|----------------|-------------------|
| SysMod | ROS Node | RaftROS itself is a SysMod; it can also represent each device as a virtual sub-node |
| DeviceManager auto-detection | Node discovery | Detected devices → auto-created publishers |
| DeviceTypeRecords (`resp` format) | Message type definitions | Binary format descriptions → CDR-serialized ROS messages |
| Device class tags (`clas`) | Topic namespacing | `ACC` → `sensor_msgs/Imu`, `DIST` → `sensor_msgs/Range`, etc. |
| Device actions | ROS Services | Action definitions → service endpoints |
| StatePublisher + CommsCoreIF | DDS DataWriter/DataReader | Callback-based pub/sub with hash change detection and rate limiting |
| MQTTManager pattern | — | Proven template for adding a network pub/sub protocol as a SysMod (see §2.4) |
| REST API subscriptions | Topic subscriptions | Existing subscription model maps to DDS DataReader concept |

### 2.3 Lessons from esp-dds

The [esp-dds](https://github.com/KristijanPruzinac/esp-dds) project (MIT license, by Kristijan Pruzinac) implements a lightweight DDS-inspired middleware for ESP32. While it is **not** a network DDS implementation (it provides only intra-process, FreeRTOS-based pub/sub), several design ideas are worth noting:

**Applicable ideas:**
- Static allocation strategy with compile-time bounds (`DDS_MAX_TOPICS`, `DDS_MAX_SUBSCRIBERS_PER_TOPIC`) — good for embedded predictability
- ROS 2 naming conventions (topics prefixed with `/`) adopted from the start
- Separation of sync and async service patterns
- FreeRTOS task notification for efficient subscriber wakeup
- Per-task message queues for isolation between subscribers
- Macro wrappers for common operations to simplify API

**Limitations to avoid:**
- No network transport at all — purely in-process
- No discovery protocol — topics are implicitly created
- No serialization framework — raw `void*` + `memcpy` with 64-byte limit
- No QoS — no delivery guarantees
- Static arrays only with very small limits (8 topics, 4 subscribers/topic)
- No type system or message schema

**Key takeaway:** esp-dds validates that a lightweight pub/sub abstraction with ROS naming conventions is implementable on ESP32, but RaftROS needs actual network interoperability, which requires implementing (a subset of) the RTPS wire protocol.

### 2.4 StatePublisher and MQTTManager — The Existing Pub/Sub Infrastructure

Raft already has a mature, transport-agnostic pub/sub system. Understanding it in detail is critical because **RaftROS should integrate as another transport channel** alongside HTTP, BLE, Serial, and MQTT — not replace the existing infrastructure.

#### 2.4.1 StatePublisher (the Pub/Sub Core)

StatePublisher is a SysMod that manages data sources and subscriptions:

**Data source registration** — other SysMods (primarily DeviceManager) register named data sources:
```cpp
uint16_t registerDataSource(
    const char* pubTopic,                    // e.g., "devjson", "devbin"
    SysMod_publishMsgGenFn msgGenCB,         // Callback to generate message data
    SysMod_stateDetectCB stateDetectCB       // Callback to detect state changes via hash
) → uint16_t topicIndex
```

**Subscription model** — subscribers specify a topic, rate, and trigger mode:
```cpp
bool createSubscription(
    const String& pubTopic,      // Which data source
    uint32_t channelID,          // Which transport channel to publish on
    double rateHz,               // Maximum publish rate
    TriggerType_t trigger,       // TRIGGER_ON_TIME_INTERVALS / TRIGGER_ON_STATE_CHANGE /
                                 // TRIGGER_ON_TIME_OR_CHANGE
    uint32_t minTimeBetweenMsgsMs // Minimum gap between publishes
)
```

**Publishing loop** — in each `loop()` call, StatePublisher:
1. Checks each subscription's timer and trigger conditions
2. Calls `stateDetectCB()` → computes a hash of the current device state
3. Compares hash with the previous to detect changes
4. If publishing is warranted: calls `msgGenCB()` to fill a `CommsChannelMsg` buffer
5. Routes the message via `CommsCoreIF::outboundHandleMsg()` to the target channel
6. Handles backoff on repeated failures (exponential: 2×, 10×, 20× interval)

**Key insight for RaftROS:** The `msgGenCB` callback produces the serialized payload. For existing channels this is JSON (RICJSON protocol). For RaftROS, the callback would produce CDR-encoded data instead. This means RaftROS either:
- Registers its own data sources with CDR-generating callbacks, or
- Intercepts the raw binary device data directly from DeviceManager and encodes it independently

#### 2.4.2 CommsCoreIF — Transport Channel Abstraction

All Raft transports register as channels via `CommsCoreIF`:

```cpp
uint32_t registerChannel(
    const char* protocolName,                // e.g., "RICJSON", "RTPS"
    const char* interfaceName,               // e.g., "MQTT", "HTTP", "RTPS"
    const char* channelName,                 // Specific channel identifier
    CommsChannelOutboundHandleMsgFnType cb,  // Send callback
    CommsChannelOutboundCanAcceptFnType cb,  // Ready-to-send callback
    const CommsChannelSettings* settings
) → uint32_t channelID
```

The `channelID` is then used in subscriptions to route data to the correct output. Each channel has its own outbound queue, and only the latest `MSG_TYPE_PUBLISH` message is kept (snapshot semantics — ideal for sensor data where only the latest state matters).

#### 2.4.3 MQTTManager — A Direct Template for RaftROS

MQTTManager implements a network pub/sub protocol (MQTT) as a Raft SysMod. Its integration pattern is **exactly what RaftROS should follow**:

```
┌─ setup():
│   Parse config (broker, topics, QoS)
│   Initialize protocol client
│
├─ addCommsChannels(CommsCoreIF&):
│   For each outbound topic:
│     Register a CommsChannel with send callback
│     Store channelID → topic mapping
│
├─ postSetup():
│   Get StatePublisher via getSysManager()->getSysMod("Publish")
│   For each configured pubSource:
│     Call createSubscription(pubTopic, channelID, rateHz, trigger)
│   This wires: DeviceManager data → StatePublisher → MQTT channel
│
└─ loop():
    Service protocol connection (connect, keepalive, retry)
    Handle inbound messages
```

**MQTTManager configuration example** (showing the pattern RaftROS would mirror):
```json
{
  "MQTTMan": {
    "enable": true,
    "brokerHostname": "broker.example.com",
    "brokerPort": 1883,
    "clientID": "raft-device",
    "topics": [
      {
        "name": "telemetry",
        "path": "raft/device/devjson",
        "qos": 1,
        "inbound": false,
        "pubSources": [
          { "pubTopic": "devjson", "rateHz": 1.0, "trigger": "timeorchange" }
        ]
      }
    ]
  }
}
```

**MQTT wire protocol implementation** — MQTTManager includes its own custom MQTT 3.1.1 protocol encoder/decoder (`MQTTProtocol` class in RaftCore), with a state machine for connection management (DISCONNECTED → SOCK_CONN_REQD → MQTT_CONN_SENT → MQTT_CONNECTED). This is a proven precedent for implementing a custom wire protocol within Raft — RaftROS would follow the same pattern with RTPS.

#### 2.4.4 Key Architectural Insight

The existing architecture means RaftROS does **not** need to reimplement data source management, change detection, rate limiting, or backoff logic. These are already handled by StatePublisher. RaftROS's primary responsibilities are:

1. **Wire protocol:** Implement RTPS message encoding/decoding and CDR serialization
2. **Discovery:** Implement SPDP/SEDP for network participation
3. **Channel integration:** Register as a CommsChannel and wire up StatePublisher subscriptions
4. **Data transformation:** Convert between Raft's binary device data format and CDR-encoded ROS messages

---

## 3. Technical Approach

### 3.1 Implementation Strategy Options

There are several possible approaches to achieving native ROS 2 participation from an ESP32:

#### Option A: Minimal RTPS Implementation (Recommended)

Implement the minimum subset of the DDSI-RTPS specification needed to:
1. Announce as a DDS participant (SPDP)
2. Advertise publishers/subscribers (SEDP)
3. Exchange data using CDR serialization over UDP

**Pros:** Full native participation, no agent needed, maximum compatibility
**Cons:** Most complex to implement; RTPS is a substantial protocol

**Scope can be limited by:**
- Supporting only Best-Effort reliability for user topics (reliable delivery is already implemented for discovery)
- Supporting only a fixed, small set of ROS message types
- Using compile-time topic configuration with runtime device-driven additions
- Targeting a specific ROS 2 DDS implementation for initial interop testing (e.g., Fast DDS or Cyclone DDS)

#### Option B: Zenoh Native Client

ROS 2 Kilted Kaiju (2025) added Zenoh as a first-class RMW alternative. Zenoh is designed for resource-constrained and IoT scenarios with lower overhead than full RTPS.

**Pros:** Simpler protocol, designed for edge/IoT, lower resource usage, official ROS 2 support
**Cons:** Requires Zenoh router for bridging to DDS-based ROS nodes; newer ecosystem with less deployment history

#### Option C: Direct UDP Protocol with ROS Bridge

Implement a custom lightweight protocol and provide a ROS 2 bridge node (running on a host) that translates.

**Pros:** Simplest ESP32-side implementation
**Cons:** Reintroduces the agent/bridge dependency that motivated this project; similar to micro-ROS

#### Chosen Path: Option A (Minimal RTPS)

**Option A (Minimal RTPS)** is the chosen approach. Option B (Zenoh) may be considered as a future addition given Zenoh's trajectory in the ROS ecosystem, but RTPS provides maximum compatibility with existing ROS 2 deployments. Option C defeats the purpose by reintroducing an agent dependency.

The MQTTManager precedent in RaftSysMods demonstrates that implementing a custom wire protocol within Raft's SysMod architecture is feasible and well-supported — MQTT's connection state machine, keepalive logic, and channel integration provide a direct template for the RTPS implementation.

### 3.2 Minimal RTPS Subset for RaftROS

The full RTPS specification (OMG DDSI-RTPS v2.5) is large, but a workable subset for an embedded publisher/subscriber is much smaller:

#### 3.2.1 Discovery (SPDP — Simple Participant Discovery Protocol)

```
                ESP32 (RaftROS)                    ROS 2 Host
                ┌──────────┐                    ┌──────────────┐
                │          │── SPDP announce ──▶│              │
                │          │   (multicast)       │              │
                │ Partici- │◀── SPDP announce ──│   Fast DDS   │
                │   pant   │   (multicast)       │   Partici-   │
                │          │                     │    pant      │
                │          │── SEDP endpoints ──▶│              │
                │          │◀── SEDP endpoints ──│              │
                └──────────┘                    └──────────────┘
```

**Required SPDP functionality:**
- Generate a unique participant GUID (based on ESP32 MAC address + process ID)
- Send periodic `SPDPdiscoveredParticipantData` messages to the well-known multicast group (`239.255.0.1`, port derived from domain ID)
- Receive and parse SPDP announcements from other participants
- Maintain a participant table with lease duration tracking

**Required SEDP functionality:**
- Announce DataWriter endpoints (publication messages) with topic name, type name, QoS, unicast locator, and participant GUID
- Announce DataReader endpoints (subscription messages) with matching parameters
- Send Participant Message Data (liveliness assertions) periodically
- Parse remote endpoint announcements to match publishers to subscribers
- Maintain matched endpoint state
- Handle reliable delivery for all SEDP exchanges (HEARTBEAT + ACKNACK + retransmit)

#### 3.2.2 Data Exchange

- Serialize outgoing messages in CDR (Common Data Representation) format
- Send data over UDP unicast to matched subscribers
- Receive data over UDP from matched publishers
- Handle the RTPS message header and submessage structure (DATA, HEARTBEAT, ACKNACK, INFO_DST, INFO_TS) — reliable delivery is already implemented

#### 3.2.3 CDR Serialization

CDR is relatively straightforward — it's a binary format with:
- Little-endian or big-endian encoding (flagged per message)
- Natural alignment for primitive types
- Length-prefixed strings and sequences
- No compression

For Raft's use case, we can generate CDR encoders from DeviceTypeRecords at build time or use the `resp` attribute descriptions at runtime.

### 3.3 RaftROS as a Raft SysMod

RaftROS would be implemented as a SysMod, following the pattern established by MQTTManager:

```cpp
class RaftROS : public RaftSysMod {
public:
    // Standard SysMod lifecycle (mirrors MQTTManager)
    void setup() override;               // Parse config, init RTPS participant, join multicast
    void addCommsChannels(CommsCoreIF& commsCoreIF) override;  // Register RTPS as transport channel
    void postSetup() override;           // Wire StatePublisher subscriptions to RTPS channels
    void loop() override;                // Service RTPS: discovery keepalive, send/receive, connection FSM
    
    // Introspection
    void addRestAPIEndpoints(RestAPIEndpointManager& mgr) override;
    String getStatusJSON() const override;
    
private:
    // RTPS participant and discovery
    RTPSParticipant _participant;         // GUID, lease duration, domain ID
    SPDPHandler _spdp;                    // Participant discovery (multicast)
    SEDPHandler _sedp;                    // Endpoint discovery (matched readers/writers)
    
    // Channel integration (following MQTTManager pattern)
    std::map<String, uint32_t> _topicChannelIDs;   // ROS topic name → CommsCoreIF channel ID
    
    // Device-to-topic mapping
    DeviceTopicMapper _topicMapper;       // Maps detected devices to ROS message types
    
    // CDR serialization
    CDREncoder _encoder;                  // Raft binary → CDR encoding
    
    // Connection state machine (like MQTTManager: DISCONNECTED → JOINING → ANNOUNCING → ACTIVE)
    enum class ConnState { DISCONNECTED, MULTICAST_JOIN, SPDP_ANNOUNCING, ACTIVE };
    ConnState _connState = ConnState::DISCONNECTED;
    
    // Outbound message handler (registered as CommsChannel callback)
    bool sendRTPSMsg(const String& topicName, CommsChannelMsg& msg);
    bool readyToSend(uint32_t channelID, CommsMsgTypeCode msgType, bool& noConn);
};
```

**Integration flow** (following MQTTManager precedent):
```
setup():           Parse JSON config (domainId, topics, pubSources, QoS)
                   Initialize RTPS participant with GUID from ESP32 MAC
                   
addCommsChannels(): For each configured ROS topic:
                     Register a CommsChannel with sendRTPSMsg callback
                     Store channelID in _topicChannelIDs
                     
postSetup():       Get StatePublisher via getSysManager()->getSysMod("Publish")
                   For each pubSource in config:
                     createSubscription(pubTopic, channelID, rateHz, trigger)
                   This wires: device data → StatePublisher → RTPS channel
                   
loop():            Service RTPS connection state machine:
                     DISCONNECTED → join multicast group on well-known port
                     MULTICAST_JOIN → start sending SPDP announcements
                     SPDP_ANNOUNCING → process SEDP endpoint matching
                     ACTIVE → handle data exchange, periodic SPDP keepalive
                   Receive and process inbound RTPS messages
```

**Example configuration** (mirrors MQTTManager style):
```json
{
  "RaftROS": {
    "enable": true,
    "domainId": 0,
    "nodeName": "raft_esp32",
    "topics": [
      {
        "name": "imu",
        "path": "/raft_esp32/imu",
        "msgType": "sensor_msgs/msg/Imu",
        "qos": "best_effort",
        "inbound": false,
        "pubSources": [
          { "pubTopic": "devjson", "rateHz": 50.0, "trigger": "timeorchange" }
        ]
      }
    ]
  }
}
```

### 3.4 Device-to-ROS Mapping Engine

The most innovative aspect of RaftROS is automatic mapping of Raft's dynamically-detected devices to ROS 2 topics and services.

#### 3.4.1 Device Class to ROS Message Type Mapping

DeviceTypeRecords include `clas` tags that categorize devices. These can map to standard ROS message types:

| Raft `clas` Tag(s) | ROS 2 Message Type | Notes |
|---------------------|-------------------|-------|
| `ACC` | `sensor_msgs/msg/Imu` | Acceleration fields populated |
| `GYRO` | `sensor_msgs/msg/Imu` | Angular velocity fields populated |
| `ACC` + `GYRO` | `sensor_msgs/msg/Imu` | Full IMU message |
| `DIST` | `sensor_msgs/msg/Range` | ToF / ultrasonic distance |
| `TEMP` | `sensor_msgs/msg/Temperature` | Temperature reading |
| `RH` | `sensor_msgs/msg/RelativeHumidity` | Humidity |
| `PRES` | `sensor_msgs/msg/FluidPressure` | Barometric pressure |
| `LGHT` | `sensor_msgs/msg/Illuminance` | Ambient light level |
| `BTN` | `std_msgs/msg/Bool` | Button press state |
| `SRVO` | `std_msgs/msg/Float32` | Servo position (via service) |
| `PIX` | Custom or `std_msgs/msg/ColorRGBA` | LED control |
| Generic / unknown | `std_msgs/msg/ByteMultiArray` | Raw binary data with metadata |

#### 3.4.2 Topic Naming Convention

Auto-generated topics would follow a consistent naming scheme:

```
/<node_name>/<bus>/<device_type>/<attribute_group>
```

Examples:
```
/raft_esp32_abc123/i2c/LSM6DS/imu          → sensor_msgs/msg/Imu
/raft_esp32_abc123/i2c/BMP280/temperature   → sensor_msgs/msg/Temperature
/raft_esp32_abc123/i2c/BMP280/pressure      → sensor_msgs/msg/FluidPressure
/raft_esp32_abc123/ble/BTHome_TH/temperature → sensor_msgs/msg/Temperature
```

#### 3.4.3 Service Generation from Device Actions

DeviceTypeRecord `actions` become ROS 2 services:

```
/raft_esp32_abc123/i2c/PCA9685/set_servo    → custom_interfaces/srv/SetServo
/raft_esp32_abc123/i2c/NeoPixel/set_color   → custom_interfaces/srv/SetColor
```

For maximum compatibility without custom message packages, generic service types could be used:

```
/raft_esp32_abc123/device_command  → rcl_interfaces/srv/SetParameters
```

#### 3.4.4 Dynamic Topic Lifecycle

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────────┐
│  I2C Bus Scan   │────▶│  Device Detected  │────▶│  Topic Created      │
│  (RaftI2C)      │     │  (DeviceManager)  │     │  (SEDP published)   │
└─────────────────┘     └──────────────────┘     └─────────────────────┘
                                                          │
                              ┌────────────────────────────┘
                              ▼
                        ┌─────────────────────┐
                        │  Data Published     │
                        │  (CDR over RTPS)    │
                        └─────────────────────┘
                              │
┌─────────────────┐           ▼
│  Device Removed │     ┌─────────────────────┐
│  (disconnect)   │────▶│  Topic Removed      │
└─────────────────┘     │  (SEDP disposed)    │
                        └─────────────────────┘
```

When a device is hot-plugged:
1. DeviceManager detects it via priority-based scanning
2. DeviceTypeRecord is matched — provides class tags, response format, actions
3. RaftROS creates a DataWriter and announces it via SEDP
4. Polling data is converted to the appropriate ROS message type and published via RTPS
5. If the device is removed, the DataWriter is disposed and SEDP updated

---

## 4. Data Flow Architecture

### 4.1 Publishing Path (ESP32 → ROS Network)

This path leverages the existing StatePublisher infrastructure, adding RTPS as another transport channel alongside HTTP, BLE, Serial, and MQTT:

```
  I2C/BLE Device
       │
       ▼
  DeviceManager (polls device, binary data in DeviceTypeRecords format)
       │  registers data source: "devjson" / "devbin"
       ▼
  StatePublisher (existing Raft infrastructure)
       │  ├── Subscription timer fires (rateHz) or state hash changes
       │  ├── Calls msgGenCB → fills CommsChannelMsg buffer
       │  └── Routes via CommsCoreIF::outboundHandleMsg(channelID)
       │
       ▼                              ┌──────────────────────┐
  CommsChannelManager                 │  Other channels      │
       │  routes to RTPS channel      │  (HTTP, MQTT, BLE…)  │
       ▼                              └──────────────────────┘
  RaftROS.sendRTPSMsg() callback
       │  ├── DeviceTopicMapper: lookup device class → ROS message type
       │  ├── Decode binary using resp.a attribute definitions
       │  ├── Apply unit conversions (divisor, offset, etc.)
       │  └── CDR Encoder: serialize to CDR format
       │
       ▼
  RTPS DataWriter (add RTPS header, submessage framing)
       │
       ▼
  UDP Socket → ROS 2 network
```

**Note:** The same device data can be simultaneously published via RTPS (to ROS), MQTT (to a broker), HTTP (to a web dashboard), and BLE (to a mobile app) — each as a separate StatePublisher subscription on different channels, with independent rates and triggers.

### 4.2 Subscription Path (ROS Network → ESP32)

```
  UDP Socket ← ROS 2 network
       │
       ▼
  RTPS Message Parser (validate header, extract DATA submessages)
       │
       ▼
  CDR Decoder (deserialize from CDR)
       │
       ▼
  RaftROS topic handler
       │  ├── Route to device action (e.g., servo command)
       │  └── Or: forward to Raft SysMod via SysManager
       │
       ▼
  DeviceManager (write to device via I2C/BLE)
```

### 4.3 Self-Description and Introspection

RaftROS could expose an enhanced self-description capability beyond standard ROS:

- **Node graph info:** Respond to standard `ros2 node info /raft_esp32_xyz` queries
- **Parameter server:** Expose device configuration as ROS 2 parameters
- **Device metadata topic:** Publish a custom topic with complete DeviceTypeRecord information (device name, manufacturer, class, all attributes, available actions) so that ROS-side tools can auto-configure visualization or control interfaces
- **Diagnostic messages:** Publish `diagnostic_msgs/msg/DiagnosticArray` with bus health, scan statistics, device error counts

---

## 5. Implementation Phases

### Phase 1: RTPS Foundation — DDS Participant Discovery

**Goal:** ESP32 appears as a DDS participant visible to `ros2 node list` ✅ COMPLETE

**Deliverable:** Running `ros2 node list` on a host PC (same WiFi network, domain 0) shows `/raft_esp32`.

**Example project:** `examples/ExampleDiscoverable` — a minimal Raft application that includes the RaftROS SysMod and demonstrates DDS participant discovery (see [§5.2](#52-examplediscoverable--the-phase-1-test-application) for full details).

#### 5.1.1 What `ros2 node list` Requires (Protocol-Level)

When a user runs `ros2 node list`, the ROS 2 CLI tool (or the ROS 2 daemon) acts as a DDS participant and discovers other participants via the SPDP multicast protocol. For the ESP32 to be visible, it must:

1. **Join the SPDP multicast group** on the correct port
2. **Send a valid `SPDPdiscoveredParticipantData` message** periodically
3. **Include ROS 2 node name metadata** in the participant's `userData` QoS parameter
4. **Respond to SEDP** built-in endpoint exchange (so the host can query topics)

The discovery flow looks like this:

```
ESP32 (RaftROS)                                     ROS 2 Host (ros2 node list)
     │                                                      │
     │──── SPDP multicast ─────────────────────────────────▶│
     │     (SPDPdiscoveredParticipantData)                   │
     │     Contains: GUID, locators, lease,                  │
     │     userData="enclave=/;"                              │
     │                                                      │
     │◀──── SPDP multicast ────────────────────────────────│
     │      (Host's own SPDP announcement)                  │
     │                                                      │
     │──── SEDP pub unicast (metatraffic) ─────────────────▶│
     │     "I have DataWriter for ros_discovery_info"        │
     │──── SEDP sub unicast (metatraffic) ─────────────────▶│
     │     "I have DataReader for ros_discovery_info"        │
     │──── Liveliness (metatraffic) ───────────────────────▶│
     │     (Participant message data)                        │
     │                                                      │
     │◀──── SEDP + ACKNACK exchange ───────────────────────│
     │      (Endpoint matching handshake)                   │
     │                                                      │
     │──── ros_discovery_info DATA (user data port) ───────▶│
     │     (CDR: participant GID, node name, namespace)      │
     │                                                      │
     │                ros2 node list shows:                  │
     │                   /raft_esp32                         │
```

#### 5.1.2 SPDP Multicast Details

**Multicast group:** `239.255.0.1`

**Port calculation** (DDSI-RTPS §9.6.1): The port numbers depend on the domain ID and participant index:

| Port | Formula | Domain 0 Value |
|------|---------|----------------|
| SPDP multicast (discovery) | PB + DG × domainId + d0 | 7400 |
| SPDP unicast (discovery) | PB + DG × domainId + d1 + PG × participantId | 7410+ |
| User multicast (data) | PB + DG × domainId + d2 | 7401 |
| User unicast (data) | PB + DG × domainId + d3 + PG × participantId | 7411+ |

Where: PB=7400, DG=250, PG=2, d0=0, d1=10, d2=1, d3=11

**SPDP announcement interval:** Typically every 30 seconds (configurable). Lease duration is typically 3× the announcement interval.

#### 5.1.3 SPDPdiscoveredParticipantData Message Format

The SPDP announcement is an RTPS message containing a DATA submessage. The payload is a CDR-serialized `ParameterList` — a sequence of (parameterId, length, value) tuples terminated by a sentinel:

```
RTPS Header (20 bytes)
  ├── "RTPS" magic (4 bytes)
  ├── Version 2.2 (2 bytes)
  ├── Vendor ID (2 bytes)
  └── GUID Prefix (12 bytes)

INFO_TS Submessage (optional, timestamp)

DATA Submessage
  └── Serialized Payload (CDR-encoded ParameterList):
        ├── PID_PROTOCOL_VERSION    (0x0015) → {2, 2}
        ├── PID_VENDORID            (0x0016) → {0x01, 0x03} (or custom)
        ├── PID_PARTICIPANT_GUID    (0x0050) → 16-byte GUID
        ├── PID_BUILTIN_ENDPOINT_SET(0x0058) → bitmap of built-in endpoints
        ├── PID_DEFAULT_UNICAST_LOCATOR    (0x0031) → {kind, port, address}
        ├── PID_METATRAFFIC_UNICAST_LOCATOR(0x0032) → {kind, port, address}
        ├── PID_METATRAFFIC_MULTICAST_LOCATOR(0x0049) → {kind, port, address}
        ├── PID_PARTICIPANT_LEASE_DURATION (0x0002) → {seconds, fraction}
        ├── PID_USER_DATA           (0x002c) → ROS 2 node name encoding
        └── PID_SENTINEL            (0x0001) → end marker
```

**ROS 2 enclave encoding in `PID_USER_DATA`:**

FastRTPS requires `PID_USER_DATA` to contain the DDS security enclave path. For the default enclave:
```
enclave=/;
```

Without this field, FastRTPS **silently ignores** the participant entirely — it will not appear in `ros2 node list`. The actual node name and namespace are **not** in the SPDP `userData` — they are communicated via the `ros_discovery_info` topic (a CDR-encoded `ParticipantEntitiesInfo` message containing participant GID, node namespace, node name, and reader/writer GID lists).

#### 5.1.4 Built-in Endpoint Set (SEDP)

The `PID_BUILTIN_ENDPOINT_SET` bitmap declares which built-in SEDP endpoints this participant supports. The required set (validated by implementation) is:

| Bit | Endpoint | Required? |
|-----|----------|-----------|
| 0 | DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR | Yes |
| 1 | DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER | Yes |
| 2 | DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR | Yes |
| 3 | DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER | Yes |
| 4 | DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR | Yes |
| 5 | DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER | Yes |
| 10 | BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_WRITER | Yes |
| 11 | BUILTIN_ENDPOINT_PARTICIPANT_MESSAGE_DATA_READER | Yes |

**Required bitmap value: `0x0C3F`** (bits 0-5 + bits 10-11). The PARTICIPANT_MESSAGE_DATA endpoints (bits 10-11) are required for liveliness assertion — without them, FastRTPS does not complete endpoint matching.

#### 5.1.5 RTPS Locator Format

Locators (used in `PID_DEFAULT_UNICAST_LOCATOR`, etc.) are 24-byte structures:

```
struct Locator_t {
    int32_t kind;       // LOCATOR_KIND_UDPv4 = 1
    uint32_t port;      // UDP port number
    uint8_t address[16]; // IPv4 mapped: [0..11]=0, [12..15]=IPv4 octets
};
```

For IPv4, the address field is 12 bytes of zero followed by the 4 IPv4 octets.

#### 5.1.6 GUID Generation

The participant GUID is 16 bytes: 12-byte GUID prefix + 4-byte entity ID.

For the participant itself, the entity ID is `{0x00, 0x00, 0x01, 0xC1}` (`ENTITYID_PARTICIPANT`).

The GUID prefix should be unique. Strategy for ESP32:
- Bytes 0–1: Vendor ID (e.g., `{0x01, 0x03}` for eProsima or `{0x00, 0x00}` for unregistered)
- Bytes 2–7: ESP32 MAC address (6 bytes, guaranteed unique per chip)
- Bytes 8–11: Process/instance discriminator (e.g., monotonic counter or timestamp-based)

#### 5.1.7 Implementation Tasks — Phase 1 Checklist

| # | Task | Component | Status |
|---|------|-----------|--------|
| 1 | RTPS message header write/parse | `RTPS/RTPSMessage` | ✅ Done |
| 2 | CDR encoder (all primitives, strings) | `CDR/CDREncoder` | ✅ Done |
| 3 | CDR decoder (all primitives, strings) | `CDR/CDRDecoder` | ✅ Done |
| 4 | RaftROS SysMod with connection state machine | `RaftROS.h/.cpp` | ✅ Done |
| 5 | RTPS participant GUID generation (MAC-based) | `RTPS/RTPSParticipant` | ✅ Done |
| 6 | CDR ParameterList encoder (PID tuples + sentinel) | `RTPS/SPDPHandler` | ✅ Done |
| 7 | SPDPdiscoveredParticipantData builder | `RTPS/SPDPHandler` | ✅ Done |
| 8 | SPDP multicast socket — join + periodic send | `RaftROS` | ✅ Done |
| 9 | SPDP receive + parse (multicast and metatraffic port) | `RaftROS` | ✅ Done |
| 10 | Participant table with lease tracking + purging | `RaftROS` | ✅ Done |
| 11 | SEDP publication endpoint announcement | `RTPS/SEDPHandler` | ✅ Done |
| 12 | SEDP subscription endpoint announcement | `RTPS/SEDPHandler` | ✅ Done |
| 13 | Participant message data (liveliness) | `RTPS/SEDPHandler` | ✅ Done |
| 14 | ros_discovery_info CDR payload builder | `RTPS/SPDPHandler` | ✅ Done |
| 15 | ros_discovery_info DataWriter (user data port) | `RTPS/SEDPHandler` | ✅ Done |
| 16 | DATA submessage write (for SPDP/SEDP/user data) | `RTPS/RTPSMessage` | ✅ Done |
| 17 | INFO_DST + INFO_TS submessages | `RTPS/RTPSMessage` | ✅ Done |
| 18 | HEARTBEAT submessage (sent with each DATA) | `RTPS/RTPSMessage` | ✅ Done |
| 19 | ACKNACK handling — respond to remote HBs | `RaftROS` | ✅ Done |
| 20 | ACKNACK handling — retransmit on remote NACKs | `RaftROS` | ✅ Done |
| 21 | Periodic writer heartbeats (all 4 writers) | `RaftROS` | ✅ Done |
| 22 | WiFi-ready gating (only join multicast after connected) | `RaftROS` | ✅ Done |
| 23 | Integration test: `ros2 node list` shows `/raft_esp32` | ESP32 + ROS 2 host | ✅ Done |
| 24 | Linux standalone test node (`raftros_standalone.cpp`) | `linux_unit_tests/` | ✅ Done |

#### 5.1.8 Phase 1 Key Learnings

The following lessons were learned during Phase 1 implementation. These are critical for anyone working on or extending the RTPS code:

**Entity ID encoding (NO_KEY vs WITH_KEY):**
The `ros_discovery_info` topic uses `WRITER_NO_KEY` (entity kind byte `0x03`) and `READER_NO_KEY` (`0x04`). Using `WITH_KEY` variants (`0x02` / `0x07`) causes FastDDS XTypes type matching to fail silently — the endpoint is discovered but never matched. This was the single most important fix.

**SEDP requires both publication AND subscription announcements:**
It is not sufficient to only announce our DataWriter (SEDP publication). We must also announce our DataReader (SEDP subscription) for `ros_discovery_info`, plus send Participant Message Data (liveliness assertions). Without all three, FastDDS does not complete the endpoint matching handshake.

**PID_UNICAST_LOCATOR and PID_PARTICIPANT_GUID in SEDP:**
SEDP publication and subscription messages must include `PID_UNICAST_LOCATOR` (our IP address) and `PID_PARTICIPANT_GUID` (linking the endpoint back to the participant). Without `PID_UNICAST_LOCATOR`, the remote peer doesn't know where to send user data.

**ros_discovery_info sequence number must not increment:**
The `ros_discovery_info` payload is static (same content every time). Its sequence number must stay at 1. Incrementing it causes the remote to ACKNACK requesting all the "missing" intermediate sequence numbers, creating an ever-growing gap.

**PID_USER_DATA must contain `"enclave=/;"`:**
FastRTPS silently ignores any SPDP participant that lacks `PID_USER_DATA` with the enclave string. There is no error, no log — the participant simply doesn't appear.

**Reliable writer protocol is required from the start:**
Even for discovery-only (Phase 1), reliable delivery (HEARTBEAT + ACKNACK handling + retransmit) is required for SEDP and `ros_discovery_info`. Best-effort is not sufficient because the remote may not have created its reader proxy when our first DATA arrives.

**lwIP buffer exhaustion on ESP32:**
Sending too many UDP packets in a burst causes `errno 12` (ENOMEM) on ESP32 due to lwIP buffer limits. The workaround is to defer some messages to the periodic heartbeat cycle rather than sending everything in `handleNewParticipant()`.

**Buffer sizes:**
Send buffer needs ≥1024 bytes and receive buffer ≥2048 bytes. SPDP announcements from FastDDS can be ~400 bytes due to multiple locators (including SHM locators that we ignore).

**Stale participant purging is essential:**
Without purging, the discovered participant list fills up with dead entries (e.g., from daemon restarts). New participants can't be added once the list is full. Purge entries older than 2× lease duration.

**SPDP arrives on metatraffic port too:**
The daemon sends unicast SPDP replies to our metatraffic port (7410), not just to the multicast port (7400). The metatraffic receive handler must detect and parse these as SPDP announcements.

#### 5.1.8 Phase 1 Testing Strategy

**Linux unit tests** (no hardware required):
- ParameterList encoder round-trip (encode → decode, verify PIDs and values)
- SPDP message builder — verify byte-level output against Wireshark captures from a known ROS 2 node
- GUID generation determinism

**On-device testing with ExampleDiscoverable:**
1. Flash ExampleDiscoverable to ESP32, connect to same WiFi as ROS 2 host
2. Run `ros2 node list` — verify `/raft_esp32` appears
3. Run `ros2 topic list` — verify `/raft_esp32/heartbeat` appears
4. Use Wireshark with RTPS dissector to capture and validate SPDP/SEDP packets
5. Test lease duration — verify the node disappears after the ESP32 is powered off and the lease expires
6. Test reconnection — power-cycle the ESP32, verify it re-appears

**Interoperability matrix:**

| ROS 2 Host DDS | Test Priority |
|----------------|---------------|
| Fast DDS (default in Humble/Iron/Jazzy) | Primary |
| Cyclone DDS | Secondary |
| Zenoh (Kilted Kaiju) | Future |

### 5.2 ExampleDiscoverable — The Phase 1 Test Application

The `examples/ExampleDiscoverable/` directory contains a complete Raft application that serves as the primary test vehicle for Phase 1. It is a standard Raft project (created via `raft new`) with the RaftROS SysMod integrated.

#### 5.2.1 Project Structure

```
examples/ExampleDiscoverable/
├── CMakeLists.txt              ← Raft bootstrap (fetches RaftCore, RaftSysMods, etc.)
├── Dockerfile                  ← ESP-IDF v5.5.2 build container
├── compose.yaml                ← Docker compose for CI builds
├── main/
│   ├── CMakeLists.txt          ← Links app to RaftCore, RaftSysMods, RaftWebServer, MainSysMod
│   └── main.cpp                ← App entry point (registers SysMods, runs main loop)
├── components/
│   └── MainSysMod/             ← Minimal application SysMod
│       ├── CMakeLists.txt
│       ├── MainSysMod.cpp
│       └── MainSysMod.h
└── systypes/
    ├── Common/
    │   └── features.cmake      ← Target chip (esp32s3), Raft component versions
    └── SysTypeMain/
        ├── SysTypes.json       ← Full system configuration (NetMan, DevMan, Publish, etc.)
        ├── features.cmake      ← Includes Common/features.cmake
        ├── partitions.csv      ← Flash partition layout
        └── sdkconfig.defaults  ← ESP-IDF SDK configuration
```

#### 5.2.2 Phase 1 Integration Plan

To make ExampleDiscoverable function as a Phase 1 demonstrator, the following changes are needed:

**1. Add RaftROS as a dependency:**

In `systypes/Common/features.cmake`, add RaftROS to the component list:
```cmake
set(RAFT_COMPONENTS
    RaftCore@main
    RaftSysMods@main
    RaftWebServer@main
    RaftROS@main                  # ← NEW
)
```

Or, for local development, use a path dependency by adding to `CMakeLists.txt`:
```cmake
list(APPEND EXTRA_COMPONENT_DIRS "../../components")
```

**2. Register the RaftROS SysMod in `main.cpp`:**

```cpp
#include "RaftROS.h"

// In app_main(), before the main loop:
raftCoreApp.registerSysMod("RaftROS", RaftROS::create, true);
```

**3. Configure RaftROS in `SysTypes.json`:**

Add a RaftROS configuration block alongside the existing SysMod configs:
```json
{
  "RaftROS": {
    "enable": true,
    "domainId": 0,
    "nodeName": "raft_esp32",
    "spdpAnnounceIntervalMs": 30000,
    "leaseDurationS": 100,
    "topics": [
      {
        "name": "heartbeat",
        "path": "/raft_esp32/heartbeat",
        "msgType": "std_msgs/msg/String",
        "qos": "best_effort",
        "inbound": false
      }
    ]
  }
}
```

**4. Ensure WiFi is configured** — the existing `NetMan` config in SysTypes.json already enables WiFi STA mode. The user must configure WiFi credentials (SSID/password) via the serial console or web interface before RaftROS can send multicast.

#### 5.2.3 Expected Phase 1 Test Workflow

```
1. Build and flash:
   $ cd examples/ExampleDiscoverable
   $ raft run

2. Configure WiFi (serial console):
   > w/<SSID>/<password>

3. On the ROS 2 host (same network):
   $ source /opt/ros/humble/setup.bash
   $ ros2 node list --no-daemon
   /raft_esp32                          ← SUCCESS: ESP32 is discoverable

   Note: If using FastDDS on the same host as the ESP32 (e.g., WSL2),
   a UDPv4-only profile XML may be needed to disable SHM transport:
   $ FASTRTPS_DEFAULT_PROFILES_FILE=fastdds_profile.xml ros2 node list --no-daemon

4. Validate with Wireshark (optional):
   - Filter: rtps
   - Verify SPDP announcements from ESP32 IP on port 7400
   - Verify GUID prefix contains ESP32 MAC
   - Verify userData contains "enclave=/;"
```

#### 5.2.4 Debugging DDS Discovery Issues

Common issues when testing Phase 1 and how to diagnose them:

| Symptom | Likely Cause | Diagnostic |
|---------|-------------|------------|
| Node not visible at all | Multicast blocked by WiFi AP or firewall | Wireshark: check if SPDP packets reach the host; try a different router |
| Node appears then disappears | Lease duration too short or SPDP keepalive not sent | Check `spdpAnnounceIntervalMs` vs `leaseDurationS`; increase lease |
| Node visible but no topics | SEDP not working | Wireshark: check for SEDP exchange after SPDP; verify built-in endpoint bitmap |
| `ros2 node list` hangs | Domain ID mismatch | Verify both ESP32 and host use `ROS_DOMAIN_ID=0` |
| Intermittent discovery | WiFi power saving dropping multicast | Disable WiFi power save in sdkconfig: `CONFIG_ESP_WIFI_SLP_DEFAULT_MIN_ACTIVE_TIME=0` |

### Phase 2: Static Publishing — COMPLETE ✅

**Goal:** Publish data on pre-configured topics ✅ verified 2026-04-21 with `/chatter` (`std_msgs/msg/String`).

**Deliverable delivered:** `ros2 topic echo /chatter std_msgs/msg/String --no-daemon` prints a sample per second from the ESP32 indefinitely.

What landed:

- Second DataWriter endpoint (`/chatter`, RELIABLE + VOLATILE) announced via SEDP publications on the same writer as `ros_discovery_info` but at a distinct sequence number (1 → `ros_discovery_info`, 2 → `/chatter`).
- CDR encoder used to serialize `std_msgs/msg/String` payloads.
- DATA + HEARTBEAT sent on the user data port (`7411`) once per second.
- Reliable QoS delivery uses the existing (Phase 1) HEARTBEAT/ACKNACK runtime; the chatter writer additionally advertises `HEARTBEAT firstSN == currentSeq` to honor VOLATILE semantics.
- `ros_discovery_info` writer GID list advertises the chatter writer as part of the participant's node entities.

Key learnings from Phase 2 bring-up (documented in `RaftROS-development-status.md` Phase 2 section):

- VOLATILE writers must advertise `HEARTBEAT firstSN == currentSeq`, otherwise newly-matched subscribers NACK historical samples that no longer exist and drive a retransmit storm.
- Two DataWriter announcements that share a SEDP publications writer entity must use distinct sequence numbers.
- The `EspStyle` ACKNACK flavor must enable chatter retransmit on SEDP publications ACKNACK (`publicationsIncludesChatterAnnouncement=true`) because the initial unicast burst from `handleNewParticipant` regularly loses the 3rd+ packet to LWIP ENOMEM on the ESP32.

### Phase 3: Topic Subscribing — COMPLETE ✅

**Goal:** Subscribe to ROS 2 topics and receive messages with per-topic handler dispatch ✅ verified 2026-04-22 with `/chatter_in` (slot 0) and `/chatter_in2` (slot 1) (`std_msgs/msg/String`).

**Deliverable delivered:** `ros2 topic pub --once /chatter_in2 std_msgs/msg/String "{data: 'hello slot2'}"` routes to the correct per-slot handler on-device.

What landed:

- Shared `RTPSReaderRuntime` (pure decision logic: DATA Accept/Dedup/Drop, HEARTBEAT → ACKNACK base/numBits/bitmap, FINAL-flag dedup, best-effort suppression) + `RTPSReaderRunner` (submessage parsing + state lookup + dispatch callbacks).
- `RTPSMessage::writeAcknackWithBitmap` bitmap-capable ACKNACK wire builder (DDSI-RTPS §9.4.2.7).
- `RTPSSubscriptionRegistry` (8-slot POD with deterministic entity-ID allocator) + `RaftROS::addStringSubscription(topic, type, handler)` public API.
- Per-topic routing via `RTPSSEDPPublicationParser` (extracts `PID_ENDPOINT_GUID` + `PID_TOPIC_NAME` from inbound SEDP publication DATAs) and `RTPSRemotePublicationMap` (16-entry `writerGuid` → slot map).
- CDR deserialization via `RTPSUserDispatch::decodeStdMsgsString`.

Key learnings from Phase 3 bring-up (documented in `RaftROS-development-status.md` Phase 3 section):

- RTPS `readerSNState` bitmap is an inverted-sense bitmap: bit=1 means NOT received / please retransmit (not a positive ACK bitmap). Per §9.4.5.2.
- DATA submessages must honour the Q flag (0x02) before reading the serialized payload; FastDDS routinely sends dispose-style DATAs with `Q=1 D=0 K=1` that contain only an inline-QoS body and no trailing payload.
- N-ary SEDP subscription announcements on a shared sub writer must use distinct sequence numbers (same pattern as Phase-2 Fix 16 for the publication writer side).

### Phase 4: Dynamic Auto-Configuration — COMPLETE ✅

**Goal:** Devices are automatically mapped to ROS topics based on DeviceTypeRecords.

What landed (12 slices 4.1 – 4.12, see `RaftROS-auto-publishing-design.md` §9):

- **DeviceManager hook** — `RaftROS::setup()` registers a status-change
  callback; `ONLINE`/`PENDING_DELETION` transitions drive a 16-slot writer
  registry (`RTPSDynamicWriterRegistry`, deterministic entityId allocation).
- **Lifecycle owner** — `RTPSAutoPubLifecycle` owns topic/type string
  buffers per slot; composite devices consume two slots keyed by
  `{bus, addr, subIndex}`.
- **Class → ROS 2 type mapping** — `RTPSAutoPubClassMap.h` with per-device
  overrides (MCP9808, RoboticalLightSensor), actuator exclusion
  (SRVO/PUMP/PIX), composite rules (`{ACC,GYRO}` → `Imu`,
  `{TEMP,RH}` → `Temperature+RelativeHumidity`,
  `{PRES,TEMP}` → `FluidPressure+Temperature`), single-class rules for
  every tag in `DeviceTypeRecords.json`, and a `std_msgs/String` JSON
  fallback.
- **SEDP auto-announce** — the existing writer-heartbeat pass walks the
  registry and emits one `PublicationBuiltinTopic` DATA(w) per active slot
  per tick.
- **User-data hot path** — on each decoded bus sample the latest record is
  serialised through `RTPSAutoPubCDRSerializer` (REP-103 unit scaling) into
  a per-slot 512-byte buffer and unicast to every discovered peer.
  Composite devices serialise twice (once per kind) from the same decoded
  struct.
- **Timestamps** — ROS 2 `Header.stamp` is taken from the poll record's
  `timeMs` field, not wall-clock.
- **QoS profiles** — four built-ins (`fast_sensor`, `slow_sensor`, `event`,
  `fallback_string`); SysTypes override surface is `qosProfiles.<alias>`
  and `qosProfiles.classDefaults.<CLAS>`. Resolution order:
  alias → class → built-in default.
- **Dispose on offline** — `PENDING_DELETION` emits an SEDP dispose
  (`PID_STATUS_INFO = 0x00000003`, `PID_KEY_HASH` = guid+entityId) on the
  builtin Publications writer so subscribers drop the topic within a
  heartbeat. Composite secondary slots are disposed alongside the primary.

**Deliverable met:** plugging in a new I2C sensor automatically creates a
new ROS 2 topic that `ros2 topic echo` can consume with the correct message
type, SI units, and QoS profile — zero per-device code.

**Verification:** `linux_unit_tests/main.cpp` = **911 passed, 0 failed**;
ESP32-S3 firmware 0x1435c0 bytes, 25% free on the `app` partition.

### Phase 5: Services and Parameters — FUTURE

**Goal:** Expose device actions as ROS 2 services and parameters

- Implement basic ROS 2 service pattern (request/response over RTPS)
- Map device actions to services
- Implement parameter server for device configuration

**Deliverable:** `ros2 service call /raft_esp32/set_servo` controls a servo connected via I2C

### Phase 6: Advanced Features

- Zenoh transport as an alternative to RTPS
- Multi-domain support
- ROS 2 lifecycle node support
- Integration with `tf2` (transform broadcasting for spatial sensors)
- DDS Security (if needed for specific deployments)
- Best-effort QoS option for high-rate sensor topics where dropped samples are acceptable

---

## 6. Resource Considerations

### 6.1 ESP32 Constraints

| Resource | ESP32 Available | Estimated RaftROS Need |
|----------|----------------|----------------------|
| RAM | ~320 KB (no PSRAM) or 4–8 MB (PSRAM) | 30–60 KB for RTPS state + buffers |
| Flash | 4–16 MB | ~50–100 KB code |
| CPU | Dual-core 240 MHz | RTPS runs in existing Raft loop or dedicated task |
| Network | WiFi 802.11 b/g/n | UDP multicast required for discovery |
| Sockets | lwIP, limited concurrent sockets | 3–5 sockets (SPDP multicast, SEDP, data unicast) |

### 6.2 Critical Constraints

- **UDP multicast:** ESP32's lwIP supports IGMP multicast, but the WiFi AP must allow it. Some consumer routers filter multicast. This is a known issue in ROS 2 deployments generally.
- **Timing:** SPDP announcements have lease durations (typically 100–300 s). The ESP32 must send keepalives within the lease period — this fits well within Raft's cooperative loop model.
- **Serialization cost:** CDR serialization is lightweight compared to JSON. The binary→CDR path from DeviceTypeRecords binary data should be very efficient since both are simple binary formats.
- **Topic count:** With static allocation (inspired by esp-dds), a reasonable bound of 16–32 topics and 8–16 matched endpoints should cover most use cases while remaining memory-friendly.

---

## 7. Existing Resources and References

### 7.1 Open-Source RTPS/DDS Implementations

> **Licensing constraint:** RaftROS must be MIT-licensable. Code from projects under Apache 2.0, EPL, LGPL, or other copyleft/attribution-required licenses **cannot be incorporated** into RaftROS source — they may only be studied as references to understand the RTPS wire protocol and DDS behaviour. The RTPS wire protocol itself is defined by the OMG DDSI-RTPS specification and is not subject to any software license — a clean-room implementation from the spec is fully MIT-licensable.

| Project | Language | License | MIT-Compatible? | Use |
|---------|----------|---------|-----------------|-----|
| [esp-dds](https://github.com/KristijanPruzinac/esp-dds) | C | MIT | **Yes** | API patterns and design ideas may be reused or adapted |
| [eProsima Fast DDS](https://github.com/eProsima/Fast-DDS) | C++ | Apache 2.0 | No (patent clause, attribution) | Protocol reference and interop testing only |
| [Eclipse Cyclone DDS](https://github.com/eclipse-cyclonedds/cyclonedds) | C | EPL 2.0 | No (weak copyleft) | Protocol reference only |
| [micro-CDR](https://github.com/eProsima/Micro-CDR) | C | Apache 2.0 | No (patent clause, attribution) | Reference for CDR format; RaftROS must implement its own CDR encoder from the OMG CDR spec |
| [Micro-XRCE-DDS-Client](https://github.com/eProsima/Micro-XRCE-DDS-Client) | C | Apache 2.0 | No | Reference for how micro-ROS serializes (XRCE protocol, not directly relevant) |
| [Zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico) | C | EPL 2.0 / Apache 2.0 | No (either license is restrictive) | Reference only if pursuing Zenoh path |
| [ros2arduino](https://github.com/ROBOTIS-GIT/ros2arduino) | C++ | Apache 2.0 | No | Reference for feasibility; do not reuse code |

**Implementation approach:** All RTPS, CDR, and discovery code in RaftROS will be written from scratch (clean-room) based on the OMG specifications, which are open standards. The CDR encoding format in particular is straightforward (it is essentially aligned little/big-endian binary with length-prefixed strings) and does not warrant pulling in an external library.

### 7.2 Specifications

| Specification | Version | Key Sections |
|--------------|---------|-------------|
| [DDSI-RTPS](https://www.omg.org/spec/DDSI-RTPS/) | 2.5 | §8 (Messages), §9 (Discovery), §10 (Serialization) |
| [DDS](https://www.omg.org/spec/DDS/) | 1.4 | §2 (PIM — Platform Independent Model) |
| [CDR](https://www.omg.org/spec/CDR/) | — | (Part of CORBA/GIOP specification) |
| [ROS 2 Interface Definition](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html) | — | .msg/.srv/.action file formats |

### 7.3 Key ROS 2 Message Types for Initial Support

These standard message types cover the most common Raft device categories:

```
sensor_msgs/msg/Imu                  → ACC, GYRO (6/9-axis IMU)
sensor_msgs/msg/Temperature          → TEMP
sensor_msgs/msg/RelativeHumidity     → RH
sensor_msgs/msg/FluidPressure        → PRES
sensor_msgs/msg/Range                → DIST (ToF, ultrasonic)
sensor_msgs/msg/Illuminance          → LGHT
sensor_msgs/msg/JointState           → SRVO (servo position/velocity)
geometry_msgs/msg/Twist              → Motor velocity commands (subscribe)
std_msgs/msg/Bool                    → BTN
std_msgs/msg/Float32                 → Generic single-value sensors
std_msgs/msg/Float32MultiArray       → Multi-axis sensor data (fallback)
diagnostic_msgs/msg/DiagnosticArray  → Device/bus health
```

---

## 8. Open Questions and Discussion Points

### 8.1 Architecture Decisions

1. **RTPS vs Zenoh as primary transport?**
   RTPS gives maximum compatibility with existing ROS 2 deployments. Zenoh is lighter and designed for exactly this use case. Could support both via a transport abstraction layer.

2. **Build-time vs runtime CDR generation?**
   Build-time generation of CDR encoders from DeviceTypeRecords would be more efficient and smaller (no runtime type interpretation). Runtime generation is more flexible for dynamically-added device types.

3. **Standard vs custom ROS message types?**
   Using only standard messages (sensor_msgs, std_msgs) maximizes interoperability. Custom message types would allow richer device-specific data but require building custom ROS packages on the host side.

4. **Node topology — single node or node-per-device?**
   A single ROS node per ESP32 is simpler and uses fewer RTPS resources. A node-per-device model (using ROS 2 lifecycle nodes) would be more idiomatic but heavier.

### 8.2 Technical Risks

| Risk | Severity | Mitigation |
|------|----------|------------|
| RTPS implementation complexity exceeds ESP32 resources | ~~High~~ Low (Phase 1 validated) | Phase 1 complete — RTPS fits comfortably on ESP32 with ~30KB RAM overhead |
| WiFi multicast unreliability | Medium | Support static peer configuration as fallback; consider unicast-only discovery mode |
| Interop issues between DDS implementations | Medium | Test against Fast DDS and Cyclone DDS from day one; use Wireshark RTPS dissector |
| CDR encoding bugs causing data corruption | Medium | Develop extensive unit tests; compare output with micro-CDR library |
| Dynamic topic creation causing SEDP storms | Low | Throttle announcements; batch device discovery notifications |

### 8.3 Testing Strategy

- **Unit tests (Linux):** CDR encoding/decoding, RTPS message construction/parsing, DeviceTypeRecord-to-ROS-message mapping — all testable without hardware using Raft's existing Linux unit test infrastructure
- **Integration tests:** ESP32 ↔ ROS 2 host communication using docker-based ROS 2 environment
- **Interop tests:** Verify with Fast DDS (default) and Cyclone DDS
- **Wireshark validation:** Capture and verify RTPS packets against known-good DDS traffic

---

## 9. Relationship to Raft Ecosystem

RaftROS would be a standard Raft library (idf_component / PlatformIO library), depending on:

- **RaftCore** — SysMod base, JSON config, REST API, CommsCoreIF
- **RaftSysMods** — StatePublisher (pub/sub core), NetworkManager (WiFi)
- **RaftI2C** — I2C bus management, device detection, DeviceTypeRecords
- Optionally **RaftMicroPy** / **RaftScript** — for scriptable topic mapping

It would **not** depend on any ROS 2 host-side packages for its core functionality (no cross-compilation of rcl/rclcpp). The ESP32 firmware is fully self-contained.

RaftROS fits into the transport layer alongside MQTTManager, CommandSocket, and BLEManager — all peers that register as CommsChannels and wire into StatePublisher subscriptions:

```
┌──────────────────────────────────────────────────────────┐
│  User Application (SysType.json configured)              │
├──────────────────────────────────────────────────────────┤
│  SysManager                                              │
│   ├── DeviceManager  (data sources: "devjson", "devbin") │
│   ├── StatePublisher (subscriptions, change detection)   │
│   ├── MQTTManager    (MQTT transport — existing)         │
│   ├── CommandSocket  (HTTP/WS transport — existing)      │
│   ├── BLEManager     (BLE transport — existing)          │
│   └── RaftROS        (RTPS transport — NEW)              │
│        ├── RTPS engine (wire protocol, clean-room)       │
│        ├── CDR serialization                             │
│        ├── Device-to-topic mapper                        │
│        └── Discovery (SPDP/SEDP)                         │
├──────────────────────────────────────────────────────────┤
│  RaftCore  │  RaftSysMods  │  RaftI2C  │  other libs     │
├──────────────────────────────────────────────────────────┤
│  ESP-IDF / Arduino + FreeRTOS + lwIP                     │
└──────────────────────────────────────────────────────────┘
```

---

## 10. Summary

RaftROS bridges two powerful paradigms:

- **Raft's** automatic device discovery, self-describing data (DeviceTypeRecords), and efficient binary data pipeline
- **ROS 2's** standardized robotics middleware with rich tool ecosystem, visualization (RViz), and computation graph

By implementing a minimal RTPS stack on ESP32, RaftROS would enable Raft-based firmware to participate natively in ROS 2 networks. The key innovation is **automatic mapping** from dynamically-detected devices (with their self-describing formats and actions) to ROS 2 topics and services — eliminating the manual configuration typically required when integrating embedded sensors into a ROS system.

Critically, RaftROS does not need to reinvent Raft's pub/sub infrastructure. The existing StatePublisher + CommsCoreIF architecture — already proven with MQTT, HTTP, BLE, and Serial transports — provides the data source management, change detection, rate limiting, and backoff logic. RaftROS's unique contribution is the RTPS wire protocol layer and CDR serialization, following the same SysMod integration pattern established by MQTTManager.

This approach makes Raft-based devices truly plug-and-play in ROS environments: connect a new I2C sensor, and a new ROS topic appears automatically with the correct message type, calibrated data, and discoverable metadata.
