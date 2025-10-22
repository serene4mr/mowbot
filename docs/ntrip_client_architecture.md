# NTRIP Client ROS2 - Architecture Design

**Version:** 2.0.0  
**Date:** October 21, 2025  
**Status:** Draft

---

## 1. Architecture Overview

### 1.1 Layered Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     ROS2 Application Layer                   │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         NTRIPNode (Lifecycle Node)                     │ │
│  │  - Parameter management                                │ │
│  │  - Topic pub/sub                                       │ │
│  │  - Lifecycle state management                          │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                    Protocol Layer                            │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         NTRIPProtocolClient                            │ │
│  │  - NTRIP protocol state machine                        │ │
│  │  - HTTP request/response handling                      │ │
│  │  - Authentication logic                                │ │
│  │  - Connection management                               │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                    Transport Layer                           │
│  ┌─────────────────────┐      ┌─────────────────────────┐  │
│  │  SocketTransport    │      │   SSLTransport          │  │
│  │  - TCP/IP sockets   │      │   - TLS/SSL wrapper     │  │
│  │  - Read/Write ops   │      │   - Certificate mgmt    │  │
│  └─────────────────────┘      └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                    Parser Layer                              │
│  ┌─────────────────────┐      ┌─────────────────────────┐  │
│  │   RTCMParser        │      │   NMEAParser            │  │
│  │  - Frame detection  │      │   - Sentence validation │  │
│  │  - CRC validation   │      │   - Checksum validation │  │
│  │  - Message buffering│      │   - Format checking     │  │
│  └─────────────────────┘      └─────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            ↕
┌─────────────────────────────────────────────────────────────┐
│                    Utilities                                 │
│  - State Machine                                             │
│  - Validators                                                │
│  - Message Adapters                                          │
│  - Configuration                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.2 Data Flow

```
NTRIP Server ←──────────────────────────────────────────┐
     │                                                   │
     │ RTCM 3.x Binary Stream                     NMEA Sentences
     ↓                                                   │
┌─────────────────────┐                                 │
│  SocketTransport    │                                 │
│  or SSLTransport    │                                 │
└─────────────────────┘                                 │
     │                                                   │
     │ Raw bytes                                         │
     ↓                                                   │
┌─────────────────────┐                      ┌──────────────────┐
│   RTCMParser        │                      │   NMEAParser     │
└─────────────────────┘                      └──────────────────┘
     │                                                   ↑
     │ Validated RTCM messages                   Validated NMEA
     ↓                                                   │
┌─────────────────────┐                      ┌──────────────────┐
│ NTRIPProtocolClient │ ────────────────────→│ NTRIPProtocolClient
└─────────────────────┘                      └──────────────────┘
     │                                                   ↑
     │ RTCM data                                   NMEA from ROS
     ↓                                                   │
┌─────────────────────┐                      ┌──────────────────┐
│    NTRIPNode        │                      │   NTRIPNode      │
│  (ROS2 Publisher)   │                      │ (ROS2 Subscriber)│
└─────────────────────┘                      └──────────────────┘
     │                                                   ↑
     │ mavros_msgs/RTCM                      nmea_msgs/Sentence
     │ or rtcm_msgs/Message                              │
     ↓                                                   │
  /rtcm topic                                      /nmea topic
```

---

## 2. Component Design

### 2.1 Transport Layer

#### 2.1.1 SocketTransport

**Purpose:** Low-level TCP/IP socket operations

**Responsibilities:**
- Socket creation and configuration
- Connection establishment
- Non-blocking I/O
- Socket health monitoring
- Clean disconnection

**Interface:**
```python
@dataclass
class TransportConfig:
    host: str
    port: int
    timeout: float = 10.0
    buffer_size: int = 4096

class SocketTransport(ABC):
    @abstractmethod
    def connect(self) -> bool: ...
    
    @abstractmethod
    def disconnect(self) -> None: ...
    
    @abstractmethod
    def send(self, data: bytes) -> int: ...
    
    @abstractmethod
    def receive(self, max_bytes: int) -> bytes: ...
    
    @abstractmethod
    def is_connected(self) -> bool: ...
    
    @property
    @abstractmethod
    def available_data(self) -> bool: ...
```

#### 2.1.2 SSLTransport

**Purpose:** SSL/TLS wrapper for secure connections

**Extends:** SocketTransport

**Additional Responsibilities:**
- SSL context configuration
- Certificate loading and validation
- TLS handshake
- Encrypted data transmission

**Interface:**
```python
@dataclass
class SSLConfig:
    enable_ssl: bool = False
    cert_file: Optional[Path] = None
    key_file: Optional[Path] = None
    ca_cert_file: Optional[Path] = None
    verify_hostname: bool = True
    check_hostname: bool = True

class SSLTransport(SocketTransport):
    def __init__(self, config: TransportConfig, ssl_config: SSLConfig): ...
```

---

### 2.2 Protocol Layer

#### 2.2.1 NTRIPProtocolClient

**Purpose:** NTRIP protocol implementation

**Responsibilities:**
- HTTP request formatting (GET with NTRIP headers)
- HTTP response parsing
- Authentication (Basic Auth)
- Connection state management
- Reconnection logic
- Data streaming

**State Machine:**
```
    DISCONNECTED
         │
         │ connect()
         ↓
    CONNECTING ──→ AUTHENTICATING
         │              │
         │ error        │ success
         ↓              ↓
      FAILED       CONNECTED ←──┐
         │              │        │
         │              │ data   │
         │              ↓        │
         │         STREAMING     │
         │              │        │
         └──← timeout/error      │
                        │        │
                        ↓        │
                   RECONNECTING ─┘
```

**Interface:**
```python
@dataclass
class NTRIPConfig:
    host: str
    port: int
    mountpoint: str
    ntrip_version: Optional[str] = None
    username: Optional[str] = None
    password: Optional[str] = None
    user_agent: str = "ntrip_client_ros2/2.0"
    
class ConnectionState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    AUTHENTICATING = "authenticating"
    CONNECTED = "connected"
    STREAMING = "streaming"
    RECONNECTING = "reconnecting"
    FAILED = "failed"

class NTRIPProtocolClient:
    def __init__(
        self,
        config: NTRIPConfig,
        transport: SocketTransport,
        rtcm_parser: RTCMParser,
        nmea_parser: NMEAParser,
    ): ...
    
    def connect(self) -> bool: ...
    
    def disconnect(self) -> None: ...
    
    def reconnect(self) -> bool: ...
    
    def send_nmea(self, sentence: str) -> bool: ...
    
    def receive_rtcm(self) -> List[bytes]: ...
    
    @property
    def state(self) -> ConnectionState: ...
    
    @property
    def is_connected(self) -> bool: ...
```

---

### 2.3 Parser Layer

#### 2.3.1 RTCMParser

**Purpose:** Parse and validate RTCM 3.x messages

**Responsibilities:**
- Detect RTCM preamble (0xD3)
- Extract message length
- Validate CRC-24Q checksum
- Buffer incomplete messages
- Handle multiple messages in stream

**Algorithm:**
1. Scan buffer for preamble byte (0xD3)
2. Read next 2 bytes for message length (10-bit)
3. Wait for complete message (length + 3 CRC bytes)
4. Validate CRC
5. Yield valid message
6. Continue scanning

**Interface:**
```python
@dataclass
class RTCMParserConfig:
    max_buffer_size: int = 10240  # 10KB
    max_message_size: int = 1024  # RTCM max ~1KB

class RTCMMessage:
    raw_data: bytes
    message_type: Optional[int] = None
    length: int = field(init=False)
    
    def __post_init__(self):
        self.length = len(self.raw_data)

class RTCMParser:
    def __init__(self, config: RTCMParserConfig = RTCMParserConfig()): ...
    
    def parse(self, data: bytes) -> List[RTCMMessage]: ...
    
    def reset(self) -> None: ...
    
    @staticmethod
    def calculate_crc24q(data: bytes) -> int: ...
    
    @staticmethod
    def validate_crc(message: bytes) -> bool: ...
```

#### 2.3.2 NMEAParser

**Purpose:** Validate NMEA-0183 sentences

**Responsibilities:**
- Format validation (start, end, checksum separator)
- Length validation
- XOR checksum calculation and verification
- Sentence type filtering

**Interface:**
```python
@dataclass
class NMEAParserConfig:
    min_length: int = 3
    max_length: int = 82
    allowed_talker_ids: List[str] = field(
        default_factory=lambda: ["GP", "GL", "GA", "GB", "GN"]
    )
    allowed_sentence_types: List[str] = field(
        default_factory=lambda: ["GGA"]
    )

@dataclass
class NMEASentence:
    raw: str
    talker_id: str
    sentence_type: str
    fields: List[str]
    checksum: int

class NMEAParser:
    def __init__(self, config: NMEAParserConfig = NMEAParserConfig()): ...
    
    def parse(self, sentence: str) -> Optional[NMEASentence]: ...
    
    def validate(self, sentence: str) -> bool: ...
    
    @staticmethod
    def calculate_checksum(sentence: str) -> int: ...
    
    def should_accept(self, sentence: NMEASentence) -> bool: ...
```

---

### 2.4 ROS2 Integration Layer

#### 2.4.1 NTRIPNode (Lifecycle Node)

**Purpose:** ROS2 interface to NTRIP client

**Lifecycle States:**
```
Unconfigured → Inactive → Active
                  ↑          ↓
                  └──────────┘
```

**Responsibilities:**
- Declare and validate ROS2 parameters
- Create publishers and subscribers
- Manage lifecycle transitions
- Handle timers for polling
- Publish RTCM messages
- Subscribe to NMEA messages
- Publish diagnostics

**Interface:**
```python
class NTRIPNode(LifecycleNode):
    def __init__(self): ...
    
    # Lifecycle callbacks
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn: ...
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn: ...
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn: ...
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn: ...
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn: ...
    
    # Parameter management
    def _declare_parameters(self) -> None: ...
    def _load_parameters(self) -> Dict[str, Any]: ...
    def _validate_parameters(self, params: Dict[str, Any]) -> bool: ...
    
    # ROS callbacks
    def _nmea_callback(self, msg: Sentence) -> None: ...
    def _timer_callback(self) -> None: ...
    
    # Publishing
    def _publish_rtcm(self, messages: List[bytes]) -> None: ...
    def _publish_diagnostics(self, status: DiagnosticStatus) -> None: ...
```

#### 2.4.2 MessageAdapters

**Purpose:** Convert between internal data and ROS message types

**Responsibilities:**
- Create mavros_msgs/RTCM messages
- Create rtcm_msgs/Message messages
- Handle message type selection

**Interface:**
```python
class RTCMMessageAdapter(ABC):
    @abstractmethod
    def create_message(
        self, 
        rtcm_data: bytes, 
        frame_id: str, 
        timestamp: Time
    ) -> Any: ...

class MavrosRTCMAdapter(RTCMMessageAdapter):
    def create_message(self, ...) -> mavros_msgs.RTCM: ...

class RTCMMsgAdapter(RTCMMessageAdapter):
    def create_message(self, ...) -> rtcm_msgs.Message: ...

class MessageAdapterFactory:
    @staticmethod
    def create(package: str) -> RTCMMessageAdapter: ...
```

---

### 2.5 Utilities

#### 2.5.1 State Machine

**Purpose:** Generic state machine for connection management

**Features:**
- Define states and transitions
- Transition guards and callbacks
- State entry/exit actions
- Event-driven transitions

#### 2.5.2 Validators

**Purpose:** Parameter validation

**Functions:**
- Validate host format (IP or hostname)
- Validate port range
- Validate file paths (certs, keys)
- Validate configuration consistency

#### 2.5.3 Configuration

**Purpose:** Centralized configuration management

**Features:**
- Load from ROS parameters
- Load from YAML files
- Validation
- Default values

---

## 3. Error Handling Strategy

### 3.1 Error Categories

| Category | Handling Strategy | Recovery |
|----------|-------------------|----------|
| Configuration Error | Fail fast, clear message | User fixes config |
| Network Error | Log, retry with backoff | Auto-reconnect |
| Protocol Error | Log, attempt reconnect | Auto-reconnect |
| Parse Error | Log, discard data, continue | Continue operation |
| Resource Error | Log, graceful degradation | Best effort |

### 3.2 Exception Hierarchy

```python
class NTRIPClientError(Exception):
    """Base exception for NTRIP client"""

class ConfigurationError(NTRIPClientError):
    """Invalid configuration"""

class ConnectionError(NTRIPClientError):
    """Connection related errors"""

class AuthenticationError(ConnectionError):
    """Authentication failed"""

class ProtocolError(NTRIPClientError):
    """NTRIP protocol error"""

class ParseError(NTRIPClientError):
    """Data parsing error"""

class TransportError(NTRIPClientError):
    """Transport layer error"""
```

---

## 4. Threading Model

### 4.1 Single-Threaded Design

- Main thread: ROS2 executor spinning
- Timer callbacks: RTCM polling and NMEA sending
- Non-blocking I/O: Use select() for socket availability

**Rationale:**
- Simpler design, easier to debug
- No race conditions
- Sufficient for typical NTRIP data rates (< 10Hz)

### 4.2 Thread Safety

- All public APIs designed for single-threaded access
- Document thread safety guarantees
- Use locks only if multi-threaded extension needed

---

## 5. Performance Considerations

### 5.1 Memory Management

- Fixed buffer sizes with overflow protection
- Reuse message objects where possible
- Clear buffers on reconnect

### 5.2 CPU Optimization

- Efficient parsing (single-pass where possible)
- Avoid unnecessary copies
- Use bytearray for buffer manipulation

### 5.3 I/O Optimization

- Non-blocking socket operations
- Batch RTCM message publishing
- Rate-limited logging

---

## 6. Testing Strategy

### 6.1 Unit Tests

- Each component independently testable
- Mock dependencies
- Property-based testing for parsers
- 85%+ coverage target

### 6.2 Integration Tests

- Mock NTRIP server
- Full protocol flow testing
- Error scenario testing
- ROS2 integration testing

### 6.3 Test Fixtures

- Sample RTCM messages
- Sample NMEA sentences
- Various HTTP responses
- Error conditions

---

## 7. Dependencies

### 7.1 Python Standard Library

- `socket`, `ssl` - Networking
- `select` - Non-blocking I/O
- `dataclasses` - Data structures
- `enum` - Enumerations
- `typing` - Type hints
- `logging` - Logging (wrapped by ROS)
- `pathlib` - Path handling
- `base64` - Authentication encoding

### 7.2 ROS2 Dependencies

- `rclpy` - ROS2 Python client library
- `lifecycle_msgs` - Lifecycle node
- `diagnostic_msgs` - Diagnostics
- `std_msgs` - Standard messages
- `nmea_msgs` - NMEA messages
- `mavros_msgs` (optional) - RTCM messages
- `rtcm_msgs` (optional) - RTCM messages

### 7.3 Testing Dependencies

- `pytest` - Test framework
- `pytest-cov` - Coverage
- `pytest-mock` - Mocking
- `hypothesis` - Property-based testing

---

## 8. Build and Packaging

### 8.1 Package Structure

- Standard ament_python package
- setup.py with proper entry points
- package.xml with all dependencies
- Launch files in share directory
- Example configs in share directory

### 8.2 Installation

```bash
# Build from source
cd ~/ros2_ws/src
git clone <repo>
cd ~/ros2_ws
colcon build --packages-select ntrip_client_ros2
source install/setup.bash
```

---

## 9. Configuration Examples

### 9.1 Basic Configuration

```yaml
ntrip_client:
  ros__parameters:
    host: "rtk2go.com"
    port: 2101
    mountpoint: "MOUNT_NAME"
    authenticate: false
```

### 9.2 Full Configuration

```yaml
ntrip_client:
  ros__parameters:
    # Connection
    host: "ntrip.example.com"
    port: 2101
    mountpoint: "BASE_STATION"
    ntrip_version: "Ntrip/2.0"
    
    # Authentication
    authenticate: true
    username: "user"
    password: "pass"
    
    # SSL
    ssl: true
    cert: "/path/to/cert.pem"
    key: "/path/to/key.pem"
    ca_cert: "/path/to/ca.pem"
    
    # Message options
    rtcm_frame_id: "base_link"
    rtcm_message_package: "rtcm_msgs"
    
    # NMEA filtering
    nmea_sentence_filter: ["GNGGA", "GPGGA"]
    nmea_max_length: 82
    nmea_min_length: 3
    
    # Connection management
    connection_timeout: 10.0
    reconnect_max_attempts: 10
    reconnect_wait_seconds: 5.0
    rtcm_timeout_seconds: 4.0
    
    # Rates
    server_poll_rate: 1.0
```

---

## 10. Deployment Considerations

### 10.1 Lifecycle Management

Use lifecycle node for proper startup/shutdown:

```bash
ros2 lifecycle set /ntrip_client configure
ros2 lifecycle set /ntrip_client activate
```

### 10.2 Monitoring

- Monitor `/diagnostics` topic for health
- Check connection status regularly
- Alert on persistent disconnections

### 10.3 Resource Limits

- Memory: < 50MB typical
- CPU: < 5% on modern systems
- Network: ~1-10KB/s depending on corrections

---

This architecture provides a solid foundation for a maintainable, testable, and robust NTRIP client implementation.

