# NTRIP Client ROS2 Package - Requirements Specification

**Version:** 2.0.0  
**Date:** October 21, 2025  
**Status:** Draft

---

## 1. Non-Functional Requirements (NFRs)

### 1.1 Performance Requirements

| ID | Requirement | Target | Priority |
|----|-------------|--------|----------|
| NFR-PERF-001 | RTCM message publishing latency | < 50ms from receipt to publish | High |
| NFR-PERF-002 | Support continuous operation | 24/7 without memory leaks | Critical |
| NFR-PERF-003 | CPU usage during normal operation | < 5% on single core | Medium |
| NFR-PERF-004 | Memory footprint | < 50MB RSS | Medium |
| NFR-PERF-005 | Handle RTCM data rates | Up to 10Hz (typical NTRIP servers) | High |
| NFR-PERF-006 | NMEA forwarding rate | Configurable, default 1Hz | High |
| NFR-PERF-007 | Reconnection time | < 10s under normal conditions | Medium |

### 1.2 Reliability Requirements

| ID | Requirement | Target | Priority |
|----|-------------|--------|----------|
| NFR-REL-001 | Automatic recovery from network failures | 100% within configured retry limits | Critical |
| NFR-REL-002 | RTCM data integrity | 100% (CRC validation) | Critical |
| NFR-REL-003 | NMEA data integrity | 100% (checksum validation) | Critical |
| NFR-REL-004 | Graceful degradation on connection loss | Continue attempting reconnect | High |
| NFR-REL-005 | No data loss on buffer overflow | Discard oldest data, log warning | Medium |
| NFR-REL-006 | Socket timeout handling | Detect and recover within timeout period | High |
| NFR-REL-007 | Thread-safe operations | All public APIs thread-safe | High |

### 1.3 Maintainability Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-MAINT-001 | Code coverage by unit tests: ≥ 85% | Critical |
| NFR-MAINT-002 | All public APIs fully documented with docstrings | Critical |
| NFR-MAINT-003 | Type hints on all functions and methods | Critical |
| NFR-MAINT-004 | Follow PEP 8 style guide | High |
| NFR-MAINT-005 | Modular architecture with clear separation of concerns | Critical |
| NFR-MAINT-006 | Maximum cyclomatic complexity per function: 10 | Medium |
| NFR-MAINT-007 | Maximum function length: 50 lines | Medium |
| NFR-MAINT-008 | Configuration via ROS2 parameters only (no env vars) | High |
| NFR-MAINT-009 | Meaningful variable and function names | High |
| NFR-MAINT-010 | Use modern Python 3.8+ features (dataclasses, pathlib, etc.) | High |

### 1.4 Testability Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-TEST-001 | Unit tests for all parsers (RTCM, NMEA) | Critical |
| NFR-TEST-002 | Unit tests for protocol client | Critical |
| NFR-TEST-003 | Integration tests with mock NTRIP server | High |
| NFR-TEST-004 | ROS2 node integration tests | High |
| NFR-TEST-005 | Mock/fake implementations for testing | High |
| NFR-TEST-006 | Continuous integration setup | Medium |
| NFR-TEST-007 | Property-based tests for parsers | Medium |

### 1.5 Usability Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-USE-001 | Clear error messages with actionable guidance | High |
| NFR-USE-002 | Comprehensive README with examples | High |
| NFR-USE-003 | Example launch files for common scenarios | High |
| NFR-USE-004 | Parameter validation with helpful error messages | High |
| NFR-USE-005 | Structured logging with appropriate levels | High |
| NFR-USE-006 | Status diagnostics publishing | Medium |

### 1.6 Portability Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-PORT-001 | Support Python 3.8+ | Critical |
| NFR-PORT-002 | Support ROS2 Humble, Iron, Jazzy, Rolling | Critical |
| NFR-PORT-003 | Platform support: Linux (Ubuntu 20.04+) | Critical |
| NFR-PORT-004 | No platform-specific dependencies | High |

### 1.7 Security Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-SEC-001 | Support SSL/TLS connections (v1.2+) | High |
| NFR-SEC-002 | Support client certificate authentication | High |
| NFR-SEC-003 | No plaintext credential logging | Critical |
| NFR-SEC-004 | Validate all input data (RTCM, NMEA) | Critical |
| NFR-SEC-005 | Protection against buffer overflow attacks | Critical |
| NFR-SEC-006 | Configurable certificate validation | High |

### 1.8 Scalability Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| NFR-SCALE-001 | Support multiple concurrent instances | High |
| NFR-SCALE-002 | Efficient buffer management for high data rates | High |
| NFR-SCALE-003 | No hardcoded limits that prevent scaling | Medium |

---

## 2. Functional Requirements (FRs)

### 2.1 NTRIP Protocol Support

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-NTRIP-001 | Support NTRIP v1.0 protocol | Critical |
| FR-NTRIP-002 | Support NTRIP v2.0 protocol | Critical |
| FR-NTRIP-003 | Auto-detect NTRIP version if not specified | Medium |
| FR-NTRIP-004 | Support HTTP/1.0 and HTTP/1.1 requests | Critical |
| FR-NTRIP-005 | Handle ICY 200 OK response (NTRIP v1) | Critical |
| FR-NTRIP-006 | Handle HTTP/1.x 200 OK responses (NTRIP v2) | Critical |
| FR-NTRIP-007 | Parse and handle SOURCETABLE responses | Medium |
| FR-NTRIP-008 | Parse and handle 401 Unauthorized responses | High |
| FR-NTRIP-009 | Custom User-Agent header | Low |

### 2.2 Network Communication

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-NET-001 | TCP/IP socket connection to NTRIP caster | Critical |
| FR-NET-002 | Configurable host, port, mountpoint | Critical |
| FR-NET-003 | Non-blocking socket I/O | High |
| FR-NET-004 | Connection timeout configuration | High |
| FR-NET-005 | Keep-alive mechanism | Medium |
| FR-NET-006 | Support IPv4 | Critical |
| FR-NET-007 | Support IPv6 | Low |

### 2.3 Authentication

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-AUTH-001 | HTTP Basic authentication support | High |
| FR-AUTH-002 | Unauthenticated connection support | High |
| FR-AUTH-003 | Base64 encoding of credentials | High |
| FR-AUTH-004 | Username and password configuration | High |

### 2.4 SSL/TLS Support

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-SSL-001 | Optional SSL/TLS connection | High |
| FR-SSL-002 | Client certificate authentication (cert + key) | High |
| FR-SSL-003 | Custom CA certificate support | High |
| FR-SSL-004 | Server hostname verification | High |
| FR-SSL-005 | Configurable SSL context options | Medium |
| FR-SSL-006 | Support TLS 1.2 and 1.3 | High |

### 2.5 RTCM Message Handling

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-RTCM-001 | Receive RTCM 3.x binary stream from server | Critical |
| FR-RTCM-002 | Parse RTCM 3.x message framing | Critical |
| FR-RTCM-003 | Validate RTCM preamble (0xD3) | Critical |
| FR-RTCM-004 | Extract 10-bit message length | Critical |
| FR-RTCM-005 | Validate 24-bit CRC-24Q checksum | Critical |
| FR-RTCM-006 | Buffer incomplete RTCM messages | High |
| FR-RTCM-007 | Handle multiple RTCM messages in single read | High |
| FR-RTCM-008 | Discard invalid RTCM messages | High |
| FR-RTCM-009 | Maximum buffer size protection | High |
| FR-RTCM-010 | Support all standard RTCM 3.x message types | Medium |
| FR-RTCM-011 | Log RTCM message types (for debugging) | Low |

### 2.6 NMEA Sentence Handling

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-NMEA-001 | Validate NMEA-0183 sentence format | Critical |
| FR-NMEA-002 | Verify NMEA checksum (XOR) | Critical |
| FR-NMEA-003 | Validate sentence start delimiter ($, !) | Critical |
| FR-NMEA-004 | Validate sentence terminator (\r\n) | Critical |
| FR-NMEA-005 | Configurable min/max sentence length | High |
| FR-NMEA-006 | Send NMEA to server at configurable rate | High |
| FR-NMEA-007 | Support filtering by NMEA sentence type | High |
| FR-NMEA-008 | Default filter: GGA sentences only | High |
| FR-NMEA-009 | Handle GNSS talker IDs (GP, GL, GA, GB, GN) | Medium |
| FR-NMEA-010 | Proper line ending handling (\r\n vs \\r\\n) | High |

### 2.7 Connection Management

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-CONN-001 | Automatic reconnection on connection failure | Critical |
| FR-CONN-002 | Configurable max reconnection attempts | High |
| FR-CONN-003 | Configurable wait time between reconnection attempts | High |
| FR-CONN-004 | Exponential backoff for reconnection (optional) | Medium |
| FR-CONN-005 | Detect connection drops via RTCM timeout | High |
| FR-CONN-006 | Detect connection drops via socket errors | High |
| FR-CONN-007 | Graceful shutdown and socket cleanup | High |
| FR-CONN-008 | Connection state machine (disconnected, connecting, connected, reconnecting) | High |

### 2.8 ROS2 Integration

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-ROS-001 | Implement as ROS2 lifecycle node | High |
| FR-ROS-002 | Publish RTCM on `/rtcm` topic | Critical |
| FR-ROS-003 | Subscribe to NMEA on `/nmea` topic | Critical |
| FR-ROS-004 | Support `mavros_msgs/RTCM` message type | High |
| FR-ROS-005 | Support `rtcm_msgs/Message` message type | High |
| FR-ROS-006 | Configurable message type selection | High |
| FR-ROS-007 | Add header with timestamp to published messages | High |
| FR-ROS-008 | Configurable frame_id for messages | Medium |
| FR-ROS-009 | All configuration via ROS2 parameters | Critical |
| FR-ROS-010 | Parameter validation on startup | High |
| FR-ROS-011 | Support topic remapping | High |
| FR-ROS-012 | Support namespace configuration | High |
| FR-ROS-013 | Publish diagnostics on `/diagnostics` topic | Medium |
| FR-ROS-014 | Publish connection status | Medium |
| FR-ROS-015 | Service to trigger reconnection | Low |
| FR-ROS-016 | Service to query connection status | Low |

### 2.9 Logging and Diagnostics

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-LOG-001 | Use ROS2 logger for all logging | Critical |
| FR-LOG-002 | Configurable log levels | High |
| FR-LOG-003 | Log connection events (connect, disconnect, reconnect) | High |
| FR-LOG-004 | Log authentication status | High |
| FR-LOG-005 | Log RTCM message statistics | Medium |
| FR-LOG-006 | Log NMEA message statistics | Medium |
| FR-LOG-007 | Log parser errors with details | High |
| FR-LOG-008 | Never log passwords/credentials | Critical |
| FR-LOG-009 | Rate-limited warning messages | Medium |
| FR-LOG-010 | Publish diagnostic messages (status, errors, warnings) | Medium |

### 2.10 Configuration and Parameters

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-CONF-001 | Server host (string, required) | Critical |
| FR-CONF-002 | Server port (int, default 2101) | Critical |
| FR-CONF-003 | Mountpoint (string, required) | Critical |
| FR-CONF-004 | NTRIP version (string, optional) | High |
| FR-CONF-005 | Authentication enable (bool, default false) | High |
| FR-CONF-006 | Username (string, conditional) | High |
| FR-CONF-007 | Password (string, conditional) | High |
| FR-CONF-008 | SSL enable (bool, default false) | High |
| FR-CONF-009 | SSL cert path (string, optional) | High |
| FR-CONF-010 | SSL key path (string, optional) | High |
| FR-CONF-011 | SSL CA cert path (string, optional) | High |
| FR-CONF-012 | RTCM frame_id (string, default "odom") | Medium |
| FR-CONF-013 | NMEA max length (int, default 82) | Medium |
| FR-CONF-014 | NMEA min length (int, default 3) | Medium |
| FR-CONF-015 | RTCM message package (enum, default rtcm_msgs) | High |
| FR-CONF-016 | Reconnect max attempts (int, default 10) | High |
| FR-CONF-017 | Reconnect wait seconds (float, default 5.0) | High |
| FR-CONF-018 | RTCM timeout seconds (float, default 4.0) | High |
| FR-CONF-019 | Server poll rate Hz (float, default 1.0) | High |
| FR-CONF-020 | NMEA sentence filter (list, default ["GNGGA", "GPGGA"]) | High |
| FR-CONF-021 | Connection timeout seconds (float, default 10.0) | Medium |
| FR-CONF-022 | Enable diagnostics publishing (bool, default true) | Medium |

### 2.11 Error Handling

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-ERR-001 | Detect and report invalid parameters | Critical |
| FR-ERR-002 | Handle socket creation failures | Critical |
| FR-ERR-003 | Handle connection failures | Critical |
| FR-ERR-004 | Handle SSL handshake failures | High |
| FR-ERR-005 | Handle authentication failures | High |
| FR-ERR-006 | Handle invalid RTCM data | High |
| FR-ERR-007 | Handle invalid NMEA data | High |
| FR-ERR-008 | Handle buffer overflow scenarios | High |
| FR-ERR-009 | Provide specific error messages for common issues | High |
| FR-ERR-010 | Never crash on invalid input | Critical |

---

## 3. Architecture Requirements

### 3.1 Module Structure

| ID | Requirement | Priority |
|----|-------------|----------|
| AR-MOD-001 | Separate transport layer (sockets, SSL) | Critical |
| AR-MOD-002 | Separate protocol layer (NTRIP HTTP) | Critical |
| AR-MOD-003 | Separate parser layer (RTCM, NMEA) | Critical |
| AR-MOD-004 | Separate ROS2 node layer | Critical |
| AR-MOD-005 | Use dependency injection for testability | High |
| AR-MOD-006 | Define clear interfaces between layers | High |

### 3.2 Design Patterns

| ID | Requirement | Priority |
|----|-------------|----------|
| AR-PAT-001 | State machine for connection management | High |
| AR-PAT-002 | Strategy pattern for RTCM message types | Medium |
| AR-PAT-003 | Observer pattern for connection events | Medium |
| AR-PAT-004 | Factory pattern for parser creation | Medium |

---

## 4. Package Structure Requirements

### 4.1 Directory Layout

```
ntrip_client_ros2/
├── ntrip_client/                    # Main package
│   ├── __init__.py
│   ├── transport/                   # Transport layer
│   │   ├── __init__.py
│   │   ├── socket_transport.py      # TCP socket implementation
│   │   └── ssl_transport.py         # SSL/TLS wrapper
│   ├── protocol/                    # Protocol layer
│   │   ├── __init__.py
│   │   ├── ntrip_client.py          # NTRIP protocol implementation
│   │   └── http_parser.py           # HTTP response parsing
│   ├── parsers/                     # Parser layer
│   │   ├── __init__.py
│   │   ├── rtcm_parser.py           # RTCM 3.x parser
│   │   └── nmea_parser.py           # NMEA-0183 parser
│   ├── ros/                         # ROS2 integration
│   │   ├── __init__.py
│   │   ├── ntrip_node.py            # Main ROS2 node
│   │   └── message_adapters.py      # Message type adapters
│   └── utils/                       # Utilities
│       ├── __init__.py
│       ├── validators.py            # Parameter validators
│       └── state_machine.py         # Connection state machine
├── test/                            # Tests
│   ├── unit/
│   ├── integration/
│   └── mocks/
├── launch/                          # Launch files
│   └── ntrip_client.launch.py
├── config/                          # Example configs
│   └── example_config.yaml
├── docs/                            # Documentation
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## 5. Success Criteria

| Criteria | Target |
|----------|--------|
| All functional requirements implemented | 100% |
| Unit test coverage | ≥ 85% |
| Integration tests passing | 100% |
| Documentation complete | 100% |
| Zero critical/high priority bugs | 100% |
| Performance targets met | 100% |
| Backward compatibility with existing launch files | 90% |

---

## 6. Out of Scope (for v2.0)

- NTRIP Caster functionality (server mode)
- NTRIP Sourcetable parsing and display
- GUI/visualization tools
- RTCM message decoding (remains binary)
- Multi-mountpoint support in single node
- CORS station configuration
- Automatic base station selection

---

## 7. Future Enhancements (Post v2.0)

- Async I/O with `asyncio` for better performance
- RTCM message decoding and publishing individual correction types
- Health monitoring with automatic caster failover
- Performance metrics and profiling tools
- Docker containerization
- Cloud NTRIP services integration
- Web-based configuration interface

