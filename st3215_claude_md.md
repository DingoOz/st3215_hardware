# ST3215 ROS2 Control Hardware Package

## Project Overview
Create a ROS2 control hardware package to interface with ST3215 digital servos using modern C++20 features, RAII principles, and Google C++ standards.

## Development Guidelines
- **Language**: C++20 preferred, with fallback to C++17 where necessary
- **Target**: ROS2 Jazzy
- **Standards**: Google C++ Style Guide
- **Resource Management**: RAII throughout
- **Authorship**: All commits should be authored by the user

## Package Structure
```
st3215_hardware/
├── CMakeLists.txt
├── package.xml
├── include/st3215_hardware/
│   ├── st3215_hardware_interface.hpp
│   ├── st3215_communication.hpp
│   └── st3215_servo.hpp
├── src/
│   ├── st3215_hardware_interface.cpp
│   ├── st3215_communication.cpp
│   └── st3215_servo.cpp
├── config/
│   └── st3215_controllers.yaml
├── launch/
│   └── st3215_bringup.launch.py
├── test/
│   ├── test_st3215_communication.cpp
│   └── test_st3215_hardware_interface.cpp
└── README.md
```

## Core Components

### 1. ST3215Servo Class (RAII Wrapper)
**Purpose**: Encapsulate single servo operations with automatic resource management

**Key Features**:
- RAII for serial port management
- Exception-safe operations
- Move semantics for efficient transfers
- Thread-safe state access

**Header Structure**:
```cpp
class ST3215Servo {
public:
    ST3215Servo(const std::string& port, uint8_t id);
    ~ST3215Servo() = default;
    
    // Move-only semantics
    ST3215Servo(const ST3215Servo&) = delete;
    ST3215Servo& operator=(const ST3215Servo&) = delete;
    ST3215Servo(ST3215Servo&&) = default;
    ST3215Servo& operator=(ST3215Servo&&) = default;
    
    // Core operations
    bool set_position(double position_rad);
    bool set_velocity(double velocity_rad_s);
    bool set_effort(double effort_nm);
    
    // State reading
    std::optional<double> get_position() const;
    std::optional<double> get_velocity() const;
    std::optional<double> get_effort() const;
    std::optional<double> get_temperature() const;
    std::optional<double> get_voltage() const;
    
private:
    std::unique_ptr<ST3215Communication> comm_;
    uint8_t servo_id_;
    mutable std::mutex state_mutex_;
};
```

### 2. ST3215Communication Class
**Purpose**: Handle low-level protocol communication

**Key Features**:
- Thread-safe UART operations
- Timeout handling with std::chrono
- Packet validation and checksums
- Connection management

**Responsibilities**:
- Serial port initialization and cleanup
- Protocol packet construction/parsing
- Error detection and recovery
- Thread-safe command queuing

### 3. ST3215HardwareInterface Class
**Purpose**: ROS2 control integration

**Base Class**: `hardware_interface::SystemInterface`

**Lifecycle Methods**:
- `on_configure()`: Initialize serial communication and servo discovery
- `on_activate()`: Enable all servos and start communication threads
- `on_deactivate()`: Safely disable servos and stop threads
- `on_cleanup()`: Release all resources (RAII cleanup)

**Interfaces**:
- **State Interfaces**: position, velocity, effort, temperature, voltage
- **Command Interfaces**: position, velocity, effort

## Dependencies (package.xml)
```xml
<depend>rclcpp</depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp_lifecycle</depend>
<depend>serial</depend>
<depend>yaml-cpp</depend>
<test_depend>gtest</test_depend>
<test_depend>gmock</test_depend>
```

## C++20 Features to Implement
- `std::span` for safe buffer management
- `std::jthread` for background communication threads
- `std::atomic` for thread-safe state sharing
- `std::chrono` for precise timing operations
- Concepts for template parameter validation
- `std::optional` for nullable return values
- `std::format` for structured logging (if available)

## Configuration Strategy

### YAML Configuration Structure
```yaml
st3215_hardware:
  port: "/dev/ttyUSB0"
  baudrate: 1000000
  timeout_ms: 100
  servos:
    - id: 1
      name: "joint_1"
      position_limits: [-3.14, 3.14]
      velocity_limits: [-5.0, 5.0]
      effort_limits: [-10.0, 10.0]
    - id: 2
      name: "joint_2"
      position_limits: [-1.57, 1.57]
      velocity_limits: [-3.0, 3.0]
      effort_limits: [-5.0, 5.0]
```

## Error Handling Strategy
- **RAII**: All resources automatically managed
- **Exception Safety**: Strong exception safety guarantees
- **Graceful Degradation**: Continue operation with partial failures
- **Emergency Stop**: Immediate servo disable on critical errors
- **Logging**: Structured logging with severity levels

## Testing Requirements
1. **Unit Tests**: Individual component testing
2. **Integration Tests**: Full system testing with mock hardware
3. **Performance Tests**: Latency and throughput benchmarks
4. **Hardware Tests**: Real servo validation scripts

## Plugin Registration
- Export hardware interface plugin via pluginlib
- Proper plugin.xml configuration
- Symbol visibility management for shared libraries

## Protocol Implementation Notes
**Note**: ST3215 protocol details need to be researched and documented

**Common Digital Servo Features**:
- UART communication (typically 1Mbps)
- Packet-based protocol with checksums
- Register-based configuration
- Position, velocity, and torque control modes
- Status reporting (position, load, temperature, voltage)

## Development Tasks
1. Research ST3215 protocol documentation
2. Implement ST3215Communication class
3. Create ST3215Servo RAII wrapper
4. Develop ST3215HardwareInterface
5. Write comprehensive tests
6. Create configuration and launch files
7. Document usage and troubleshooting

## Questions for Implementation
1. What is the exact ST3215 communication protocol?
2. How many servos will be controlled simultaneously?
3. What are the performance requirements (update rate, latency)?
4. Are there specific safety requirements?
5. What serial communication library is preferred?

## Additional Considerations
- **Thread Safety**: All operations must be thread-safe
- **Real-time**: Consider real-time constraints for control loops
- **Diagnostics**: Implement comprehensive diagnostic reporting
- **Parameter Updates**: Support runtime parameter changes
- **Hot-plug**: Handle servo connection/disconnection gracefully