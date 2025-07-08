# st3215_hardware

A ros2_control hardware package for interfacing with the st3215 digital servo.

## Overview

This package provides a ROS 2 hardware interface for the ST3215 digital servo motor. It implements the `ros2_control` hardware interface to enable seamless integration with ROS 2 control systems.

## Features

- Hardware interface for ST3215 digital servo
- Serial communication support
- ROS 2 control framework integration
- Lifecycle management support

## Dependencies

- ROS 2 (Humble or later)
- `hardware_interface`
- `rclcpp`
- `pluginlib`
- `rclcpp_lifecycle`
- `serial`
- `yaml-cpp`

## Building

```bash
colcon build --packages-select st3215_hardware
```

## Usage

Launch the hardware interface:

```bash
ros2 launch st3215_hardware st3215_bringup.launch.py
```

## Configuration

Hardware configuration files are located in the `config/` directory:
- `st3215_hardware.yaml` - Hardware interface configuration
- `st3215_controllers.yaml` - Controller configuration

## Testing

Run the test suite:

```bash
colcon test --packages-select st3215_hardware
```

## License

MIT
