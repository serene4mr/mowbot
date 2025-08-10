# RMW Implementation

This role installs and configures ROS 2 RMW (ROS Middleware) implementations to enable different communication protocols and performance characteristics.

## Purpose

RMW implementations provide the communication layer for ROS 2, enabling different middleware solutions with varying performance characteristics:

- **rmw_cyclonedds_cpp** - High-performance DDS implementation (default)
- **rmw_fastrtps_cpp** - FastRTPS DDS implementation
- **rmw_connextdds** - RTI Connext DDS implementation
- **rmw_opensplice_cpp** - OpenSplice DDS implementation

This role is essential for:
- **ROS 2 communication** setup and configuration
- **Performance optimization** for different use cases
- **Multi-RMW environments** with different middleware needs
- **Robotics applications** requiring specific communication protocols

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `rosdistro` | `"humble"` | ROS 2 distribution (e.g., humble, iron, rolling) |
| `rmw_implementation` | `"rmw_cyclonedds_cpp"` | RMW implementation to install |
| `rmw_verify_installation` | `true` | Whether to verify installation success |

## Supported RMW Implementations

### rmw_cyclonedds_cpp (Recommended)
- **Performance**: High performance, low latency
- **Memory**: Efficient memory usage
- **Use cases**: Real-time applications, embedded systems
- **Package**: `ros-{distro}-rmw-cyclonedds-cpp`

### rmw_fastrtps_cpp
- **Performance**: Good performance, widely used
- **Memory**: Moderate memory usage
- **Use cases**: General purpose, development
- **Package**: `ros-{distro}-rmw-fastrtps-cpp`

### rmw_connextdds
- **Performance**: Enterprise-grade performance
- **Memory**: Higher memory usage
- **Use cases**: Enterprise applications, large deployments
- **Package**: `ros-{distro}-rmw-connextdds`

### rmw_opensplice_cpp
- **Performance**: Legacy implementation
- **Memory**: Moderate memory usage
- **Use cases**: Legacy applications, compatibility
- **Package**: `ros-{distro}-rmw-opensplice-cpp`

## Installation Process

The role performs the following steps:

1. **Update package cache** - Ensures latest package information
2. **Install RMW package** - Installs the specified RMW implementation
3. **Configure .bashrc** - Sets RMW_IMPLEMENTATION environment variable
4. **Configure .profile** - Sets RMW_IMPLEMENTATION for login shells
5. **Verify installation** - Checks if package is properly installed
6. **Display status** - Provides feedback on installation and configuration

## Usage Examples

### Basic installation (default)
```yaml
- hosts: localhost
  roles:
    - rmw_implementation
```

### Custom RMW implementation
```yaml
- hosts: localhost
  roles:
    - rmw_implementation
  vars:
    rmw_implementation: rmw_fastrtps_cpp
```

### Different ROS distribution
```yaml
- hosts: localhost
  roles:
    - rmw_implementation
  vars:
    rosdistro: iron
    rmw_implementation: rmw_cyclonedds_cpp
```

### Disable verification
```yaml
- hosts: localhost
  roles:
    - rmw_implementation
  vars:
    rmw_verify_installation: false
```

## Manual Installation

If you need to install manually:

```bash
# Set environment variables
export rosdistro=humble
export rmw_implementation=rmw_cyclonedds_cpp

# Update package lists
sudo apt update

# Install RMW implementation
rmw_implementation_dashed=$(echo "${rmw_implementation}" | sed 's/_/-/g')
sudo apt install ros-${rosdistro}-${rmw_implementation_dashed}

# Configure environment variables
echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.profile

# Verify installation
dpkg -l | grep "ros-${rosdistro}-${rmw_implementation_dashed}"

# Test in new shell
source ~/.bashrc
echo $RMW_IMPLEMENTATION
```

## Testing

### Verify installation
```bash
# Check if package is installed
dpkg -l | grep "ros-humble-rmw-cyclonedds-cpp"

# Check environment variable
echo $RMW_IMPLEMENTATION
```

### Test ROS 2 communication
```bash
# Start ROS 2 daemon
ros2 daemon start

# Check RMW implementation
ros2 doctor --report

# Test publisher/subscriber
ros2 topic pub /test std_msgs/msg/String "data: 'Hello RMW'"
```

### Switch RMW implementations
```bash
# Set different RMW implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Verify change
ros2 doctor --report
```

## Troubleshooting

### Package not found
```bash
# Check available RMW packages
apt search ros-humble-rmw

# Check ROS 2 installation
ros2 doctor --report
```

### Environment variable not set
```bash
# Check current shell
echo $RMW_IMPLEMENTATION

# Check configuration files
grep RMW_IMPLEMENTATION ~/.bashrc
grep RMW_IMPLEMENTATION ~/.profile

# Reload shell configuration
source ~/.bashrc
```

### Communication issues
```bash
# Check ROS 2 daemon
ros2 daemon status

# Restart daemon
ros2 daemon stop
ros2 daemon start

# Check network configuration
ros2 doctor --report
```

## Performance Considerations

### rmw_cyclonedds_cpp (Recommended)
- **Best for**: Real-time applications, embedded systems
- **Memory usage**: Low
- **Latency**: Very low
- **Throughput**: High

### rmw_fastrtps_cpp
- **Best for**: General development, compatibility
- **Memory usage**: Moderate
- **Latency**: Low
- **Throughput**: Good

### rmw_connextdds
- **Best for**: Enterprise applications, large deployments
- **Memory usage**: High
- **Latency**: Low
- **Throughput**: Very high

## Prerequisites

- **ROS 2** installed and configured
- **Ubuntu 20.04+** or compatible distribution
- **Internet connection** for package installation

## Dependencies

- **ROS 2 base installation** (installed separately)
- **APT package manager** (for package installation)

## Related Documentation

- [ROS 2 RMW Implementations](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html)
- [CycloneDDS](https://cyclonedds.io/)
- [FastRTPS](https://www.eprosima.com/index.php/products-all/eprosima-fast-rtps)
- [RTI Connext](https://www.rti.com/products/connext-dds)

This role provides flexible RMW implementation management for ROS 2 environments! 🤖