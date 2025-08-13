# Gazebo Host + ROS 2 Docker Setup Notes

## Goal
Use Gazebo Harmonic (8.9.0) running on the host and access it from a ROS 2 Humble Docker container.

## Prerequisites
- Gazebo Harmonic 8.9.0 installed on both host and container
- ROS 2 Humble in Docker container
- Matching Gazebo versions for protocol compatibility

## Host Side (Native)

### Start Gazebo Simulation
```bash
# Run simulation on host
gz sim your_world.sdf

# Optional: Set environment variables for consistency
export GZ_IP=127.0.0.1
export GZ_PARTITION=default
```

**Why host-side?** Native performance, full GPU access, no Docker overhead

## Container Side (Docker)

### 1. Dev Container Configuration (.devcontainer/devcontainer.json)
```json
{
    "name": "mowbot_dev",
    "image": "ghcr.io/serene4mr/mowbot:main-dev-latest",
    "runArgs": [
        "--name", "mowbot_dev",
        "--net", "host",          // Share host's network namespace
        "--ipc", "host",          // Enable shared memory communication 
        "--privileged",           // Allow hardware access
        "-v", "/dev:/dev",        // Mount device files
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix",  // X11 forwarding
        "-e", "DISPLAY"           // Display environment
    ],
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-python.python"
            ]
        }
    }
}
```

### 2. Environment Variables (Inside Container)
```bash
# Set these every time you want to access host Gazebo
export GZ_IP=127.0.0.1          # Container's network address
export GZ_PARTITION=default     # Logical network partition name
export GZ_RELAY=127.0.0.1      # Relay discovery messages to host
export GZ_VERBOSE=1             # Enable debug output (optional)
```

**Critical:** `GZ_RELAY=127.0.0.1` is the key - it forwards discovery messages to host

### 3. Test Connection
```bash
# Verify Gazebo topic access
gz topic -l

# Should show topics like:
# /clock
# /world/shapes/clock
# /world/shapes/stats
# /world/shapes/pose/info
# etc.
```

### 4. Bridge to ROS 2
```bash
# Bridge specific topics
ros2 run ros_gz_bridge parameter_bridge /world/shapes/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Check ROS 2 topics
ros2 topic list | grep shapes
ros2 topic echo /world/shapes/clock
```

## Technical Explanation

### Why This Works
- **Problem**: Gazebo Transport uses UDP multicast for discovery, which doesn't work across Docker boundaries
- **Solution**: `GZ_RELAY` creates relay mechanism for discovery messages
- **Network**: `--network=host` shares network interfaces
- **Memory**: `--ipc=host` enables shared memory for Gazebo Transport efficiency

### Key Components
| Component | Location | Purpose |
|-----------|----------|---------|
| `gz sim` | Host | Native simulation execution |
| `--network=host` | Container | Network sharing |
| `--ipc=host` | Container | Memory sharing |
| `GZ_RELAY` | Container | Discovery forwarding |
| `GZ_PARTITION` | Both | Shared logical network |
| `ros_gz_bridge` | Container | Gazebo ↔ ROS 2 translation |

## Quick Start Commands

### Terminal 1 (Host)
```bash
gz sim shapes.sdf
```

### Terminal 2 (Container)
```bash
export GZ_IP=127.0.0.1
export GZ_PARTITION=default
export GZ_RELAY=127.0.0.1
export GZ_VERBOSE=1

# Test connection
gz topic -l

# Bridge to ROS 2
ros2 run ros_gz_bridge parameter_bridge /world/shapes/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
```

### Terminal 3 (Container - ROS 2)
```bash
ros2 topic list
ros2 topic echo /world/shapes/clock
```

## Troubleshooting

### If `gz topic -l` shows no topics:
1. Check Gazebo is running on host
2. Verify environment variables are set
3. Ensure `--network=host` and `--ipc=host` in container config
4. Check Gazebo versions match exactly

### If ROS 2 bridge fails:
1. Verify `gz topic -l` works first
2. Check topic name spelling
3. Ensure message type mapping is correct
4. Try with `--verbose` flag

## Success Indicators
✅ `gz topic -l` shows host's Gazebo topics  
✅ Connection callbacks appear in verbose output  
✅ ROS 2 topics appear after bridging  
✅ Can echo ROS 2 topics successfully  

## Benefits
- Native Gazebo performance on host
- Isolated ROS 2 development in container
- No GPU passthrough complexity
- Multiple containers can connect to same simulation
- Reproducible development environment