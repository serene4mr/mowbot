# DevContainer Configurations

This directory contains VS Code DevContainer configurations for Mowbot development.

## Available Configurations

### 1. CPU Development (`devel/`)
- **Container Name**: `mowbot_dev`
- **Base Image**: `ghcr.io/serene4mr/mowbot:main-dev-latest`
- **Use Case**: Standard development without GPU acceleration
- **Features**: Full ROS2 + build tools + development dependencies

### 2. CUDA Development (`devel-cuda/`)
- **Container Name**: `mowbot_dev_cuda`
- **Base Image**: `ghcr.io/serene4mr/mowbot:main-dev-cuda-latest`
- **Use Case**: Development with GPU acceleration (AI/ML, Gazebo, etc.)
- **Features**: Full ROS2 + CUDA 12.4 + TensorRT + development dependencies
- **Requirements**: NVIDIA GPU with compatible drivers

## How to Use

### Option 1: VS Code Command Palette
1. Open project in VS Code
2. `Ctrl+Shift+P` → "Dev Containers: Reopen in Container"
3. VS Code will show both configurations - choose your preferred one

### Option 2: Direct Configuration Selection
1. Copy the desired configuration to `.devcontainer/devcontainer.json`:
   ```bash
   # For CPU development
   cp .devcontainer/devel/devcontainer.json .devcontainer/devcontainer.json
   
   # For CUDA development  
   cp .devcontainer/devel-cuda/devcontainer.json .devcontainer/devcontainer.json
   ```
2. `Ctrl+Shift+P` → "Dev Containers: Rebuild and Reopen in Container"

## Development Workflow

Once inside the container:
```bash
# Build the project
colcon build

# Source the workspace
source install/setup.bash

# Run tests
colcon test

# Launch applications
ros2 launch mowbot_launch ...
```

## Features Included

- **User Setup**: Creates `mowbot` user with sudo privileges
- **Network**: Host networking for ROS2 communication
- **Hardware Access**: Device mounting for sensors/cameras
- **GUI Support**: X11 forwarding for RViz, Gazebo, etc.
- **VS Code Extensions**: Python, C++, CMake tools pre-installed

## Troubleshooting

### CUDA Container Issues
- Ensure NVIDIA drivers are installed on host
- Verify Docker has NVIDIA runtime: `docker run --rm --gpus all nvidia/cuda:12.4-runtime nvidia-smi`
- Check VS Code has "Dev Containers" extension installed

### Permission Issues
- Container creates `mowbot` user with UID/GID 1000
- If your host user has different ID, files may have permission issues
- Solution: Rebuild container or adjust USER_UID in Dockerfile
