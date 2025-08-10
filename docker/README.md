# Containerized workloads for Mowbot

These containers are offered to simplify the development and deployment of Mowbot and its dependencies. This directory contains scripts to build and run the containers.

## Available Images

### Core Module (Current)
- `ghcr.io/amr4serene/mowbot:core-dev` - Core development environment with full build tools
- `ghcr.io/amr4serene/mowbot:core-dev-cuda` - Core development environment with CUDA support
- `ghcr.io/amr4serene/mowbot:core-runtime` - Core production runtime environment
- `ghcr.io/amr4serene/mowbot:core-runtime-cuda` - Core production runtime with CUDA support

### Base Images
- `ghcr.io/amr4serene/mowbot:base` - Base image with ROS2 and dependencies
- `ghcr.io/amr4serene/mowbot:base-cuda` - Base image with CUDA runtime

### Future Modules (Planned)
- `ghcr.io/amr4serene/mowbot:navigation-dev` - Navigation module development
- `ghcr.io/amr4serene/mowbot:navigation-runtime` - Navigation module runtime
- `ghcr.io/amr4serene/mowbot:perception-dev` - Perception module development
- `ghcr.io/amr4serene/mowbot:perception-runtime` - Perception module runtime

## Version Support
All images support both `latest` and versioned tags:
- `ghcr.io/amr4serene/mowbot:dev` (latest)
- `ghcr.io/amr4serene/mowbot:dev-v1.0.0` (specific version)

## Usage

### Building Images

```bash
# Build development environment
./docker/build.sh --devel-only

# Build with CUDA support
./docker/build.sh --devel-only

# Build specific version
./docker/build.sh --devel-only --version v1.0.0

# Build for ARM64 platform
./docker/build.sh --platform linux/arm64 --devel-only

# Build runtime environment
./docker/build.sh

# Build specific target
./docker/build.sh --target mowbot-devel-cuda
```

### Running Containers

```bash
# Run core development environment
./docker/run.sh

# Run with CUDA support
./docker/run.sh --cuda

# Run production runtime
./docker/run.sh --runtime

# Run specific version
./docker/run.sh --version v1.0.0

# Run navigation module (future)
./docker/run.sh --module navigation

# Run with custom command
./docker/run.sh bash

# Run with additional volumes
./docker/run.sh --volume /path/to/data:/data

# Run in detached mode
./docker/run.sh --detached
```

### Docker Commands

```bash
# Pull latest core development image
docker pull ghcr.io/amr4serene/mowbot:core-dev

# Run core development container
docker run -it --rm \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ghcr.io/amr4serene/mowbot:core-dev

# Run with CUDA
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace \
  ghcr.io/amr4serene/mowbot:core-dev-cuda
```

## Multi-stage Dockerfile structure

### `$BASE_IMAGE`

This is a base image of this Dockerfile. [`ros:humble-ros-base-jammy`](https://hub.docker.com/_/ros/tags?page=&page_size=&ordering=&name=humble-ros-base-jammy) will be given.

### `$MOWBOT_BASE_IMAGE` (from Dockerfile.base)

This stage performs only the basic setup required for all Mowbot images.

### `$MOWBOT_BASE_CUDA_IMAGE` (from Dockerfile.base)

This stage is built on top of `$MOWBOT_BASE_IMAGE` and adds the CUDA runtime environment and artifacts.

### `rosdep-depend`

The ROS dependency package list files will be generated.
These files will be used in the subsequent stages:

- `mowbot-devel` (maps to `dev` tag)
- `mowbot` (maps to `runtime` tag)

By generating only the package list files and copying them to the subsequent stages, the dependency packages will not be reinstalled during the container build process unless the dependency packages change.

### `mowbot-devel` → `ghcr.io/amr4serene/mowbot:dev`

Development environment with full build tools, debugging capabilities, and development dependencies.

### `mowbot` → `ghcr.io/amr4serene/mowbot:runtime`

Production runtime environment optimized for deployment with minimal size and security hardening.