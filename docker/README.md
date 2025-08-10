# Containerized workloads for Mowbot

These containers are offered to simplify the development and deployment of Mowbot and its dependencies. This directory contains scripts to build and run the containers.

## Available Images

### Main Module (Full Version)
- `ghcr.io/serene4mr/mowbot:main-dev` - Main development environment with full build tools
- `ghcr.io/serene4mr/mowbot:main-dev-cuda` - Main development environment with CUDA support
- `ghcr.io/serene4mr/mowbot:main-runtime` - Main production runtime environment
- `ghcr.io/serene4mr/mowbot:main-runtime-cuda` - Main production runtime with CUDA support

### Base Images
- `ghcr.io/serene4mr/mowbot:base` - Base image with ROS2 and dependencies
- `ghcr.io/serene4mr/mowbot:base-cuda` - Base image with CUDA runtime

### Future Modules (Planned)
- `ghcr.io/serene4mr/mowbot:navigation-dev` - Navigation module development
- `ghcr.io/serene4mr/mowbot:navigation-runtime` - Navigation module runtime
- `ghcr.io/serene4mr/mowbot:perception-dev` - Perception module development
- `ghcr.io/serene4mr/mowbot:perception-runtime` - Perception module runtime

## Version Support
All images support both `latest` and versioned tags:
- `ghcr.io/serene4mr/mowbot:main-dev` (latest)
- `ghcr.io/serene4mr/mowbot:main-dev-v1.0.0` (specific version)

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
./docker/build.sh --target main-dev-cuda
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
# Pull latest main development image
docker pull ghcr.io/serene4mr/mowbot:main-dev

# Run main development container
docker run -it --rm \
  -v $(pwd):/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  ghcr.io/serene4mr/mowbot:main-dev

# Run with CUDA
docker run -it --rm \
  --gpus all \
  -v $(pwd):/workspace \
  ghcr.io/serene4mr/mowbot:main-dev-cuda
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

- `main-dev` (maps to `dev` tag)
- `main-runtime` (maps to `runtime` tag)

By generating only the package list files and copying them to the subsequent stages, the dependency packages will not be reinstalled during the container build process unless the dependency packages change.

### `main-dev` → `ghcr.io/serene4mr/mowbot:main-dev`

Development environment with full build tools, debugging capabilities, and development dependencies.

### `main-runtime` → `ghcr.io/serene4mr/mowbot:main-runtime`

Production runtime environment optimized for deployment with minimal size and security hardening.