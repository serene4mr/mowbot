# Mowbot

A ROS2-based autonomous mowing robot platform with comprehensive navigation, perception, and control capabilities.

## Quick Start with Docker

### Development Environment
```bash
# Build the development image
./docker/build.sh --devel-only

# Run the development container
./docker/run.sh

# Build the project inside the container
colcon build
```

### Available Docker Images
- **`ghcr.io/serene4mr/mowbot:main-dev`** - Development environment with full build tools
- **`ghcr.io/serene4mr/mowbot:main-runtime`** - Production runtime environment
- **CUDA variants available** - Add `-cuda` suffix for GPU acceleration

## Documentation

- **[Docker Guide](docker/README.md)** - Complete containerization documentation
- **[Setup Guide](setup-dev-env.sh)** - Native development environment setup

## Project Structure

- `src/` - ROS2 packages and source code
- `docker/` - Container build and run scripts
- `ansible/` - System setup and dependency management
- `build/` & `install/` - Colcon build artifacts

## Dependencies

- **ROS2 Humble** - Robot Operating System
- **CUDA 12.4** (optional) - GPU acceleration
- **Ubuntu 22.04** - Supported OS

## Getting Started

1. **Clone with submodules:**
   ```bash
   git clone --recursive https://github.com/serene4mr/mowbot.git
   cd mowbot
   ```

2. **Choose your development approach:**
   - **Docker (Recommended):** `./docker/run.sh`
   - **Native:** `./setup-dev-env.sh`

3. **Build the project:**
   ```bash
   colcon build
   source install/setup.bash
   ```

## License

This project is licensed under the terms specified in the individual package directories.