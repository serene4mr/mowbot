# Docker Scripts

This directory contains utility scripts used during Docker image builds to optimize and clean the container environment.

## Scripts Overview

### 🧹 `cleanup_apt.sh`
**Purpose**: Cleans APT package cache and removes unnecessary files to reduce image size.

**Usage**:
```bash
./cleanup_apt.sh [apt_clean]
```

**Parameters**:
- `apt_clean` (optional): Set to `true` to also run `apt-get clean`

**What it does**:
- Removes automatically installed packages no longer needed (`apt-get autoremove`)
- Cleans `/var/lib/apt/lists/*` (package lists cache)
- Removes `$HOME/.cache` directory
- Optionally runs `apt-get clean` to remove downloaded package files

**Used in**: All Dockerfile stages to minimize image size

---

### 🗑️ `cleanup_system.sh`
**Purpose**: Aggressive system cleanup for production runtime images to minimize size.

**Usage**:
```bash
./cleanup_system.sh <lib_dir> <ros_distro>
```

**Parameters**:
- `lib_dir`: Architecture library directory (e.g., `x86_64`, `aarch64`)
- `ros_distro`: ROS distribution name (e.g., `humble`)

**What it does**:
- **Removes development files**: Static libraries (`.a`), object files (`.o`), headers (`.h`, `.hpp`)
- **Cleans build artifacts**: Ansible files, setup scripts, environment files
- **Removes development tools**: Include directories, documentation, GCC, JVM, LLVM
- **Cleans package sources**: CUDA, Docker, NVIDIA repository lists
- Calls `cleanup_apt.sh` automatically

**⚠️ Warning**: This is destructive and removes development tools. Only use in runtime images!

**Used in**: Runtime Dockerfile stages only

---

### 🔍 `resolve_rosdep_keys.sh`
**Purpose**: Resolves ROS package dependencies to system packages for installation.

**Usage**:
```bash
./resolve_rosdep_keys.sh <src_path> <ros_distro>
```

**Parameters**:
- `src_path`: Path to ROS workspace source directory
- `ros_distro`: ROS distribution name (e.g., `humble`)

**What it does**:
- Extracts rosdep keys from ROS packages in the source path
- Resolves keys to actual system package names
- Filters and sorts the package list
- Outputs clean package list for `apt-get install`

**Used in**: Development Dockerfile stages for dependency installation

---

### 🔨 `build_and_clean.sh`
**Purpose**: Builds ROS workspace with optimizations and cleans up build artifacts.

**Usage**:
```bash
./build_and_clean.sh <ccache_dir> <install_base> <colcon_build_args>
```

**Parameters**:
- `ccache_dir`: Path to ccache directory for compilation caching
- `install_base`: Installation directory for built packages
- `colcon_build_args`: Additional arguments for colcon build

**What it does**:
- **Pre-build**: Shows ccache directory size and statistics
- **Build**: Runs `colcon build` with optimizations:
  - Release mode compilation
  - Compiler cache (ccache) for faster rebuilds
  - Compile commands generation for IDE support
  - Merged install layout
- **Post-build**: Shows updated ccache statistics
- **Cleanup**: Removes build and log directories to save space

**Optimizations used**:
- `--mixin release`: Release mode compilation
- `--mixin compile-commands`: Generate `compile_commands.json`
- `--mixin ccache`: Use compiler cache
- `--merge-install`: Single install directory

**Used in**: Development Dockerfile stages for ROS workspace builds

---

## Script Integration

### Build Process Flow
```
1. Setup base system packages
2. resolve_rosdep_keys.sh → Get ROS dependencies
3. Install dependencies with apt
4. build_and_clean.sh → Build ROS workspace
5. cleanup_apt.sh → Clean package cache
6. [Runtime only] cleanup_system.sh → Aggressive cleanup
```

### Image Size Optimization
- **Development images**: Use `cleanup_apt.sh` only to preserve development tools
- **Runtime images**: Use both cleanup scripts for minimal size
- **ccache**: Shared between builds to speed up compilation

### Best Practices
- Always run cleanup scripts at the end of Dockerfile layers
- Use `--mount=type=cache` for apt and ccache directories
- Run scripts with appropriate parameters for the target architecture
- Test scripts individually before integrating into Dockerfiles

## Dependencies

### Required Tools
- `apt-get` (APT package manager)
- `rosdep` (ROS dependency manager)
- `colcon` (ROS build tool)
- `ccache` (Compiler cache)
- Standard Unix tools: `find`, `rm`, `du`, `sort`, `sed`, `grep`

### Environment Variables
Scripts may use these environment variables:
- `HOME`: User home directory
- `ROS_DISTRO`: ROS distribution name

## Troubleshooting

### Common Issues
- **Permission denied**: Ensure scripts have execute permissions (`chmod +x`)
- **Command not found**: Verify required tools are installed
- **Space issues**: Check available disk space before cleanup operations
- **Architecture mismatch**: Verify `lib_dir` parameter matches target architecture

### Debugging
- Add `set -x` to scripts for verbose output
- Check Docker build logs for script execution details
- Test scripts individually outside Docker for validation
