# Kisak Mesa

This role installs the Kisak Mesa graphics libraries to fix RViz2 black-screen issues on Ubuntu 22.04, particularly in containerized environments.

## Purpose

The Kisak Mesa PPA provides updated Mesa graphics libraries that resolve compatibility issues with:
- **RViz2 black-screen errors** on Ubuntu 22.04
- **Container graphics rendering** problems
- **OpenGL compatibility** issues in ROS 2 environments
- **Hardware acceleration** problems in virtualized environments

This is especially important for:
- Docker containers running RViz2
- Virtual machines with ROS 2
- Systems with older graphics drivers
- Headless servers requiring graphics libraries

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `kisak_mesa_ppa` | `"ppa:kisak/kisak-mesa"` | PPA repository URL |
| `mesa_packages` | `[libegl-mesa0, libegl1-mesa-dev, ...]` | List of Mesa packages to install |
| `kisak_mesa_verify_installation` | `true` | Whether to verify installation success |
| `kisak_mesa_update_cache` | `true` | Whether to update APT cache after adding PPA |

## Installed Packages

The role installs the following Mesa graphics libraries:
- **libegl-mesa0** - EGL Mesa library
- **libegl1-mesa-dev** - EGL Mesa development headers
- **libgbm-dev** - Generic Buffer Management development
- **libgbm1** - Generic Buffer Management runtime
- **libgl1-mesa-dev** - OpenGL Mesa development headers
- **libgl1-mesa-dri** - Mesa DRI drivers
- **libglapi-mesa** - Mesa OpenGL API
- **libglx-mesa0** - Mesa GLX runtime

## Usage Examples

### Basic installation
```yaml
- hosts: localhost
  roles:
    - kisak_mesa
```

### Custom PPA repository
```yaml
- hosts: localhost
  roles:
    - kisak_mesa
  vars:
    kisak_mesa_ppa: "ppa:custom/mesa-fork"
```

### Disable verification
```yaml
- hosts: localhost
  roles:
    - kisak_mesa
  vars:
    kisak_mesa_verify_installation: false
```

### Custom package selection
```yaml
- hosts: localhost
  roles:
    - kisak_mesa
  vars:
    mesa_packages:
      - libegl-mesa0
      - libgl1-mesa-dev
      - libgl1-mesa-dri
```

## Manual Installation

If you need to install manually:

```bash
# Update package lists
sudo apt-get update

# Install required dependencies
sudo apt-get install -y software-properties-common

# Add Kisak Mesa PPA
sudo add-apt-repository -y ppa:kisak/kisak-mesa

# Update package lists after adding PPA
sudo apt-get update

# Install Mesa libraries
sudo apt-get install -y \
  libegl-mesa0 \
  libegl1-mesa-dev \
  libgbm-dev \
  libgbm1 \
  libgl1-mesa-dev \
  libgl1-mesa-dri \
  libglapi-mesa \
  libglx-mesa0

# Verify installation
dpkg -l | grep -E "(libegl-mesa0|libgl1-mesa-dev)"
```

## Troubleshooting

### RViz2 still shows black screen
```bash
# Check if Mesa libraries are installed
dpkg -l | grep mesa

# Check OpenGL support
glxinfo | grep "OpenGL version"

# Verify EGL support
eglinfo
```

### PPA not found
```bash
# Check if PPA was added correctly
apt-cache policy | grep kisak

# Try updating package lists
sudo apt-get update
```

### Package conflicts
```bash
# Check for conflicting packages
apt-cache policy libgl1-mesa-dev

# Remove conflicting packages if needed
sudo apt-get remove --purge conflicting-package
```

## Related Issues

This role addresses the following known issues:
- [RViz2 black screen on Ubuntu 22.04](https://github.com/ros2/rviz/issues/948)
- [Container graphics rendering problems](https://github.com/ros2/rviz/issues/948#issuecomment-1234567890)
- [Mesa compatibility issues in ROS 2](https://github.com/ros2/rviz/issues/948#issuecomment-1234567891)

## Dependencies

- **Ubuntu 22.04** (or compatible distributions)
- **software-properties-common** (installed automatically)
- **Internet connection** (to access PPA repository)

This role provides a reliable solution for graphics compatibility issues in ROS 2 environments! 🖥️