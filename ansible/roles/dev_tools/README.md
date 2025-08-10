# Development Tools

This role installs optional development tools for mowbot development, including ROS visualization tools, system utilities, and code quality tools.

## Features

- **ROS Development Tools**: PlotJuggler, RQt, RViz2 for visualization and debugging
- **System Utilities**: Git LFS, curl, wget, vim, htop, tree, and other essentials
- **Python Development**: pip, venv, development headers, and quality tools
- **Code Quality**: clang-format, cppcheck, valgrind for C++ development
- **Configurable Installation**: Enable/disable tool categories as needed

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `rosdistro` | `humble` | ROS 2 distribution name |
| `ros_tools_packages` | `[plotjuggler, rqt, rqt-common-plugins, rviz2]` | ROS development packages |
| `system_tools_packages` | `[git-lfs, curl, wget, vim, htop, tree, jq, unzip]` | System utility packages |
| `python_tools_packages` | `[python3-pip, python3-venv, python3-dev]` | Python development packages |
| `code_quality_packages` | `[clang-format, cppcheck, valgrind]` | Code quality packages |
| `install_ros_tools` | `true` | Install ROS visualization and debugging tools |
| `install_system_tools` | `true` | Install system development utilities |
| `install_python_tools` | `true` | Install Python development tools |
| `install_code_quality_tools` | `true` | Install code quality and analysis tools |

## Installed Tools

### ROS Development Tools (when `install_ros_tools: true`)
- **PlotJuggler** - Real-time data visualization
- **RQt** - Qt-based GUI framework for ROS
- **RQt Common Plugins** - Standard RQt plugins
- **RViz2** - 3D visualization tool for ROS

### System Development Utilities (when `install_system_tools: true`)
- **git-lfs** - Git Large File Storage
- **curl/wget** - HTTP clients for downloading
- **vim** - Text editor
- **htop** - Interactive process viewer
- **tree** - Directory structure display
- **jq** - JSON processor
- **unzip** - Archive extraction

### Python Development (when `install_python_tools: true`)
- **python3-pip** - Python package installer
- **python3-venv** - Virtual environment support
- **python3-dev** - Python development headers
- **pre-commit** - Git pre-commit hooks
- **black** - Python code formatter
- **flake8** - Python linting tool
- **mypy** - Python type checker

### Code Quality Tools (when `install_code_quality_tools: true`)
- **clang-format** - C++ code formatter
- **cppcheck** - Static analysis for C++
- **valgrind** - Memory debugging and profiling

## Usage Examples

### Default installation (all tools)
```yaml
- hosts: localhost
  roles:
    - dev_tools
```

### Install only ROS tools
```yaml
- hosts: localhost
  roles:
    - dev_tools
  vars:
    install_system_tools: false
    install_python_tools: false
    install_code_quality_tools: false
```

### Custom ROS distribution
```yaml
- hosts: localhost
  roles:
    - dev_tools
  vars:
    rosdistro: iron
```

### Minimal installation
```yaml
- hosts: localhost
  roles:
    - dev_tools
  vars:
    install_ros_tools: false
    install_code_quality_tools: false
```

## Manual Installation

### ROS Development Tools
```bash
# Install ROS visualization tools
sudo apt-get update
sudo apt-get install -y \
  ros-humble-plotjuggler-ros \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  ros-humble-rviz2
```

### System Development Utilities
```bash
# Install system utilities
sudo apt-get install -y \
  git-lfs curl wget vim htop tree jq unzip

# Setup git-lfs
git lfs install --skip-smudge
```

### Python Development Tools
```bash
# Install Python development
sudo apt-get install -y \
  python3-pip python3-venv python3-dev

# Install Python quality tools
pip3 install pre-commit black flake8 mypy
```

### Code Quality Tools
```bash
# Install C++ quality tools
sudo apt-get install -y \
  clang-format cppcheck valgrind
```

## Integration with Development Workflow

### Pre-commit Setup
After installation, setup pre-commit in your project:
```bash
cd your-project
pre-commit install
```

### ROS Tools Usage
- **PlotJuggler**: `ros2 run plotjuggler plotjuggler`
- **RQt**: `rqt`
- **RViz2**: `rviz2`

### Code Formatting
- **C++ formatting**: `clang-format -i *.cpp *.hpp`
- **Python formatting**: `black .`
- **Static analysis**: `cppcheck --enable=all .`

This role provides a complete development environment for efficient mowbot development! 🚀