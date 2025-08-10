# Mowbot Development Environment Collection

This Ansible collection provides comprehensive roles for setting up the mowbot development environment, including ROS 2, GPU acceleration, AI tools, and development utilities.

## Overview

The `mowbot.dev_env` collection is designed to automate the setup of a complete robotics development environment with:

- **ROS 2 Humble** with optimized RMW middleware
- **NVIDIA GPU acceleration** (CUDA, TensorRT, Container Toolkit)
- **Development tools** for robotics and AI development
- **Build optimization** with ccache and essential tools
- **Geospatial libraries** for navigation and mapping
- **Graphics compatibility** for RViz2 and visualization tools

## Collection Structure

```
ansible/
├── galaxy.yml                    # Collection metadata
├── README.md                     # This file
├── ansible-galaxy-requirements.yaml  # Collection dependencies
├── playbooks/
│   ├── main.yaml                 # Main setup playbook
│   └── README.md                 # Playbook documentation
└── roles/
    ├── README.md                 # Roles overview
    ├── artifacts/                # ONNX models and assets
    ├── build_tools/              # Build essentials and ccache
    ├── cuda/                     # NVIDIA CUDA Toolkit
    ├── dev_tools/                # Development utilities
    ├── geographiclib/            # Geospatial calculations
    ├── kisak_mesa/               # Graphics compatibility
    ├── nvidia_container_toolkit/ # Container GPU access
    ├── rmw_implementation/       # ROS 2 communication
    └── tensorrt/                 # Deep learning inference
```

## Installation

### Method 1: Using setup-dev-env.sh (Recommended)

The easiest way to use this collection is through the provided setup script:

```bash
# Interactive installation
./setup-dev-env.sh

# Non-interactive installation
./setup-dev-env.sh -y

# Custom options
./setup-dev-env.sh --no-nvidia --runtime --data-dir /custom/path
```

### Method 2: Manual Collection Installation

```bash
# Install the collection locally
cd ansible
ansible-galaxy collection install -f -r ansible-galaxy-requirements.yaml

# Run the playbook
ansible-playbook mowbot.dev_env.main
```

### Method 3: Direct Playbook Execution

```bash
# Run directly without collection installation
cd ansible
ansible-playbook playbooks/main.yaml
```

## Available Roles

### Core Development Environment

#### 🔧 **build_tools**
- **Purpose**: Install build tools and configure ccache for faster compilation
- **Features**: Build essentials, CMake, Ninja, pkg-config, ccache configuration
- **Documentation**: [roles/build_tools/README.md](roles/build_tools/README.md)

#### 🌍 **geographiclib**
- **Purpose**: Install GeographicLib for geospatial calculations
- **Features**: Smart installation, configurable versions, verification
- **Documentation**: [roles/geographiclib/README.md](roles/geographiclib/README.md)

#### 🤖 **rmw_implementation**
- **Purpose**: Install and configure ROS 2 RMW implementations
- **Features**: Multiple RMW options, environment configuration, performance optimization
- **Documentation**: [roles/rmw_implementation/README.md](roles/rmw_implementation/README.md)

### GPU and AI Acceleration

#### 🚀 **cuda**
- **Purpose**: Install NVIDIA CUDA Toolkit and configure GPU support
- **Features**: CUDA installation, GPU vendor support, environment configuration
- **Documentation**: [roles/cuda/README.md](roles/cuda/README.md)

#### 🧠 **tensorrt**
- **Purpose**: Install TensorRT and cuDNN for deep learning inference
- **Features**: TensorRT runtime/dev packages, cuDNN support, package protection
- **Documentation**: [roles/tensorrt/README.md](roles/tensorrt/README.md)

#### 🐳 **nvidia_container_toolkit**
- **Purpose**: Install NVIDIA Container Toolkit for GPU access in containers
- **Features**: Docker runtime configuration, GPU testing, multi-GPU support
- **Documentation**: [roles/nvidia_container_toolkit/README.md](roles/nvidia_container_toolkit/README.md)

#### 🖥️ **kisak_mesa**
- **Purpose**: Install Kisak Mesa graphics libraries for RViz2 compatibility
- **Features**: RViz2 black-screen fix, OpenGL compatibility, container graphics
- **Documentation**: [roles/kisak_mesa/README.md](roles/kisak_mesa/README.md)

### Development Tools

#### 🛠️ **dev_tools**
- **Purpose**: Install comprehensive development tools and utilities
- **Features**: ROS tools, system tools, Python tools, code quality tools
- **Documentation**: [roles/dev_tools/README.md](roles/dev_tools/README.md)

### Artifacts and Assets

#### 📦 **artifacts**
- **Purpose**: Download and manage ONNX models and other artifacts
- **Features**: Model downloads, asset management, version control, checksum verification
- **Documentation**: [roles/artifacts/README.md](roles/artifacts/README.md)

## Configuration

### Environment Variables

The collection uses environment files for configuration:

- **amd64.env** - Configuration for x86_64 systems
- **arm64.env** - Configuration for ARM64 systems

### Key Variables

```yaml
# ROS 2 Configuration
rosdistro: "humble"
rmw_implementation: "rmw_cyclonedds_cpp"

# NVIDIA Versions
cuda_version: "12.4"
cudnn_version: "8.9.7.29"
tensorrt_version: "8.6.1.6"

# Installation Options
prompt_install_nvidia: "N"
prompt_download_artifacts: "N"
install_devel: "y"
```

### Custom Configuration

```bash
# Override variables
ansible-playbook mowbot.dev_env.main \
  -e "cuda_version=11.8" \
  -e "rosdistro=iron" \
  -e "prompt_install_nvidia=y"
```

## Usage Examples

### Basic Installation

```bash
# Interactive setup
./setup-dev-env.sh

# Non-interactive setup
./setup-dev-env.sh -y
```

### GPU Development Environment

```bash
# Install with NVIDIA support
./setup-dev-env.sh -y

# Or manually
ansible-playbook mowbot.dev_env.main \
  -e "prompt_install_nvidia=y" \
  -e "install_devel=y"
```

### Runtime Only Installation

```bash
# Install runtime components only
./setup-dev-env.sh --runtime

# Or manually
ansible-playbook mowbot.dev_env.main \
  -e "install_devel=N" \
  -e "prompt_install_nvidia=y"
```

### Custom Data Directory

```bash
# Set custom artifacts directory
./setup-dev-env.sh --data-dir /opt/mowbot/data

# Or manually
ansible-playbook mowbot.dev_env.main \
  -e "data_dir=/opt/mowbot/data"
```

## Interactive Prompts

The main playbook includes interactive prompts for:

1. **NVIDIA Libraries**: Install CUDA, cuDNN, and TensorRT
2. **Artifacts Download**: Download ONNX models and configuration files
3. **Development Tools**: Install recommended development utilities

## Testing

### Validation

```bash
# Syntax check
ansible-playbook mowbot.dev_env.main --check

# Dry run
ansible-playbook mowbot.dev_env.main --check --diff

# Verbose execution
ansible-playbook mowbot.dev_env.main -v
```

### Individual Role Testing

```bash
# Test specific role
ansible-playbook -i localhost, -c local test-role.yaml

# Test with custom variables
ansible-playbook mowbot.dev_env.main \
  -e "cuda_version=12.4" \
  --check
```

## Troubleshooting

### Common Issues

1. **Permission errors** - Ensure proper file ownership
2. **Package conflicts** - Check version compatibility
3. **GPU detection** - Verify NVIDIA drivers and hardware
4. **Network issues** - Check repository access and downloads
5. **Role not found** - Verify collection installation

### Debug Information

```bash
# Enable debug output
ansible-playbook mowbot.dev_env.main -vvv

# Check collection installation
ansible-galaxy collection list | grep mowbot

# Check role paths
ansible-playbook mowbot.dev_env.main --list-roles
```

### Role-Specific Issues

- See individual role README files for detailed troubleshooting
- Each role includes comprehensive error handling and verification
- Most roles provide manual installation instructions as fallback

## Prerequisites

### System Requirements
- **Ubuntu 22.04** (or compatible distribution)
- **APT package manager** (for package installation)
- **Internet connection** (for downloads and repositories)
- **Ansible 2.9+** (for playbook execution)

### GPU Requirements
- **NVIDIA GPU** (for CUDA, TensorRT, Container Toolkit)
- **NVIDIA drivers** (installed separately)
- **Docker** (for Container Toolkit)

### ROS 2 Requirements
- **ROS 2** (installed separately)
- **Python 3.8+** (for ROS 2 tools)

## Best Practices

### Collection Usage
- **Use setup-dev-env.sh** for standard installations
- **Review prompts carefully** before answering
- **Use non-interactive mode** for automation
- **Test with --check** before actual execution

### Environment Setup
- **Install roles in order** of dependencies
- **Verify prerequisites** before collection execution
- **Use consistent versions** across related roles
- **Enable package protection** for critical components

### Configuration Management
- **Use environment files** for system-specific configuration
- **Document custom configurations**
- **Version control** your customizations
- **Test configurations** in isolated environments

## Contributing

### Adding New Roles
1. Create role directory with standard structure
2. Include comprehensive documentation
3. Add proper error handling and verification
4. Test with multiple configurations
5. Update collection documentation

### Role Standards
- **YAML syntax validation** - All files must be valid YAML
- **Variable usage** - Use variables instead of hardcoded values
- **Error handling** - Include verification and status messages
- **Documentation** - Comprehensive README with examples
- **Testing** - Include validation and testing procedures

## Related Documentation

- [Collection Metadata](galaxy.yml)
- [Playbook Documentation](playbooks/README.md)
- [Roles Overview](roles/README.md)
- [Individual Role Documentation](roles/*/README.md)
- [Setup Script Documentation](../../README.md)
- [Ansible Best Practices](https://docs.ansible.com/ansible/latest/user_guide/playbooks_best_practices.html)

## License

This collection is licensed under the MIT License.

This collection provides a complete, modular, and production-ready setup for mowbot development environments! 🚀