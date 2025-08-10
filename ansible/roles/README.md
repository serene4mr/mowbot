# Mowbot Ansible Roles

This directory contains Ansible roles for setting up the mowbot development environment. Each role is designed to be modular, configurable, and production-ready.

## Available Roles

### Core Development Environment

#### 🔧 **build_tools**
- **Purpose**: Install build tools and configure ccache for faster compilation
- **Features**: Build essentials, CMake, Ninja, pkg-config, ccache configuration
- **Documentation**: [build_tools/README.md](build_tools/README.md) (52 lines)

#### 🌍 **geographiclib**
- **Purpose**: Install GeographicLib for geospatial calculations
- **Features**: Smart installation, configurable versions, verification
- **Documentation**: [geographiclib/README.md](geographiclib/README.md) (68 lines)

#### 🤖 **rmw_implementation**
- **Purpose**: Install and configure ROS 2 RMW implementations
- **Features**: Multiple RMW options, environment configuration, performance optimization
- **Documentation**: [rmw_implementation/README.md](rmw_implementation/README.md) (236 lines)

### GPU and AI Acceleration

#### 🚀 **cuda**
- **Purpose**: Install NVIDIA CUDA Toolkit and configure GPU support
- **Features**: CUDA installation, GPU vendor support, environment configuration
- **Documentation**: [cuda/README.md](cuda/README.md) (201 lines)

#### 🧠 **tensorrt**
- **Purpose**: Install TensorRT and cuDNN for deep learning inference
- **Features**: TensorRT runtime/dev packages, cuDNN support, package protection
- **Documentation**: [tensorrt/README.md](tensorrt/README.md) (282 lines)

#### 🐳 **nvidia_container_toolkit**
- **Purpose**: Install NVIDIA Container Toolkit for GPU access in containers
- **Features**: Docker runtime configuration, GPU testing, multi-GPU support
- **Documentation**: [nvidia_container_toolkit/README.md](nvidia_container_toolkit/README.md) (207 lines)

#### 🖥️ **kisak_mesa**
- **Purpose**: Install Kisak Mesa graphics libraries for RViz2 compatibility
- **Features**: RViz2 black-screen fix, OpenGL compatibility, container graphics
- **Documentation**: [kisak_mesa/README.md](kisak_mesa/README.md) (155 lines)

### Development Tools

#### 🛠️ **dev_tools**
- **Purpose**: Install comprehensive development tools and utilities
- **Features**: ROS tools, system tools, Python tools, code quality tools
- **Documentation**: [dev_tools/README.md](dev_tools/README.md) (155 lines)

### Artifacts and Assets

#### 📦 **artifacts**
- **Purpose**: Download and manage ONNX models and other artifacts
- **Features**: Model downloads, asset management, version control, checksum verification
- **Documentation**: [artifacts/README.md](artifacts/README.md) (203 lines)

## Role Categories

### **Essential Roles** (Always Installed)
- `build_tools` - Required for compilation
- `geographiclib` - Required for geospatial calculations
- `rmw_implementation` - Required for ROS 2 communication

### **GPU Acceleration Roles** (Conditional)
- `cuda` - NVIDIA CUDA Toolkit
- `tensorrt` - Deep learning inference
- `nvidia_container_toolkit` - Container GPU access
- `kisak_mesa` - Graphics compatibility

### **Development Tools** (Optional)
- `dev_tools` - Comprehensive development utilities

### **Assets** (Optional)
- `artifacts` - ONNX models and other files

## Usage Examples

### Basic Installation
```yaml
- hosts: localhost
  roles:
    - build_tools
    - geographiclib
    - rmw_implementation
```

### Full GPU Development Environment
```yaml
- hosts: localhost
  roles:
    - build_tools
    - geographiclib
    - rmw_implementation
    - cuda
    - tensorrt
    - nvidia_container_toolkit
    - kisak_mesa
    - dev_tools
    - artifacts
```

### Container Development
```yaml
- hosts: localhost
  roles:
    - build_tools
    - geographiclib
    - rmw_implementation
    - nvidia_container_toolkit
    - kisak_mesa
```

## Role Dependencies

### Prerequisites
- **Ubuntu 22.04** (or compatible distribution)
- **APT package manager** (for package installation)
- **Internet connection** (for downloads and repositories)

### GPU Requirements
- **NVIDIA GPU** (for CUDA, TensorRT, Container Toolkit)
- **NVIDIA drivers** (installed separately)
- **Docker** (for Container Toolkit)

### ROS 2 Requirements
- **ROS 2** (installed separately)
- **Python 3.8+** (for ROS 2 tools)

## Configuration

Each role provides configurable variables in their `defaults/main.yaml` files. Common configuration patterns:

### Version Management
```yaml
# CUDA and TensorRT versions
cuda_version: "12.4"
tensorrt_version: "8.6.1.6"
cudnn_version: "8.9.7.29"

# ROS 2 configuration
rosdistro: "humble"
rmw_implementation: "rmw_cyclonedds_cpp"
```

### Installation Options
```yaml
# Development tools
tensorrt_install_devel: true
cuda_install_devel: true

# Verification
tensorrt_verify_installation: true
cuda_verify_installation: true
```

## Testing

### Individual Role Testing
```bash
# Test specific role
ansible-playbook -i localhost, -c local test-role.yaml

# Test with custom variables
ansible-playbook -i localhost, -c local test-role.yaml -e "cuda_version=12.4"
```

### Full Environment Testing
```bash
# Test complete setup
ansible-playbook playbooks/main.yaml

# Test with all options enabled
ansible-playbook playbooks/main.yaml \
  -e prompt_install_nvidia=y \
  -e prompt_download_artifacts=y \
  -e install_devel=y
```

## Troubleshooting

### Common Issues
1. **Permission errors** - Ensure proper file ownership
2. **Package conflicts** - Check version compatibility
3. **GPU detection** - Verify NVIDIA drivers and hardware
4. **Network issues** - Check repository access and downloads

### Role-Specific Issues
- See individual role README files for detailed troubleshooting
- Each role includes comprehensive error handling and verification
- Most roles provide manual installation instructions as fallback

## Best Practices

### Role Usage
- **Use variables** for configuration instead of hardcoding values
- **Enable verification** for production deployments
- **Test roles individually** before combining them
- **Review role documentation** for specific requirements

### Environment Setup
- **Install roles in order** of dependencies
- **Verify prerequisites** before role execution
- **Use consistent versions** across related roles
- **Enable package protection** for critical components

## Contributing

### Adding New Roles
1. Create role directory with standard structure
2. Include comprehensive documentation
3. Add proper error handling and verification
4. Test with multiple configurations
5. Update this README with role information

### Role Standards
- **YAML syntax validation** - All files must be valid YAML
- **Variable usage** - Use variables instead of hardcoded values
- **Error handling** - Include verification and status messages
- **Documentation** - Comprehensive README with examples
- **Testing** - Include validation and testing procedures

## Related Documentation

- [Ansible Best Practices](https://docs.ansible.com/ansible/latest/user_guide/playbooks_best_practices.html)
- [ROS 2 Installation](https://docs.ros.org/en/humble/Installation.html)
- [NVIDIA CUDA Installation](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)
- [TensorRT Installation](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html)

This roles collection provides a complete, modular, and production-ready setup for mowbot development environments! 🚀
