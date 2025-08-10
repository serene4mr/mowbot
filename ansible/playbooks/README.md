# Ansible Playbooks

This directory contains Ansible playbooks for setting up the mowbot development environment with comprehensive role management and interactive configuration.

## Playbooks

### `main.yaml`
**Purpose**: Complete mowbot development environment setup with modular role execution

**Features**:
- **Interactive prompts** for NVIDIA libraries, artifacts download, and development tools
- **OS verification** (Ubuntu 22.04 only)
- **Modular role execution** based on user choices
- **Comprehensive role coverage** including all 9 available roles
- **Smart dependency management** with proper execution order
- **Detailed configuration display** with default values

## Usage

### Interactive Installation
```bash
cd ansible
ansible-playbook playbooks/main.yaml
```

### Non-Interactive Installation
For automated/CI environments, override the prompt variables:

```bash
# Full installation with all components
ansible-playbook playbooks/main.yaml \
  -e prompt_install_nvidia=y \
  -e prompt_download_artifacts=y \
  -e install_devel=y

# Minimal installation (core only)
ansible-playbook playbooks/main.yaml \
  -e prompt_install_nvidia=n \
  -e prompt_download_artifacts=n \
  -e install_devel=n

# GPU development environment
ansible-playbook playbooks/main.yaml \
  -e prompt_install_nvidia=y \
  -e prompt_download_artifacts=n \
  -e install_devel=y
```

## Interactive Prompts

### 1. NVIDIA Libraries Installation
```
[Warning] Some Mowbot components depend on the CUDA, cuDNN and TensorRT NVIDIA libraries which have end-user license agreements that should be reviewed before installation.
Install NVIDIA libraries? [y/N]
```
- **Purpose**: Install GPU acceleration components
- **Default**: No (N)
- **Installs**: CUDA, TensorRT, NVIDIA Container Toolkit, Kisak Mesa

### 2. Artifacts Download
```
[Warning] Should the ONNX model files and other artifacts be downloaded alongside setting up the development environment.
Download artifacts? [y/N]
```
- **Purpose**: Download ONNX models and configuration files
- **Default**: No (N)
- **Note**: Currently no artifacts are available

### 3. Development Tools
```
[Warning] Do you want to install recommended development tools for Mowbot? [y/N]
```
- **Purpose**: Install comprehensive development utilities
- **Default**: Yes (y)
- **Installs**: ROS tools, system tools, Python tools, code quality tools

## Role Execution Order

### Core Development Environment (Always Installed)
1. **build_tools** - Build essentials and ccache configuration
2. **geographiclib** - Geospatial calculations library
3. **rmw_implementation** - ROS 2 communication middleware

### GPU Acceleration (Conditional)
4. **cuda** - NVIDIA CUDA Toolkit
5. **tensorrt** - Deep learning inference
6. **nvidia_container_toolkit** - Container GPU access
7. **kisak_mesa** - Graphics compatibility

### Development Tools (Conditional)
8. **dev_tools** - Comprehensive development utilities

### Artifacts and Assets (Conditional)
9. **artifacts** - ONNX models and configuration files

## Configuration Variables

### Default Values
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
# Custom ROS 2 distribution
ansible-playbook playbooks/main.yaml -e "rosdistro=iron"

# Custom CUDA version
ansible-playbook playbooks/main.yaml -e "cuda_version=11.8"

# Custom RMW implementation
ansible-playbook playbooks/main.yaml -e "rmw_implementation=rmw_fastrtps_cpp"
```

## Pre-Tasks

### OS Verification
- **Check**: Ubuntu 22.04 compatibility
- **Action**: Fail if unsupported OS detected
- **Message**: "Only Ubuntu 22.04 is supported for this branch."

### Configuration Display
- **Purpose**: Show all configuration values with defaults
- **Includes**: ROS 2, RMW, CUDA, cuDNN, TensorRT versions
- **Includes**: Installation choices for NVIDIA, artifacts, dev tools

### NVIDIA Warning
- **Trigger**: When NVIDIA libraries are skipped
- **Action**: 10-second pause with warning message
- **Purpose**: Remind user to install manually if needed

## Role Dependencies

### Prerequisites
- **Ubuntu 22.04** (or compatible distribution)
- **APT package manager** (for package installation)
- **Internet connection** (for downloads and repositories)
- **Ansible** (for playbook execution)

### GPU Requirements
- **NVIDIA GPU** (for CUDA, TensorRT, Container Toolkit)
- **NVIDIA drivers** (installed separately)
- **Docker** (for Container Toolkit)

### ROS 2 Requirements
- **ROS 2** (installed separately)
- **Python 3.8+** (for ROS 2 tools)

## Testing

### Validation
```bash
# Syntax check
ansible-playbook playbooks/main.yaml --check

# Dry run
ansible-playbook playbooks/main.yaml --check --diff

# Verbose execution
ansible-playbook playbooks/main.yaml -v
```

### Role Testing
```bash
# Test specific role
ansible-playbook -i localhost, -c local test-role.yaml

# Test with custom variables
ansible-playbook playbooks/main.yaml -e "cuda_version=12.4" --check
```

## Troubleshooting

### Common Issues
1. **Permission errors** - Ensure proper file ownership
2. **Package conflicts** - Check version compatibility
3. **GPU detection** - Verify NVIDIA drivers and hardware
4. **Network issues** - Check repository access and downloads
5. **Role not found** - Verify role paths and namespaces

### Debug Information
```bash
# Enable debug output
ansible-playbook playbooks/main.yaml -vvv

# Check role paths
ansible-playbook playbooks/main.yaml --list-roles

# Check task execution
ansible-playbook playbooks/main.yaml --list-tasks
```

### Role-Specific Issues
- See individual role README files for detailed troubleshooting
- Each role includes comprehensive error handling and verification
- Most roles provide manual installation instructions as fallback

## Best Practices

### Playbook Usage
- **Review prompts carefully** before answering
- **Use non-interactive mode** for automation
- **Test with --check** before actual execution
- **Monitor role execution** for any failures

### Environment Setup
- **Install roles in order** of dependencies
- **Verify prerequisites** before playbook execution
- **Use consistent versions** across related roles
- **Enable package protection** for critical components

### Configuration Management
- **Use variables** for customization
- **Document custom configurations**
- **Version control** your customizations
- **Test configurations** in isolated environments

## Related Documentation

- [Ansible Playbook Best Practices](https://docs.ansible.com/ansible/latest/user_guide/playbooks_best_practices.html)
- [Role Documentation](../roles/README.md)
- [Individual Role READMEs](../roles/*/README.md)
- [Mowbot Setup Guide](../../README.md)

This playbook provides a complete, interactive, and production-ready setup for mowbot development environments! 🚀
