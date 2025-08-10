# CUDA

This role installs [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) and configures GPU vendors for Vulkan, OpenGL, and OpenCL support. It follows [NVIDIA's official installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) and supports both x86_64 and ARM64 architectures.

## Features

- **Multi-architecture support**: Automatically detects and installs appropriate packages for x86_64 and ARM64 (sbsa)
- **Flexible installation modes**: Development (full toolkit) or runtime (minimal) installation
- **GPU vendor registration**: Configures Vulkan, OpenGL, and OpenCL support
- **Environment setup**: Automatically configures PATH and LD_LIBRARY_PATH
- **Optional driver installation**: Configurable CUDA driver installation
- **Installation verification**: Checks for successful CUDA installation

## Variables

| Name | Default | Required | Description |
|------|---------|----------|-------------|
| `cuda_version` | `"12.4"` | Yes | CUDA Toolkit version to install |
| `cuda_install_drivers` | `true` | No | Whether to install CUDA drivers |
| `install_devel` | `"y"` | No | Install development packages ("y") or runtime only (set by playbook) |
| `vulkan_icd_dir` | `"/etc/vulkan/icd.d"` | No | Vulkan ICD directory path |
| `opengl_vendor_dir` | `"/etc/glvnd/egl_vendor.d"` | No | OpenGL vendor directory path |
| `opencl_vendor_dir` | `"/etc/OpenCL/vendors"` | No | OpenCL vendor directory path |
| `cuda_bin_path` | `"/usr/local/cuda/bin"` | No | CUDA binary directory path |
| `cuda_lib_path` | `"/usr/local/cuda/lib64"` | No | CUDA library directory path |

## Installation Modes

### Development Mode (`install_devel: "y"`)
Installs complete CUDA development toolkit including:
- `cuda-command-line-tools` - CUDA command line utilities
- `cuda-minimal-build` - Basic build tools
- `libcusparse-dev` - Sparse linear algebra library (dev)
- `libcublas-dev` - Dense linear algebra library (dev)
- `libcurand-dev` - Random number generation library (dev)
- `cuda-nvml-dev` - NVIDIA Management Library (dev)
- `cuda-nvrtc-dev` - Runtime compilation library (dev)
- `cuda-nvprof` - NVIDIA profiler (x86_64 only)

### Runtime Mode (`install_devel != "y"`)
Installs minimal runtime libraries:
- `cuda-minimal-build` - Basic build tools
- `libcusparse` - Sparse linear algebra library (runtime)
- `libcublas` - Dense linear algebra library (runtime)
- `libcurand` - Random number generation library (runtime)

## GPU Vendor Support

The role automatically configures:

### Vulkan Support
- Downloads NVIDIA Vulkan ICD configuration
- Enables Vulkan GPU acceleration for graphics applications

### OpenGL Support  
- Downloads NVIDIA OpenGL vendor configuration
- Enables OpenGL GPU acceleration

### OpenCL Support
- Creates NVIDIA OpenCL vendor file
- Enables OpenCL GPU compute acceleration

## Environment Configuration

The role automatically adds to both user and system-wide bash configurations:
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

## Version Compatibility

**Current Default**: CUDA 12.4
- **Minimum Driver Version**: 550
- **Supported OS**: Ubuntu 22.04 (Jammy)
- **Supported Architectures**: x86_64, ARM64

## Usage Examples

### Basic CUDA installation
```yaml
- hosts: localhost
  roles:
    - cuda
```

### Runtime-only installation without drivers
```yaml
- hosts: localhost
  roles:
    - cuda
  vars:
    install_devel: "n"
    cuda_install_drivers: false
```

### Custom CUDA version
```yaml
- hosts: localhost
  roles:
    - cuda
  vars:
    cuda_version: "12.2"
```

## Manual Installation

If you need to install CUDA manually, follow these steps:

### 1. Install CUDA Toolkit
```bash
# Download environment variables
wget -O /tmp/amd64.env https://raw.githubusercontent.com/serene4mr/mowbot/main/amd64.env
source /tmp/amd64.env

# Add NVIDIA repository
os=ubuntu2204
arch=$(uname -m)
wget https://developer.download.nvidia.com/compute/cuda/repos/$os/$arch/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update

# Install CUDA toolkit
cuda_version_dashed=$(echo "12.4" | sed -e "s/\./-/g")
sudo apt-get -y install cuda-toolkit-${cuda_version_dashed}

# Install drivers (optional)
sudo apt-get install -y cuda-drivers
```

### 2. Configure Environment
```bash
# Add to ~/.bashrc
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 3. Setup GPU Vendors
```bash
# Create directories
sudo mkdir -p /etc/vulkan/icd.d /etc/glvnd/egl_vendor.d /etc/OpenCL/vendors

# Download Vulkan ICD
sudo wget https://gitlab.com/nvidia/container-images/vulkan/raw/dc389b0445c788901fda1d85be96fd1cb9410164/nvidia_icd.json \
  -O /etc/vulkan/icd.d/nvidia_icd.json

# Download OpenGL vendor
sudo wget https://gitlab.com/nvidia/container-images/opengl/raw/5191cf205d3e4bb1150091f9464499b076104354/glvnd/runtime/10_nvidia.json \
  -O /etc/glvnd/egl_vendor.d/10_nvidia.json

# Create OpenCL vendor
echo "libnvidia-opencl.so.1" | sudo tee /etc/OpenCL/vendors/nvidia.icd
```

### 4. Verify Installation
```bash
# Check CUDA compiler
nvcc --version

# Check NVIDIA driver
nvidia-smi

# Test CUDA sample (optional)
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery
```

## Troubleshooting

### Common Issues

**CUDA not found in PATH**
```bash
# Reload bash configuration
source ~/.bashrc
# Or check if CUDA is installed
ls -la /usr/local/cuda/bin/nvcc
```

**Driver compatibility issues**
```bash
# Check driver version
nvidia-smi
# Ensure driver version >= 550 for CUDA 12.4
```

**Architecture mismatch**
```bash
# Check architecture
uname -m
# Ensure correct CUDA packages for your architecture
```

**Permission issues**
```bash
# Fix GPU vendor file permissions
sudo chmod -R 644 /etc/vulkan/icd.d/ /etc/glvnd/egl_vendor.d/ /etc/OpenCL/vendors/
```

For more troubleshooting, see [NVIDIA's CUDA Installation Guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#troubleshooting).