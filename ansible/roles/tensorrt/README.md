# TensorRT

This role installs and configures NVIDIA TensorRT and cuDNN for high-performance deep learning inference acceleration.

## Purpose

TensorRT is NVIDIA's high-performance deep learning inference library that optimizes neural networks for production deployment. This role provides:

- **TensorRT runtime** - Optimized inference engine
- **cuDNN support** - Deep learning primitives
- **Development tools** - Headers and development libraries
- **Version management** - Specific version control
- **Package protection** - Prevents unwanted upgrades

This is essential for:
- **Deep learning inference** in production environments
- **Computer vision applications** with GPU acceleration
- **ROS 2 perception nodes** requiring neural network inference
- **Autonomous driving** perception and planning systems
- **Real-time AI applications** with low latency requirements

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `tensorrt_version` | `"8.6.1.6"` | TensorRT version to install |
| `tensorrt_cuda_version` | `"12.0"` | CUDA version compatibility |
| `cudnn_version` | `"8.9.7.29"` | cuDNN version to install |
| `tensorrt_install_devel` | `true` | Whether to install development packages |
| `tensorrt_verify_installation` | `true` | Whether to verify installation success |
| `tensorrt_hold_packages` | `true` | Whether to hold packages from upgrades |

## Installed Packages

### Runtime Packages
- **libcudnn8** - cuDNN runtime library
- **libnvinfer10** - TensorRT inference runtime
- **libnvinfer-plugin10** - TensorRT plugin runtime
- **libnvonnxparsers10** - ONNX parser runtime

### Development Packages (optional)
- **libcudnn8-dev** - cuDNN development headers
- **libnvinfer-dev** - TensorRT development headers
- **libnvinfer-plugin-dev** - TensorRT plugin development
- **libnvinfer-headers-dev** - TensorRT header files
- **libnvinfer-headers-plugin-dev** - TensorRT plugin headers
- **libnvonnxparsers-dev** - ONNX parser development

## Installation Process

The role performs the following steps:

1. **Update package cache** - Ensures latest package information
2. **Install runtime packages** - Installs TensorRT and cuDNN runtime libraries
3. **Install development packages** - Installs development headers (optional)
4. **Hold packages** - Prevents automatic upgrades to maintain compatibility
5. **Verify installation** - Checks if packages are properly installed
6. **Display status** - Provides feedback on installation and configuration

## Usage Examples

### Basic installation (runtime only)
```yaml
- hosts: localhost
  roles:
    - tensorrt
  vars:
    tensorrt_install_devel: false
```

### Full installation with development packages
```yaml
- hosts: localhost
  roles:
    - tensorrt
  vars:
    tensorrt_install_devel: true
```

### Custom versions
```yaml
- hosts: localhost
  roles:
    - tensorrt
  vars:
    tensorrt_version: "8.5.3.1"
    cudnn_version: "8.7.0.84"
    tensorrt_cuda_version: "11.8"
```

### Disable package holding
```yaml
- hosts: localhost
  roles:
    - tensorrt
  vars:
    tensorrt_hold_packages: false
```

### Disable verification
```yaml
- hosts: localhost
  roles:
    - tensorrt
  vars:
    tensorrt_verify_installation: false
```

## Manual Installation

If you need to install manually:

```bash
# Set environment variables
export tensorrt_version="8.6.1.6"
export cudnn_version="8.9.7.29"

# Update package lists
sudo apt update

# Install runtime packages
sudo apt install -y \
  libcudnn8=${cudnn_version} \
  libnvinfer10=${tensorrt_version} \
  libnvinfer-plugin10=${tensorrt_version} \
  libnvonnxparsers10=${tensorrt_version}

# Install development packages (optional)
sudo apt install -y \
  libcudnn8-dev=${cudnn_version} \
  libnvinfer-dev=${tensorrt_version} \
  libnvinfer-plugin-dev=${tensorrt_version} \
  libnvinfer-headers-dev=${tensorrt_version} \
  libnvinfer-headers-plugin-dev=${tensorrt_version} \
  libnvonnxparsers-dev=${tensorrt_version}

# Hold packages to prevent upgrades
sudo apt-mark hold \
  libcudnn8 \
  libnvinfer10 \
  libnvinfer-plugin10 \
  libnvonnxparsers10 \
  libcudnn8-dev \
  libnvinfer-dev \
  libnvinfer-plugin-dev \
  libnvinfer-headers-dev \
  libnvinfer-headers-plugin-dev \
  libnvonnxparsers-dev

# Verify installation
dpkg -l | grep -E "(libnvinfer10|libcudnn8)"
```

## Testing

### Verify installation
```bash
# Check runtime packages
dpkg -l | grep -E "(libnvinfer10|libcudnn8)"

# Check development packages
dpkg -l | grep -E "(libnvinfer-dev|libcudnn8-dev)"

# Check package hold status
apt-mark showhold | grep -E "(nvinfer|cudnn)"
```

### Test TensorRT functionality
```bash
# Check TensorRT version
python3 -c "import tensorrt as trt; print(trt.__version__)"

# Test cuDNN
python3 -c "import torch; print(torch.backends.cudnn.version())"

# Run TensorRT sample (if available)
/usr/src/tensorrt/samples/sampleMNIST/sampleMNIST
```

### Test with ROS 2
```bash
# Check if ROS 2 can find TensorRT
ros2 pkg list | grep tensorrt

# Test perception node (if available)
ros2 run perception_node inference_test
```

## Troubleshooting

### Package not found
```bash
# Check available TensorRT packages
apt search libnvinfer

# Check CUDA compatibility
nvidia-smi
nvcc --version

# Check repository configuration
apt-cache policy libnvinfer10
```

### Version conflicts
```bash
# Check installed versions
dpkg -l | grep -E "(nvinfer|cudnn)"

# Check for conflicts
apt-cache policy libnvinfer10

# Remove conflicting packages if needed
sudo apt remove conflicting-package
```

### Development headers missing
```bash
# Check if dev packages are installed
dpkg -l | grep -E "(libnvinfer-dev|libcudnn8-dev)"

# Install missing dev packages
sudo apt install libnvinfer-dev libcudnn8-dev
```

### Package upgrades
```bash
# Check hold status
apt-mark showhold

# Temporarily unhold for upgrade
sudo apt-mark unhold package-name
sudo apt upgrade package-name
sudo apt-mark hold package-name
```

## Performance Considerations

### TensorRT Optimization
- **FP16/INT8 quantization** - Reduced precision for speed
- **Kernel fusion** - Combined operations for efficiency
- **Memory optimization** - Optimized memory allocation
- **Dynamic batching** - Efficient batch processing

### cuDNN Features
- **Convolution algorithms** - Optimized convolution operations
- **Memory management** - Efficient GPU memory usage
- **Multi-GPU support** - Distributed training and inference
- **Automatic tuning** - Performance optimization

## Prerequisites

- **NVIDIA GPU** with supported drivers
- **CUDA Toolkit** installed and configured
- **Ubuntu 20.04+** or compatible distribution
- **Internet connection** for package installation

## Dependencies

- **NVIDIA drivers** (installed separately)
- **CUDA Toolkit** (installed separately)
- **APT package manager** (for package installation)

## Version Compatibility

### TensorRT 8.6.1.6
- **CUDA**: 11.8, 12.0, 12.1, 12.2
- **cuDNN**: 8.7.0.84, 8.8.0.131, 8.9.0.131, 8.9.2.26, 8.9.3.29, 8.9.4.25, 8.9.5.29, 8.9.6.50, 8.9.7.29
- **Python**: 3.8, 3.9, 3.10, 3.11

### cuDNN 8.9.7.29
- **CUDA**: 11.8, 12.0, 12.1, 12.2
- **TensorRT**: 8.5.3.1, 8.6.1.6
- **Architectures**: x86_64, ARM64

## Related Documentation

- [NVIDIA TensorRT Installation Guide](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html)
- [TensorRT Developer Guide](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html)
- [cuDNN Documentation](https://docs.nvidia.com/deeplearning/cudnn/)
- [ROS 2 Perception](https://github.com/ros-perception)

This role provides production-ready TensorRT and cuDNN installation for AI applications! 🚀
