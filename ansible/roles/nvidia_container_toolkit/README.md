# NVIDIA Container Toolkit

This role installs and configures the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) to enable GPU acceleration in Docker containers.

## Purpose

The NVIDIA Container Toolkit provides a collection of tools and libraries that allow users to build and run GPU-accelerated containers. It enables:

- **GPU access in containers** - Direct access to NVIDIA GPUs
- **CUDA applications** - Run CUDA workloads in containers
- **Deep learning frameworks** - TensorFlow, PyTorch, etc. with GPU support
- **ROS 2 GPU acceleration** - GPU-accelerated robotics applications
- **Multi-GPU support** - Access to multiple GPUs in containers

This is essential for:
- **Machine learning workloads** in containers
- **Computer vision applications** with GPU acceleration
- **ROS 2 GPU-accelerated nodes** in robotics
- **CUDA development** in containerized environments

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `nvidia_container_toolkit_gpg_url` | `"https://nvidia.github.io/libnvidia-container/gpgkey"` | GPG key URL |
| `nvidia_container_toolkit_keyring` | `"/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg"` | Keyring path |
| `nvidia_container_toolkit_repo_url` | `"https://nvidia.github.io/libnvidia-container/stable/deb"` | Repository URL |
| `nvidia_container_toolkit_repo_filename` | `"nvidia-container-toolkit"` | Repository filename |
| `nvidia_container_toolkit_packages` | `["nvidia-container-toolkit"]` | Packages to install |
| `nvidia_container_toolkit_runtime` | `"docker"` | Container runtime to configure |
| `nvidia_container_toolkit_restart_docker` | `true` | Whether to restart Docker daemon |
| `nvidia_container_toolkit_verify_installation` | `true` | Whether to verify installation |
| `nvidia_container_toolkit_test_installation` | `true` | Whether to test with CUDA container |

## Installation Process

The role performs the following steps:

1. **Detect system architecture** - Automatically determines CPU architecture
2. **Add GPG key** - Imports NVIDIA's GPG key for package verification
3. **Add repository** - Configures NVIDIA's package repository
4. **Install toolkit** - Installs the NVIDIA Container Toolkit package
5. **Configure runtime** - Configures Docker to use NVIDIA runtime
6. **Restart Docker** - Restarts Docker daemon to apply changes
7. **Verify installation** - Checks if toolkit is properly installed
8. **Test functionality** - Runs a test CUDA container to verify GPU access

## Usage Examples

### Basic installation
```yaml
- hosts: localhost
  roles:
    - nvidia_container_toolkit
```

### Custom repository
```yaml
- hosts: localhost
  roles:
    - nvidia_container_toolkit
  vars:
    nvidia_container_toolkit_repo_url: "https://custom.nvidia.com/libnvidia-container/stable/deb"
```

### Disable testing
```yaml
- hosts: localhost
  roles:
    - nvidia_container_toolkit
  vars:
    nvidia_container_toolkit_test_installation: false
```

### Custom runtime
```yaml
- hosts: localhost
  roles:
    - nvidia_container_toolkit
  vars:
    nvidia_container_toolkit_runtime: "containerd"
```

## Manual Installation

If you need to install manually:

```bash
# Add NVIDIA container toolkit GPG key
sudo apt-key adv --fetch-keys https://nvidia.github.io/libnvidia-container/gpgkey
sudo gpg --no-default-keyring --keyring /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg --import /etc/apt/trusted.gpg

# Add NVIDIA container toolkit repository
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/$(dpkg --print-architecture) /" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package lists
sudo apt-get update

# Install NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker runtime
sudo nvidia-ctk runtime configure --runtime=docker

# Restart Docker daemon
sudo systemctl restart docker

# Verify installation
nvidia-ctk --version

# Test with CUDA container
sudo docker run --rm --gpus all nvcr.io/nvidia/cuda:12.3.1-runtime-ubuntu20.04 nvidia-smi
```

## Testing

### Verify installation
```bash
# Check toolkit version
nvidia-ctk --version

# Check Docker runtime configuration
cat /etc/docker/daemon.json
```

### Test GPU access
```bash
# Run CUDA container with GPU access
docker run --rm --gpus all nvcr.io/nvidia/cuda:12.3.1-runtime-ubuntu20.04 nvidia-smi

# Expected output shows GPU information:
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 545.23.08    Driver Version: 545.23.08    CUDA Version: 12.3.1   |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |                               |                      |               MIG M. |
# |===============================+======================+======================|
# |   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
# | N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
# |                               |                      |                  N/A |
# +-------------------------------+----------------------+----------------------+
```

## Troubleshooting

### Toolkit not found
```bash
# Check if package is installed
dpkg -l | grep nvidia-container-toolkit

# Check if repository is added
apt-cache policy nvidia-container-toolkit
```

### Docker runtime not configured
```bash
# Check Docker daemon configuration
cat /etc/docker/daemon.json

# Manually configure runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### GPU not accessible in container
```bash
# Check if NVIDIA drivers are installed
nvidia-smi

# Check if Docker has GPU access
docker run --rm --gpus all nvidia/cuda:12.3.1-runtime-ubuntu20.04 nvidia-smi

# Check Docker daemon logs
sudo journalctl -u docker.service
```

### Permission denied
```bash
# Check if user is in docker group
groups $USER

# Add user to docker group
sudo usermod -aG docker $USER
# Log out and back in, or run: newgrp docker
```

## Prerequisites

- **NVIDIA GPU** with supported drivers
- **Docker** installed and running
- **Ubuntu 20.04+** or compatible distribution
- **Internet connection** for repository access

## Dependencies

- **NVIDIA drivers** (installed separately)
- **Docker** (installed separately)
- **systemd** (for service management)

## Related Documentation

- [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [Docker GPU Support](https://docs.docker.com/config/containers/resource_constraints/#gpu)
- [CUDA Container Images](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/cuda)

This role provides seamless GPU acceleration for containerized applications! 🚀
