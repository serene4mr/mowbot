# Build Tools

This role installs essential build tools and configures compilation caching for faster Mowbot builds.

## Tools Installed

- **ccache** - Compiler cache for faster rebuilds
- **build-essential** - GCC, g++, make, and other essential build tools
- **cmake** - Cross-platform build system
- **ninja-build** - Fast build system
- **pkg-config** - Package configuration tool

## Configuration

### ccache Setup
- **Cache directory**: `/var/tmp/ccache` (configurable via `ccache_dir`)
- **Max cache size**: `5G` (configurable via `ccache_max_size`)
- **Compiler wrappers**: Automatically configured for CC and CXX

### Environment Variables
The role adds these exports to `~/.bashrc`:
```bash
export CCACHE_DIR=/var/tmp/ccache
export CC=/usr/lib/ccache/gcc
export CXX=/usr/lib/ccache/g++
```

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `ccache_dir` | `/var/tmp/ccache` | Directory for ccache storage |
| `ccache_max_size` | `5G` | Maximum cache size |

## Manual Installation

```bash
# Update package lists
sudo apt-get update

# Install build tools
sudo apt-get install -y ccache build-essential cmake ninja-build pkg-config

# Configure ccache
export CCACHE_DIR=/var/tmp/ccache
mkdir -p $CCACHE_DIR
ccache -M 5G

# Add to ~/.bashrc
echo 'export CCACHE_DIR=/var/tmp/ccache' >> ~/.bashrc
echo 'export CC=/usr/lib/ccache/gcc' >> ~/.bashrc
echo 'export CXX=/usr/lib/ccache/g++' >> ~/.bashrc
```