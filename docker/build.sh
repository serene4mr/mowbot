#!/usr/bin/env bash

set -e

# Function to print help message
print_help() {
    echo "Usage: build.sh [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --no-cuda       Disable CUDA support"
    echo "  --platform      Specify the platform (default: current platform)"
    echo "  --devel-only    Build development image only (targets: mowbot-devel)"
    echo "  --target        Specify the target image (default: mowbot or mowbot-devel if --devel-only is set)"
    echo "  --version       Specify the version tag (default: latest)"
    echo ""
    echo "Available targets:"
    echo "  mowbot-devel    → ghcr.io/serene4mr/mowbot:core-dev"
    echo "  mowbot          → ghcr.io/serene4mr/mowbot:core-runtime"
    echo "  mowbot-devel-cuda → ghcr.io/serene4mr/mowbot:core-dev-cuda"
    echo "  mowbot-cuda     → ghcr.io/serene4mr/mowbot:core-runtime-cuda"
    echo ""
    echo "Note: The --platform option should be one of 'linux/amd64' or 'linux/arm64'."
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Parse arguments
parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 1
            ;;
        --no-cuda)
            option_no_cuda=true
            ;;
        --platform)
            option_platform="$2"
            shift
            ;;
        --devel-only)
            option_devel_only=true
            ;;
        --target)
            option_target="$2"
            shift
            ;;
        --version)
            option_version="$2"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        esac
        shift
    done
}

# Set CUDA options
set_cuda_options() {
    if [ "$option_no_cuda" = "true" ]; then
        setup_args="--no-nvidia"
        image_name_suffix=""
    else
        image_name_suffix="-cuda"
    fi
}

# Set build options
set_build_options() {
    if [ -n "$option_target" ]; then
        target="$option_target"
        image_name_suffix=""
    else
        if [ "$option_devel_only" = "true" ]; then
            target="mowbot-devel"
        else
            target="mowbot"
        fi
    fi
}

# Set version
set_version() {
    if [ -n "$option_version" ]; then
        VERSION="$option_version"
    else
        VERSION="latest"
    fi
}

# Set platform
set_platform() {
    if [ -n "$option_platform" ]; then
        platform="$option_platform"
    else
        platform="linux/amd64"
        if [ "$(uname -m)" = "aarch64" ]; then
            platform="linux/arm64"
        fi
    fi
}

# Set arch lib dir
set_arch_lib_dir() {
    if [ "$platform" = "linux/arm64" ]; then
        lib_dir="aarch64"
    elif [ "$platform" = "linux/amd64" ]; then
        lib_dir="x86_64"
    else
        echo "Unsupported platform: $platform"
        exit 1
    fi
}

# Load env
load_env() {
    source "$WORKSPACE_ROOT/amd64.env"
    if [ "$platform" = "linux/arm64" ]; then
        source "$WORKSPACE_ROOT/arm64.env"
    fi
}

# Clone repositories
clone_repositories() {
    cd "$WORKSPACE_ROOT"
    if [ ! -d "src" ]; then
        mkdir -p src
        vcs import src <mowbot.repos
    else
        echo "Source directory already exists. Updating repositories..."
        vcs import src <mowbot.repos
        vcs pull src
    fi
}

# Build images
build_images() {
    # https://github.com/docker/buildx/issues/484
    export BUILDKIT_STEP_LOG_MAX_SIZE=10000000

    # Set version (can be overridden by environment variable)
    VERSION=${VERSION:-latest}

    echo "Building images for platform: $platform"
    echo "ROS distro: $rosdistro"
    echo "Base image: $base_image"
    echo "Setup args: $setup_args"
    echo "Lib dir: $lib_dir"
    echo "Image name suffix: $image_name_suffix"
    echo "Target: $target"
    echo "Version: $VERSION"

    set -x
    docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake-base.hcl" \
        --set "*.context=$WORKSPACE_ROOT" \
        --set "*.ssh=default" \
        --set "*.platform=$platform" \
        --set "*.args.ROS_DISTRO=$rosdistro" \
        --set "*.args.BASE_IMAGE=$base_image" \
        --set "*.args.SETUP_ARGS=$setup_args" \
        --set "*.args.LIB_DIR=$lib_dir" \
        --set "base.tags=ghcr.io/serene4mr/mowbot:base" \
        --set "base-cuda.tags=ghcr.io/serene4mr/mowbot:base-cuda"
    docker buildx bake --load --progress=plain -f "$SCRIPT_DIR/docker-bake.hcl" -f "$SCRIPT_DIR/docker-bake-cuda.hcl" \
        --set "*.context=$WORKSPACE_ROOT" \
        --set "*.ssh=default" \
        --set "*.platform=$platform" \
        --set "*.args.ROS_DISTRO=$rosdistro" \
        --set "*.args.MOWBOT_BASE_IMAGE=$mowbot_base_image" \
        --set "*.args.MOWBOT_BASE_CUDA_IMAGE=$mowbot_base_cuda_image" \
        --set "*.args.SETUP_ARGS=$setup_args" \
        --set "*.args.LIB_DIR=$lib_dir" \
        --set "mowbot-devel.tags=ghcr.io/serene4mr/mowbot:core-dev,ghcr.io/serene4mr/mowbot:core-dev-$VERSION" \
        --set "mowbot.tags=ghcr.io/serene4mr/mowbot:core-runtime,ghcr.io/serene4mr/mowbot:core-runtime-$VERSION" \
        --set "mowbot-devel-cuda.tags=ghcr.io/serene4mr/mowbot:core-dev-cuda,ghcr.io/serene4mr/mowbot:core-dev-cuda-$VERSION" \
        --set "mowbot-cuda.tags=ghcr.io/serene4mr/mowbot:core-runtime-cuda,ghcr.io/serene4mr/mowbot:core-runtime-cuda-$VERSION" \
        "$target$image_name_suffix"
    set +x
}

# Remove dangling images
remove_dangling_images() {
    docker image prune -f
}

# Main script execution
parse_arguments "$@"
set_cuda_options
set_build_options
set_version
set_platform
set_arch_lib_dir
load_env
clone_repositories
build_images
remove_dangling_images