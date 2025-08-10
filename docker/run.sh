#!/usr/bin/env bash

set -e

# Function to print help message
print_help() {
    echo "Usage: run.sh [OPTIONS] [COMMAND]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  --image         Specify the image to run (default: dev)"
    echo "  --version       Specify the version tag (default: latest)"
    echo "  --module        Specify the module (default: main)"
    echo "  --cuda          Use CUDA-enabled image"
    echo "  --runtime       Use runtime image instead of development"
    echo "  --name          Container name (default: mowbot-container)"
    echo "  --host          Use host networking"
    echo "  --privileged    Run in privileged mode"
    echo "  --volume        Mount additional volume (format: host:container)"
    echo "  --env           Set environment variable (format: KEY=VALUE)"
    echo "  --gpus          GPU access (default: all)"
    echo "  --interactive   Run in interactive mode (default: true)"
    echo "  --detached      Run in detached mode"
    echo ""
    echo "Examples:"
    echo "  ./run.sh                    # Run main development environment"
    echo "  ./run.sh --cuda             # Run main CUDA development environment"
    echo "  ./run.sh --runtime          # Run main production runtime"
    echo "  ./run.sh --version v1.0.0   # Run specific version"
    echo "  ./run.sh --module navigation # Run navigation module (future)"
    echo "  ./run.sh --volume /path:/workspace  # Mount workspace"
    echo "  ./run.sh bash               # Run with specific command"
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

# Default values
IMAGE_TYPE="dev"
VERSION="latest"
CONTAINER_NAME="mowbot-container"
USE_CUDA=false
USE_RUNTIME=false
USE_HOST_NETWORK=false
USE_PRIVILEGED=false
USE_GPU=false
INTERACTIVE=true
DETACHED=false
ADDITIONAL_VOLUMES=()
ADDITIONAL_ENV_VARS=()

# Parse arguments
parse_arguments() {
    while [ "$1" != "" ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 0
            ;;
        --image)
            IMAGE_TYPE="$2"
            shift
            ;;
        --version)
            VERSION="$2"
            shift
            ;;
        --module)
            MODULE="$2"
            shift
            ;;
        --cuda)
            USE_CUDA=true
            ;;
        --runtime)
            USE_RUNTIME=true
            ;;
        --name)
            CONTAINER_NAME="$2"
            shift
            ;;
        --host)
            USE_HOST_NETWORK=true
            ;;
        --privileged)
            USE_PRIVILEGED=true
            ;;
        --volume)
            ADDITIONAL_VOLUMES+=("$2")
            shift
            ;;
        --env)
            ADDITIONAL_ENV_VARS+=("$2")
            shift
            ;;
        --gpus)
            USE_GPU=true
            GPU_OPTION="$2"
            shift
            ;;
        --interactive)
            INTERACTIVE=true
            DETACHED=false
            ;;
        --detached)
            DETACHED=true
            INTERACTIVE=false
            ;;
        -*)
            echo "Unknown option: $1"
            print_help
            exit 1
            ;;
        *)
            # Treat as command
            COMMAND="$*"
            break
            ;;
        esac
        shift
    done
}

# Determine image name
get_image_name() {
    local base_name="ghcr.io/serene4mr/mowbot"
    
    # Default to core module
    local module=${MODULE:-core}
    local image_name=""
    
    if [ "$USE_RUNTIME" = "true" ]; then
        if [ "$USE_CUDA" = "true" ]; then
            image_name="${base_name}:${module}-runtime-cuda"
        else
            image_name="${base_name}:${module}-runtime"
        fi
    else
        if [ "$USE_CUDA" = "true" ]; then
            image_name="${base_name}:${module}-dev-cuda"
        else
            image_name="${base_name}:${module}-dev"
        fi
    fi
    
    # Add version if not latest
    if [ "$VERSION" != "latest" ]; then
        image_name="${image_name}-${VERSION}"
    fi
    
    echo "$image_name"
}

# Build docker run command
build_run_command() {
    local image_name=$(get_image_name)
    local cmd="docker run"
    
    # Container name
    cmd="$cmd --name $CONTAINER_NAME"
    
    # Remove container if it exists
    cmd="$cmd --rm"
    
    # Interactive mode
    if [ "$INTERACTIVE" = "true" ]; then
        cmd="$cmd -it"
    fi
    
    # Detached mode
    if [ "$DETACHED" = "true" ]; then
        cmd="$cmd -d"
    fi
    
    # Host networking
    if [ "$USE_HOST_NETWORK" = "true" ]; then
        cmd="$cmd --net host"
    fi
    
    # Privileged mode
    if [ "$USE_PRIVILEGED" = "true" ]; then
        cmd="$cmd --privileged"
    fi
    
    # GPU access
    if [ "$USE_CUDA" = "true" ] || [ "$USE_GPU" = "true" ]; then
        if [ -n "$GPU_OPTION" ]; then
            cmd="$cmd --gpus $GPU_OPTION"
        else
            cmd="$cmd --gpus all"
        fi
    fi
    
    # Standard volumes
    cmd="$cmd -v $WORKSPACE_ROOT:/workspace"
    cmd="$cmd -v /tmp/.X11-unix:/tmp/.X11-unix"
    
    # Additional volumes
    for volume in "${ADDITIONAL_VOLUMES[@]}"; do
        cmd="$cmd -v $volume"
    done
    
    # Environment variables
    cmd="$cmd -e DISPLAY=\$DISPLAY"
    cmd="$cmd -e LOCAL_UID=\$(id -u)"
    cmd="$cmd -e LOCAL_GID=\$(id -g)"
    cmd="$cmd -e LOCAL_USER=\$(whoami)"
    cmd="$cmd -e LOCAL_GROUP=\$(id -gn)"
    
    # Additional environment variables
    for env_var in "${ADDITIONAL_ENV_VARS[@]}"; do
        cmd="$cmd -e $env_var"
    done
    
    # Image and command
    cmd="$cmd $image_name"
    
    if [ -n "$COMMAND" ]; then
        cmd="$cmd $COMMAND"
    fi
    
    echo "$cmd"
}

# Main execution
main() {
    parse_arguments "$@"
    
    # Check if image exists locally, pull if not
    local image_name=$(get_image_name)
    if ! docker image inspect "$image_name" >/dev/null 2>&1; then
        echo "Image $image_name not found locally. Pulling..."
        docker pull "$image_name"
    fi
    
    # Build and execute run command
    local run_cmd=$(build_run_command)
    echo "Running: $run_cmd"
    eval "$run_cmd"
}

main "$@"
