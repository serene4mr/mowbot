#!/usr/bin/env bash
# Run a mowbot container (images: runtime-<platform>-<ver>, devel-<platform>-<ver>).

set -e

print_help() {
    echo "Usage: run.sh [OPTIONS] [COMMAND]"
    echo "Options:"
    echo "  --help -h       This help"
    echo "  --version       Version tag (default: latest)"
    echo "  --platform      amd64 (default) or jetson"
    echo "  --runtime       Runtime image instead of devel"
    echo "  --cuda          Pass --gpus all to docker run"
    echo "  --name          Container name (default: mowbot-container)"
    echo "  --host          Host networking"
    echo "  --privileged    Privileged mode"
    echo "  --volume        Extra mount host:container"
    echo "  --env           Extra env KEY=VALUE"
    echo "  --gpus          GPU selector (implies GPU; default all)"
    echo ""
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."

VERSION="latest"
CONTAINER_NAME="mowbot-container"
USE_RUNTIME=false
USE_CUDA=false
USE_HOST_NETWORK=false
USE_PRIVILEGED=false
USE_GPU=false
PLATFORM="amd64"
INTERACTIVE=true
DETACHED=false
ADDITIONAL_VOLUMES=()
ADDITIONAL_ENV_VARS=()
GPU_OPTION=""

parse_arguments() {
    while [ $# -gt 0 ]; do
        case "$1" in
        --help | -h)
            print_help
            exit 0
            ;;
        --version)
            VERSION="$2"
            shift
            ;;
        --platform)
            PLATFORM="$2"
            shift
            ;;
        --runtime)
            USE_RUNTIME=true
            ;;
        --cuda)
            USE_CUDA=true
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
            COMMAND=("$@")
            return
            ;;
        esac
        shift
    done
}

platform_slug() {
    case "$PLATFORM" in
    amd64) echo "amd64" ;;
    jetson) echo "jetson-l4t-r36.4" ;;
    *) echo "amd64" ;;
    esac
}

get_image_name() {
    local base_name="ghcr.io/serene4mr/mowbot"
    local slug
    slug=$(platform_slug)
    local role
    if [ "$USE_RUNTIME" = "true" ]; then
        role="runtime"
    else
        role="devel"
    fi
    echo "${base_name}:${role}-${slug}-${VERSION}"
}

main() {
    parse_arguments "$@"
    local image_name
    image_name=$(get_image_name)
    if ! docker image inspect "$image_name" >/dev/null 2>&1; then
        echo "Image $image_name not found locally. Pulling..."
        docker pull "$image_name"
    fi

    local -a drun=(docker run --name "$CONTAINER_NAME" --rm)
    if [ "$INTERACTIVE" = "true" ]; then
        drun+=(-it)
    fi
    if [ "$DETACHED" = "true" ]; then
        drun+=(-d)
    fi
    if [ "$USE_HOST_NETWORK" = "true" ]; then
        drun+=(--net host)
    fi
    if [ "$USE_PRIVILEGED" = "true" ]; then
        drun+=(--privileged)
    fi
    if [ "$USE_CUDA" = "true" ] || [ "$USE_GPU" = "true" ]; then
        if [ -n "$GPU_OPTION" ]; then
            drun+=(--gpus "$GPU_OPTION")
        else
            drun+=(--gpus all)
        fi
    fi
    drun+=(-v "$WORKSPACE_ROOT:/workspace")
    drun+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
    for volume in "${ADDITIONAL_VOLUMES[@]}"; do
        drun+=(-v "$volume")
    done
    drun+=(-e "DISPLAY=$DISPLAY" -e "LOCAL_UID=$(id -u)" -e "LOCAL_GID=$(id -g)")
    for env_var in "${ADDITIONAL_ENV_VARS[@]}"; do
        drun+=(-e "$env_var")
    done
    drun+=("$image_name")
    if [ ${#COMMAND[@]} -gt 0 ]; then
        drun+=("${COMMAND[@]}")
    fi
    echo "Running: ${drun[*]}"
    exec "${drun[@]}"
}

main "$@"
