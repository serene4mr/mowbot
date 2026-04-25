#!/usr/bin/env bash
# Build mowbot container images (runtime / devel).
# Usage: ./build.sh --platform <amd64|jetson> --target <runtime|devel> --version <vX.Y.Z> [options]

set -e

print_help() {
    echo "Usage: build.sh --platform <amd64|jetson> --target <runtime|devel> --version <vX.Y.Z> [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --platform    amd64  (PC) or jetson (L4T — must run on a Jetson/AArch64 host)"
    echo "  --target      runtime  or  devel"
    echo "  --version     Image tag version, e.g. v0.1.0 or a git short SHA"
    echo "  --push        Also docker push the tag (requires login)"
    echo "  -h, --help    This help"
    echo ""
    echo "Examples:"
    echo "  ./build.sh --platform amd64 --target runtime --version v0.1.0"
    echo "  ./build.sh --platform jetson --target devel --version v0.1.0  # run on Jetson"
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
WORKSPACE_ROOT="$SCRIPT_DIR/.."
ENV_DIR="${WORKSPACE_ROOT}/env"

# shellcheck source=/dev/null
load_env_for_platform() {
    local pl=$1
    # shellcheck source=/dev/null
    source "${ENV_DIR}/amd64.env"
    if [ "$pl" = "jetson" ]; then
        # shellcheck source=/dev/null
        source "${ENV_DIR}/jetson-l4t-r36.4.env"
    fi
    : "${mowbot_image:=ghcr.io/serene4mr/mowbot}"
    : "${mowbot_platform_slug:=amd64}"
    : "${rosdistro:=humble}"
    export ROS_DISTRO="${rosdistro}"
}

map_platform() {
    case "$1" in
    amd64) echo "linux/amd64" ;;
    jetson) echo "linux/arm64" ;;
    *) echo "unknown" ;;
    esac
}

# Defaults
option_platform=""
option_target=""
option_version=""
option_push=""

while [ $# -gt 0 ]; do
    case "$1" in
    -h | --help)
        print_help
        exit 0
        ;;
    --platform)
        option_platform="$2"
        shift
        ;;
    --target)
        option_target="$2"
        shift
        ;;
    --version)
        option_version="$2"
        shift
        ;;
    --push)
        option_push="true"
        ;;
    *)
        echo "Unknown option: $1" >&2
        print_help
        exit 1
        ;;
    esac
    shift
done

if [ -z "$option_platform" ] || [ -z "$option_target" ] || [ -z "$option_version" ]; then
    echo "Error: --platform, --target, and --version are required" >&2
    print_help
    exit 1
fi

if [ "$option_target" != "runtime" ] && [ "$option_target" != "devel" ]; then
    echo "Error: --target must be runtime or devel" >&2
    exit 1
fi

if [ "$option_platform" != "amd64" ] && [ "$option_platform" != "jetson" ]; then
    echo "Error: --platform must be amd64 or jetson" >&2
    exit 1
fi

load_env_for_platform "$option_platform"
docker_platform=$(map_platform "$option_platform")

if [ "$option_platform" = "jetson" ] && [ "$(uname -m)" = "x86_64" ]; then
    echo "Error: Jetson images must be built on a Jetson/AArch64 host." >&2
    exit 1
fi

# Clone overlay workspace for docker build
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

clone_repositories

export BUILDKIT_STEP_LOG_MAX_SIZE=10000000
TAG="${mowbot_image}:${option_target}-${mowbot_platform_slug}-${option_version}"
BAKE_FILE="${SCRIPT_DIR}/docker-bake.hcl"
OUTPUT_FLAGS=(--load)
if [ -n "$option_push" ]; then
    OUTPUT_FLAGS=(--push)
fi

echo "=== mowbot docker build ==="
echo "  platform:     $option_platform ($docker_platform)"
echo "  target:       $option_target"
echo "  version:      $option_version"
echo "  image tag:    $TAG"
echo "  base_image:   $base_image"
echo "  ros distro:   $ROS_DISTRO"

set -x
# shellcheck disable=SC2086
docker buildx bake --allow=ssh -f "$BAKE_FILE" \
    "${OUTPUT_FLAGS[@]}" \
    --progress=plain \
    --set "*.context=${WORKSPACE_ROOT}" \
    --set "*.ssh=default" \
    --set "*.platform=${docker_platform}" \
    --set "*.args.BASE_IMAGE=${base_image}" \
    --set "*.args.ROS_DISTRO=${ROS_DISTRO}" \
    --set "${option_target}.tags=${TAG}" \
    "$option_target"
set +x

if [ -n "$option_push" ] && [ "${OUTPUT_FLAGS[0]}" = "--load" ]; then
    echo "Note: use --push to push to a registry" >&2
fi

remove_dangling_images() {
    docker image prune -f
}

remove_dangling_images
