#!/usr/bin/env bash
# Setup development environment for Mowbot
# Usage: ./setup-dev-env.sh [options]
# Note: -y option is only for CI.


set -e

# Function to print help message
print_help() {
    echo "Usage: setup-dev-env.sh [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  -y              Use non-interactive mode"
    echo "  -v              Enable debug outputs"
    echo "  --no-nvidia     Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers Disable installation of 'cuda-drivers' in the role 'cuda'"
    echo "  --runtime       Disable installation dev package of role 'cuda' and 'tensorrt'"
    echo "  --data-dir      Set data directory (default: $HOME/autoware_data)"
    echo "  --download-artifacts"
    echo "                  Download artifacts"
    echo "  --module        Specify the module (default: all)"
    echo ""
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")") # Absolute path of this script

# Parse arguments
args=()
option_data_dir="$HOME/mowbot_data"

while [ "$1" != "" ]; do
    case "$1" in
    --help | -h)
        print_help
        exit 1
        ;;
    -y)
        # Use non-interactive mode.
        option_yes=true
        ;;
    -v)
        # Enable debug outputs.
        option_verbose=true
        ;;
    --no-nvidia)
        # Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt').
        option_no_nvidia=true
        ;;
    --no-cuda-drivers)
        # Disable installation of 'cuda-drivers' in the role 'cuda'.
        option_no_cuda_drivers=true
        ;;
    --runtime)
        # Disable installation dev package of role 'cuda' and 'tensorrt'.
        option_runtime=true
        ;;
    --data-dir)
        # Set data directory
        option_data_dir="$2"
        shift
        ;;
    --module)
        option_module="$2"
        shift
        ;;
    *)
        args+=("$1")
        ;;
    esac
    shift
done

# Select installation type (fixed)
target_playbook="mowbot.dev_env.main" # Default playbook

if [ ${#args[@]} -ge 1 ]; then
    target_playbook="autoware.dev_env.${args[0]}"
fi