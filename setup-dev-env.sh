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

# Initialize ansible args
ansible_args=()

# Confirm to start installation
if [ "$option_yes" = "true" ]; then
    echo -e "\e[36mRun the setup in non-interactive mode.\e[m"
else
    echo -e "\e[33mSetting up the build environment can take up to 1 hour.\e[m"
    read -rp ">  Are you sure you want to run setup? [y/N] " answer

    # Check whether to cancel
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi

    ansible_args+=("--ask-become-pass")
fi

# Check verbose option
if [ "$option_verbose" = "true" ]; then
    ansible_args+=("-vvv")
fi

# Check installation of NVIDIA libraries
if [ "$option_no_nvidia" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_nvidia=n")
elif [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_nvidia=y")
fi

# Check installation of CUDA Drivers
if [ "$option_no_cuda_drivers" = "true" ]; then
    ansible_args+=("--extra-vars" "cuda_install_drivers=false")
fi

# Check installation of dev package
if [ "$option_runtime" = "true" ]; then
    ansible_args+=("--extra-vars" "ros2_installation_type=ros-base") # ROS installation type, default "desktop"
    ansible_args+=("--extra-vars" "install_devel=N")
else
    ansible_args+=("--extra-vars" "install_devel=y")
fi