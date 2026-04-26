#!/usr/bin/env bash
# Setup development environment for Mowbot
# Usage: ./setup-dev-env.sh [options]
# Note: -y option is only for CI.

set -e

# Function to print help message
print_help() {
    echo "Usage: ./setup-dev-env.sh [OPTIONS]"
    echo "Options:"
    echo "  --help          Display this help message"
    echo "  -h              Display this help message"
    echo "  -y              Use non-interactive mode"
    echo "  -v              Enable debug outputs"
    echo "  --no-nvidia     Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers Disable installation of 'cuda-drivers' in the role 'cuda'"
    echo "  --profile X     Build profile: 'runtime' (minimal) or 'devel' (full tools). Default: devel"
    echo "  --runtime       Same as --profile runtime (backward compatible)"
    echo "  --data-dir      Set data directory (default: $HOME/mowbot_data)"
    echo "  --download-artifacts  In -y mode, download ONNX models and other artifacts"
    echo "  --skip-artifacts      Pass mowbot_skip_artifacts=true (e.g. container image builds)"
    echo "  --install-build-tools For build_profile=runtime, still install build_tools (Docker image build)"
    echo "  --module        Pass module= to ansible (internal)"
    echo ""
    echo "Host-only (run on the machine, not inside a container image build):"
    echo "  ./setup-dev-env.sh host   # runs mowbot.dev_env.host (NVIDIA Container Toolkit)"
    echo ""
}

SCRIPT_DIR=$(readlink -f "$(dirname "$0")") # Absolute path of this script
ENV_DIR="${SCRIPT_DIR}/env"

# Parse arguments
args=()
option_data_dir="$HOME/mowbot_data"
option_build_profile=""
option_download_artifacts=""
option_skip_artifacts=""
option_install_build_tools=""
option_mowbot_non_interactive=""

while [ "$1" != "" ]; do
    case "$1" in
    --help | -h)
        print_help
        exit 1
        ;;
    -y)
        option_yes=true
        option_mowbot_non_interactive="true"
        ;;
    -v)
        option_verbose=true
        ;;
    --no-nvidia)
        option_no_nvidia=true
        ;;
    --no-cuda-drivers)
        option_no_cuda_drivers=true
        ;;
    --profile)
        option_build_profile="$2"
        shift
        ;;
    --runtime)
        option_build_profile="runtime"
        ;;
    --data-dir)
        option_data_dir="$2"
        shift
        ;;
    --download-artifacts)
        option_download_artifacts="y"
        ;;
    --skip-artifacts)
        option_skip_artifacts="true"
        ;;
    --install-build-tools)
        option_install_build_tools="true"
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
    target_playbook="mowbot.dev_env.${args[0]}"
fi

# Load env defaults (amd64 then Jetson overrides on aarch64)
# shellcheck source=/dev/null
source "${ENV_DIR}/amd64.env"
if [ "$(uname -m)" = "aarch64" ]; then
    # shellcheck source=/dev/null
    source "${ENV_DIR}/jetson-l4t-r36.4.env"
fi

# Initialize ansible args
ansible_args=()

# Add env defaults as ansible variables (keys from env/amd64.env; values reflect aarch64 overrides)
# shellcheck disable=SC2013
for env_name in $(sed -e "s/^\s*//" -e "/^#/d" -e "s/=.*//" <"${ENV_DIR}/amd64.env"); do
    ansible_args+=("--extra-vars" "${env_name}=${!env_name}")
done
# shellcheck disable=SC2013
if [ -f "${ENV_DIR}/jetson-l4t-r36.4.env" ] && [ "$(uname -m)" = "aarch64" ]; then
    for env_name in $(sed -e "s/^\s*//" -e "/^#/d" -e "s/=.*//" <"${ENV_DIR}/jetson-l4t-r36.4.env"); do
        ansible_args+=("--extra-vars" "${env_name}=${!env_name}")
    done
fi

if [ "$target_playbook" = "mowbot.dev_env.host" ] && [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_nvidia_container_toolkit=y")
fi

# Confirm to start installation
if [ "$option_yes" = "true" ]; then
    echo -e "\e[36mRun the setup in non-interactive mode.\e[m"
else
    echo -e "\e[33mSetting up the build environment can take up to 1 hour.\e[m"
    read -rp ">  Are you sure you want to run setup? [y/N] " answer

    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi

    ansible_args+=("--ask-become-pass")
fi

# Verbose
if [ "$option_verbose" = "true" ]; then
    ansible_args+=("-vvv")
fi

# NVIDIA libraries
if [ "$option_no_nvidia" = "true" ]; then
    ansible_args+=("--extra-vars" "prompt_install_nvidia=n")
elif [ "$option_yes" = "true" ]; then
    if [ -n "${prompt_install_nvidia}" ]; then
        ansible_args+=("--extra-vars" "prompt_install_nvidia=${prompt_install_nvidia}")
    else
        ansible_args+=("--extra-vars" "prompt_install_nvidia=y")
    fi
fi

# Build profile: runtime = minimal; devel = build_tools + dev_tools + devel CUDA/TRT
if [ -n "$option_build_profile" ]; then
    ansible_args+=("--extra-vars" "build_profile=${option_build_profile}")
elif [ "$option_yes" = "true" ]; then
    ansible_args+=("--extra-vars" "build_profile=devel")
fi

# Non-interactive: satisfy vars_prompt and skip download unless requested
if [ "$option_yes" = "true" ]; then
    if [ -z "${option_download_artifacts}" ]; then
        ansible_args+=("--extra-vars" "prompt_download_artifacts=n")
    else
        ansible_args+=("--extra-vars" "prompt_download_artifacts=y")
    fi
fi

if [ "$option_skip_artifacts" = "true" ]; then
    ansible_args+=("--extra-vars" "mowbot_skip_artifacts=true")
fi

if [ "$option_mowbot_non_interactive" = "true" ]; then
    ansible_args+=("--extra-vars" "mowbot_non_interactive=true")
fi

if [ "$option_install_build_tools" = "true" ]; then
    ansible_args+=("--extra-vars" "mowbot_install_build_tools=true")
fi

# Host playbook: no container docker restart var needed for main; host.yaml handles toolkit
if { [ -f /.dockerenv ] || [ -f /run/.containerenv ]; } && [ "$target_playbook" = "mowbot.dev_env.host" ]; then
    echo "Refusing to run host playbook inside a container." >&2
    exit 1
fi

# Disable Docker restart when running in container (legacy; main playbook no longer installs toolkit)
if [ -f /.dockerenv ] || [ -f /run/.containerenv ] || [ ! -d /run/systemd/system ] || ! ps -p 1 -o comm= 2>/dev/null | grep -qE "^systemd|^init$"; then
    ansible_args+=("--extra-vars" "nvidia_container_toolkit_restart_docker=false")
fi

ansible_args+=("--extra-vars" "data_dir=$option_data_dir")

# Module
if [ "${option_module:-}" != "" ]; then
    ansible_args+=("--extra-vars" "module=$option_module")
fi

# CUDA drivers
if [ "$option_no_cuda_drivers" = "true" ]; then
    ansible_args+=("--extra-vars" "cuda_install_drivers=false")
fi

# Install sudo
if ! (command -v sudo >/dev/null 2>&1); then
    apt-get -y update
    apt-get -y install sudo
fi

# Install git
if ! (command -v git >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install git
fi

# Install pip for ansible
if ! (python3 -m pip --version >/dev/null 2>&1); then
    sudo apt-get -y update
    sudo apt-get -y install python3-pip python3-venv
fi

# Install pipx for ansible
if ! (python3 -m pipx --version >/dev/null 2>&1); then
    sudo apt-get -y update
    if ! sudo apt-get -y install pipx; then
        python3 -m pip install --user --index-url https://pypi.org/simple pipx
    fi
fi

# Install ansible
python3 -m pipx ensurepath
export PATH="${PIPX_BIN_DIR:=$HOME/.local/bin}:$PATH"
PIP_INDEX_URL="https://pypi.org/simple" \
    PIP_EXTRA_INDEX_URL="" \
    pipx install --include-deps --force "ansible==6.*"

# Install ansible collections
echo -e "\e[36mansible-galaxy collection install -f -r ${SCRIPT_DIR}/ansible-galaxy-requirements.yaml\e[m"
ansible-galaxy collection install -f -r "$SCRIPT_DIR/ansible-galaxy-requirements.yaml"

# Run ansible
echo -e "\e[36mansible-playbook ${target_playbook} ${ansible_args[*]}\e[m"
if ansible-playbook "$target_playbook" "${ansible_args[@]}"; then
    echo -e "\e[32mCompleted.\e[0m"
    exit 0
else
    echo -e "\e[31mFailed.\e[0m"
    exit 1
fi
