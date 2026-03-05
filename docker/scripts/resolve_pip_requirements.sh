#!/bin/bash
set -e

function resolve_pip_requirements() {
    local src_path="${1:-src}"

    find "$src_path" -name 'requirements.txt' -type f | sort | xargs cat | grep -v '^#' | grep -v '^[[:space:]]*$'
}

resolve_pip_requirements "$@"