#!/bin/bash
set -e

function resolve_pip_requirements() {
    local src_path="${1:-src}"

    while IFS= read -r -d '' requirements_file; do
        while IFS= read -r line || [[ -n "$line" ]]; do
            # Skip comments and empty/whitespace-only lines.
            [[ "$line" =~ ^[[:space:]]*# ]] && continue
            [[ "$line" =~ ^[[:space:]]*$ ]] && continue
            printf '%s\n' "$line"
        done < "$requirements_file"
    done < <(find "$src_path" -name 'requirements.txt' -type f -print0 | sort -z)
}

resolve_pip_requirements "$@"