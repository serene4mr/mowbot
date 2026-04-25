# DevContainer Configurations

VS Code DevContainer setups for Mowbot. Base images use the unified registry tags:

- `ghcr.io/serene4mr/mowbot:devel-amd64-latest` — PC (GPU stack per env)
- `ghcr.io/serene4mr/mowbot:devel-jetson-l4t-r36.4-latest` — Jetson (build on device / manual push)

## Configurations

### `devel/` — CPU-focused devcontainer

- **Base image**: `devel-amd64-latest`
- **Use**: General development; add `--gpus` in `runArgs` if you need GPU.

### `devel-cuda/` — PC with NVIDIA GPU

- **Base image**: `devel-amd64-latest`
- **Use**: Same image as above; `runArgs` includes `--gpus all`.

### `devel-cuda-jetson/` — Jetson

- **Base image**: `devel-jetson-l4t-r36.4-latest`
- **Use**: On Jetson hardware; uses `nvidia` container runtime.

## Usage

1. Open the repo in VS Code with the Dev Containers extension.
2. **Dev Containers: Reopen in Container** and pick a configuration (or copy one to `.devcontainer/devcontainer.json`).

See the [main README](../README.md) for tag naming and migration from old `main-dev-cuda` style tags.
