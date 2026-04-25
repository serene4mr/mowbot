# DevContainer Configurations

Two configurations — one per deployment platform.

## `amd64/` — PC with NVIDIA GPU

- **Base image**: `ghcr.io/serene4mr/mowbot:devel-amd64-latest`
- **GPU**: `--gpus all` (CUDA always present in the devel image)
- **Use on**: any x86_64 Linux machine with an NVIDIA GPU

## `jetson/` — Jetson (L4T r36.4)

- **Base image**: `ghcr.io/serene4mr/mowbot:devel-jetson-l4t-r36.4-latest`
- **GPU**: `--runtime=nvidia` (Tegra integrated GPU via NVIDIA container runtime)
- **Use on**: Jetson device with JetPack / L4T r36.4 installed on host
- `PATH` and `LD_LIBRARY_PATH` extended with CUDA paths via `remoteEnv`

## Usage

1. Open repo in VS Code with the **Dev Containers** extension installed.
2. `Ctrl+Shift+P` → **Dev Containers: Open Folder in Container...**
3. Choose `amd64` or `jetson`.

## Notes

- Both configurations build on top of `.devcontainer/Dockerfile` which adds a VS Code-compatible user layer.
- `updateRemoteUserUID` is `true` for amd64 (matches host UID), `false` for Jetson (Jetson user setup differs).
- Jetson images are built manually on-device; see [docker/build.sh](../docker/build.sh).
