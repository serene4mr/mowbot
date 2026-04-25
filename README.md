# Mowbot

A ROS2-based autonomous mowing robot platform with comprehensive navigation, perception, and control capabilities.

## Quick Start with Docker

### Build images locally

```bash
cd docker
./build.sh --platform amd64 --target devel --version v0.1.0
./build.sh --platform amd64 --target runtime --version v0.1.0
```

Jetson (on device — must run on a Jetson/AArch64 host):

```bash
./build.sh --platform jetson --target devel --version v0.1.0
```

### Run a container

```bash
./docker/run.sh --cuda              # devel image, GPU
./docker/run.sh --runtime           # production-style runtime image
./docker/run.sh --platform jetson  # when using jetson-tagged images
```

### Registry tags (ghcr.io/serene4mr/mowbot)

- `runtime-amd64-<version>` / `devel-amd64-<version>` — PC (CI builds on `main` also publishes `*-latest`)
- `runtime-jetson-l4t-r36.4-<version>` / `devel-jetson-l4t-r36.4-<version>` — Jetson (build on device)

**Deprecated (removed in this layout):** `base`, `base-cuda`, `main`, `main-dev`, `main-cuda`, `main-dev-cuda` — use `runtime-*` / `devel-*` above.

### Native / Ansible host

```bash
./setup-dev-env.sh                  # interactive
./setup-dev-env.sh -y --profile devel
./setup-dev-env.sh host -y         # install NVIDIA Container Toolkit on the **host** only
```

## Documentation

- **[Docker Guide](docker/README.md)** — container build and run
- **Ansible** — [ansible/README.md](ansible/README.md) — `env/amd64.env` and `env/jetson-l4t-r36.4.env`

## Project Structure

- `src/` — ROS2 packages
- `docker/` — `Dockerfile`, `build.sh`, `run.sh`
- `env/` — platform environment files for builds and setup
- `ansible/` — system setup and roles
- `build/` & `install/` — colcon build artifacts (local / container)

## Dependencies

- **ROS 2 Humble**
- **CUDA / TensorRT** (optional, platform-pinned via `env/*.env`)
- **Ubuntu 22.04** (for playbooks; Jetson base image is L4T-based)

## Getting Started

1. **Clone with submodules:**
   ```bash
   git clone --recursive https://github.com/serene4mr/mowbot.git
   cd mowbot
   ```

2. **Development approach:**
   - **Dev Containers:** see [.devcontainer/](.devcontainer/) (base image `devel-*-latest`)
   - **Docker:** `./docker/run.sh`
   - **Native:** `./setup-dev-env.sh`

3. **Build the workspace**
   ```bash
   colcon build
   source install/setup.bash
   ```

## License

This project is licensed as specified in the individual package directories.
