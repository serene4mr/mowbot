# Mowbot Docker

## Images

| Tag pattern | Role |
|-------------|------|
| `runtime-<platform>-<ver>` | Deploy: minimal base + built workspace (compilers for image build are installed during Docker build only; use `mowbot_install_build_tools` there) |
| `devel-<platform>-<ver>` | Full dev: extends runtime with `build_profile=devel` (build tools, dev tools, optional CUDA/TRT devel) |

`platform` is `amd64` or `jetson-l4t-r36.4` (from `env/*.env`).

## Build

From repository root (requires `vcs` / `vcstool` and Docker Buildx; SSH for private `mowbot.repos`).

```bash
cd docker
chmod +x build.sh
./build.sh --platform amd64 --target runtime --version v0.1.0
./build.sh --platform amd64 --target devel --version v0.1.0
./build.sh --platform jetson --target devel --version v0.1.0  # run on a Jetson/AArch64 host
```

- `--push` — push the tag to the registry (after `docker login`).

`build.sh` reads `env/amd64.env` for `--platform amd64`, and merges `env/jetson-l4t-r36.4.env` for `--platform jetson`.

## Run

```bash
./run.sh                    # devel, amd64, tag latest
./run.sh --runtime          # runtime image
./run.sh --cuda --gpus all  # with GPU
./run.sh --platform jetson
```

## Dockerfile layout

- Single [Dockerfile](Dockerfile) with targets `runtime` and `devel`.
- `FROM $BASE_IMAGE` and `ARG ROS_DISTRO` are passed by `docker buildx bake` from `env/` via `build.sh`.
- The runtime stage calls `./setup-dev-env.sh -y --profile runtime --install-build-tools` so the workspace can be built in-image; a second `devel` stage calls `--profile devel` on top of `runtime`.

## See also

- [README.md](../README.md) — project overview and migration from old tag names
- [ansible/README.md](../ansible/README.md) — Ansible roles and `mowbot.dev_env.host` for NVIDIA Container Toolkit on the **host**
