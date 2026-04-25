# amd64 Devcontainer Notes

This devcontainer is tuned for x86_64 Linux hosts with an NVIDIA GPU.

## Why These `runArgs` Exist

- `--gpus all`: exposes NVIDIA GPUs through the NVIDIA Container Toolkit on amd64 hosts.
- `-v /dev:/dev`: keeps access to host device nodes used by GPU and robotics tooling.
- `--group-add video` and `--group-add render`: not always required for CUDA, but helps avoid permission issues with graphics or `/dev/dri`-related workloads.
- `--net host`, `--ipc host`, `--pid host`: chosen for development workflows that need direct host networking and shared memory behavior.

## Quick GPU Check

- Confirm providers/tools can see GPU:
  - `nvidia-smi`
  - `python3 -c "import torch; print(torch.cuda.is_available())"`
