# Jetson Devcontainer Notes

This devcontainer is tuned for Jetson (L4T/JetPack) where GPU access depends on host device nodes and NVIDIA runtime behavior.

## Why These `runArgs` Exist

- `--runtime=nvidia`: required on Jetson to expose Tegra GPU stack in containers.
- `--privileged` and `-v /dev:/dev`: gives access to Jetson device nodes used by CUDA/TensorRT (`/dev/nv*`, `/dev/nvhost*`).
- `--group-add video` and `--group-add render`: ensures the container user can open device files that are `root:video`/`root:render` (for example `/dev/nvmap`), avoiding TensorRT `Permission denied` failures.
- `--net host`, `--ipc host`, `--pid host`: used for robotics/dev workflows that need low-friction interop with host networking and shared memory.

## If TensorRT Fails With Permission Errors

1. Rebuild/reopen the container so updated `runArgs` are applied.
2. Check groups inside container: `id` (should include `video` and/or `render`).
3. Verify TensorRT can initialize a builder:
   - `python3 -c "import tensorrt as trt; trt.Builder(trt.Logger()); print('TRT GPU OK')"`
