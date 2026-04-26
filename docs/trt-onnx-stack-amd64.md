# TensorRT + ONNX Runtime Stack (AMD64)

## Current versions

- GPU: NVIDIA GeForce RTX 3090 Ti
- NVIDIA Driver: 535.288.01
- CUDA runtime (from driver / nvidia-smi): 12.2
- CUDA toolkit (`nvcc`): 12.4
- TensorRT system libs (apt): `10.8.0.43-1+cuda12.8`
- TensorRT Python: `tensorrt-cu12==10.8.0.43`
- cuDNN (apt): `8.9.7.29-1+cuda12.2`
- ONNX Runtime GPU: `1.23.2`
- ONNX Python: `1.21.0`

## ONNX Runtime providers

Expected providers in this environment:

- `TensorrtExecutionProvider`
- `CUDAExecutionProvider`
- `CPUExecutionProvider`

## Quick health checks

### 1) TensorRT import + builder

```bash
python3 - <<'PY'
import tensorrt as trt
print('TensorRT:', trt.__version__)
logger = trt.Logger(trt.Logger.WARNING)
builder = trt.Builder(logger)
print('Builder OK:', builder is not None)
PY
```

### 2) ONNX Runtime providers

```bash
python3 - <<'PY'
import onnxruntime as ort
print('onnxruntime:', ort.__version__)
print('providers:', ort.get_available_providers())
PY
```

### 3) Engine load test

```bash
python3 - <<'PY'
import tensorrt as trt
p='yolov8n.engine'
logger=trt.Logger(trt.Logger.WARNING)
runtime=trt.Runtime(logger)
with open(p,'rb') as f:
    eng=runtime.deserialize_cuda_engine(f.read())
print('Deserialize OK:', eng is not None)
ctx = eng.create_execution_context() if eng else None
print('Execution context OK:', ctx is not None)
PY
```

## Notes

- TensorRT engine files (`.engine`) are version/build specific. Rebuild engine if deserialization fails after stack changes.
- If ONNX Runtime falls back to CPU, verify CUDA shared libs are discoverable (`ldconfig -p | grep -E 'cudnn|cufft|cublas'`).
