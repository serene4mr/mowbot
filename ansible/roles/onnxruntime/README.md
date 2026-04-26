# onnxruntime

Installs ONNX Runtime GPU and the ONNX package via pip.

## Purpose

Provides GPU-accelerated ONNX model inference for mowbot perception nodes via:

- **`onnxruntime-gpu`** — GPU inference with `CUDAExecutionProvider` and `TensorrtExecutionProvider`
- **`onnx`** — ONNX model serialisation / inspection utilities

> **Prerequisites**
> - `mowbot.dev_env.cuda` must run first — it registers the pip `nvidia-*` CUDA shared
>   libs in ldconfig, which `CUDAExecutionProvider` needs to load `libcufft.so.11` etc.
> - `mowbot.dev_env.tensorrt` is optional but enables `TensorrtExecutionProvider`.

## Variables

| Name | Default | Description |
|------|---------|-------------|
| `onnxruntime_version` | `""` | Pin to a specific version (e.g. `"1.23.2"`). Empty = latest. |
| `onnx_version` | `""` | Pin `onnx` package version. Empty = latest. |
| `onnxruntime_expected_providers` | `[CUDAExecutionProvider, TensorrtExecutionProvider]` | Providers validated when `onnxruntime_verify_installation: true` |
| `onnxruntime_verify_installation` | `false` | Run provider check after install |

## Usage

### Pinned version (recommended for production)

```yaml
- hosts: localhost
  roles:
    - role: mowbot.dev_env.cuda
    - role: mowbot.dev_env.onnxruntime
  vars:
    onnxruntime_version: "1.23.2"
    onnx_version: "1.21.0"
    onnxruntime_verify_installation: true
```

### Latest version

```yaml
- hosts: localhost
  roles:
    - role: mowbot.dev_env.cuda
    - role: mowbot.dev_env.onnxruntime
```

## Verification

```bash
# Check available execution providers
python3 -c "import onnxruntime as ort; print(ort.get_available_providers())"
# Expected: ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']

# Quick GPU inference smoke-test
python3 - <<'PY'
import onnx, onnxruntime as ort, numpy as np
from onnx import helper, TensorProto
X = helper.make_tensor_value_info('x', TensorProto.FLOAT, [1, 3])
Y = helper.make_tensor_value_info('y', TensorProto.FLOAT, [1, 3])
one_init = onnx.numpy_helper.from_array(np.ones((1,3), dtype=np.float32), name='one')
graph = helper.make_graph([helper.make_node('Add', ['x','one'], ['y'])], 'g', [X], [Y], [one_init])
model = helper.make_model(graph, opset_imports=[helper.make_opsetid('', 11)])
model.ir_version = 6
sess = ort.InferenceSession(model.SerializeToString(), providers=['CUDAExecutionProvider'])
result = sess.run(['y'], {'x': np.zeros((1,3), dtype=np.float32)})
print("Provider:", sess.get_providers()[0])
print("Result:", result[0])  # [[1. 1. 1.]]
PY
```

## Troubleshooting

### `CUDAExecutionProvider` fails to load (`libcufft.so.11` not found)

The pip `nvidia-cufft` package installs the lib under `site-packages/nvidia/cufft/lib/`
which is not in the default linker path. Ensure the `mowbot.dev_env.cuda` role has run
(it writes `/etc/ld.so.conf.d/nvidia-pip-cuda.conf` and refreshes ldconfig).

```bash
ldconfig -p | grep cufft   # should show libcufft.so.11
```

### `TensorrtExecutionProvider` not listed

Install the `mowbot.dev_env.tensorrt` role. The provider is only available when
`libnvinfer` system packages are present.
