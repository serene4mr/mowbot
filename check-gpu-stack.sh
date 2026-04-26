#!/usr/bin/env bash
# GPU stack health check: driver, TensorRT, ONNX Runtime GPU.
# Usage: ./check-gpu-stack.sh [--quiet]
# Exit code: 0 = all checks passed, 1 = one or more failed.

set -euo pipefail

QUIET=false
[[ "${1:-}" == "--quiet" ]] && QUIET=true

PASS=0
FAIL=0

_green() { echo -e "\e[32m$*\e[0m"; }
_red()   { echo -e "\e[31m$*\e[0m"; }
_bold()  { echo -e "\e[1m$*\e[0m"; }

pass() { PASS=$((PASS+1)); $QUIET || _green "  ✅  $*"; }
fail() { FAIL=$((FAIL+1)); _red   "  ❌  $*"; }

$QUIET || _bold "\n=== Mowbot GPU stack health check ===\n"

# ── 1. nvidia-smi ──────────────────────────────────────────────────────────────
$QUIET || echo "[1/5] NVIDIA driver / GPU"
if nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | head -1 | grep -q "."; then
    info=$(nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | head -1)
    pass "GPU visible: $info"
else
    fail "nvidia-smi failed — driver not loaded or no GPU present"
fi

# ── 2. TensorRT system libs ─────────────────────────────────────────────────────
$QUIET || echo "[2/5] TensorRT system libs (apt)"
if dpkg-query -W -f='${Package} ${Version}\n' libnvinfer10 libcudnn8 2>/dev/null | grep -q "libnvinfer10"; then
    trt_apt=$(dpkg-query -W -f='${Version}' libnvinfer10 2>/dev/null)
    cudnn_apt=$(dpkg-query -W -f='${Version}' libcudnn8 2>/dev/null)
    pass "libnvinfer10=$trt_apt  libcudnn8=$cudnn_apt"
else
    fail "libnvinfer10 not installed — run Ansible tensorrt role"
fi

# ── 3. TensorRT Python ─────────────────────────────────────────────────────────
$QUIET || echo "[3/5] TensorRT Python binding"
trt_result=$(python3 - <<'PY'
import sys
try:
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    if builder is None:
        raise RuntimeError("Builder returned None")
    print(f"OK {trt.__version__}")
except Exception as e:
    print(f"FAIL {e}", file=sys.stderr)
    sys.exit(1)
PY
)
if [[ $? -eq 0 ]]; then
    pass "TensorRT Python: $trt_result"
else
    fail "TensorRT Python: $trt_result"
fi

# ── 4. ONNX Runtime providers ──────────────────────────────────────────────────
$QUIET || echo "[4/5] ONNX Runtime GPU providers"
ort_result=$(python3 - <<'PY'
import sys
try:
    import onnxruntime as ort
    providers = ort.get_available_providers()
    has_cuda = "CUDAExecutionProvider" in providers
    has_trt  = "TensorrtExecutionProvider" in providers
    status = "OK" if has_cuda else "WARN"
    print(f"{status} v{ort.__version__}  providers={providers}  cuda={has_cuda}  trt={has_trt}")
    if not has_cuda:
        sys.exit(1)
except Exception as e:
    print(f"FAIL {e}", file=sys.stderr)
    sys.exit(2)
PY
)
exit_code=$?
if [[ $exit_code -eq 0 ]]; then
    pass "ONNX Runtime: $ort_result"
else
    fail "ONNX Runtime: $ort_result"
fi

# ── 5. ONNX Runtime GPU inference smoke test ───────────────────────────────────
$QUIET || echo "[5/5] ONNX Runtime GPU inference (CUDAExecutionProvider)"
infer_result=$(python3 - <<'PY'
import sys
import numpy as np
try:
    import onnx, onnxruntime as ort
    from onnx import helper, TensorProto
    X   = helper.make_tensor_value_info('x', TensorProto.FLOAT, [1, 3])
    Y   = helper.make_tensor_value_info('y', TensorProto.FLOAT, [1, 3])
    one = onnx.numpy_helper.from_array(np.ones((1, 3), dtype=np.float32), name='one')
    g   = helper.make_graph([helper.make_node('Add', ['x', 'one'], ['y'])], 'g', [X], [Y], [one])
    m   = helper.make_model(g, opset_imports=[helper.make_opsetid('', 11)])
    m.ir_version = 6
    sess = ort.InferenceSession(m.SerializeToString(),
                                providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
    y = sess.run(['y'], {'x': np.zeros((1, 3), dtype=np.float32)})[0]
    active = sess.get_providers()[0]
    assert (y == np.ones((1, 3))).all(), "wrong result"
    print(f"OK provider={active} result={y.tolist()}")
    if active != 'CUDAExecutionProvider':
        print("WARN: fell back to CPU", file=sys.stderr)
        sys.exit(1)
except Exception as e:
    print(f"FAIL {e}", file=sys.stderr)
    sys.exit(2)
PY
)
exit_code=$?
if [[ $exit_code -eq 0 ]]; then
    pass "Inference: $infer_result"
else
    fail "Inference: $infer_result"
fi

# ── Summary ────────────────────────────────────────────────────────────────────
echo ""
if [[ $FAIL -eq 0 ]]; then
    _green "All $PASS checks passed ✅"
    exit 0
else
    _red "$FAIL of $((PASS+FAIL)) checks failed ❌"
    exit 1
fi
