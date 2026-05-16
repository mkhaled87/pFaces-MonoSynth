#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DEFAULT_OUTPUT_ROOT="$ROOT_DIR/tools/rt_controller/experiments/docker_run_$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${1:-${OUTPUT_DIR:-$DEFAULT_OUTPUT_ROOT}}"
PFACES_DEVICE="${PFACES_GPU_DEVICE:-}"
PFACES_CLASS="${PFACES_DEVICE_CLASS:-}"
PFACES_OPENCL_OPTS="${PFACES_OPENCL_OPTS:-}"
ACC_ONLY="${MONOSAFE_ACC_ONLY:-0}"

strip_ansi() {
    sed -E 's/\x1b\[[0-9;]*m//g'
}

first_device_line_for_class() {
    local class="$1"
    local output
    output="$(pfaces "-$class" -l 2>&1 || true)"
    printf '%s\n' "$output" | strip_ansi | awk -v cls="$class" '
        cls == "G" && /\[[0-9]+: GPU\]/{print; exit}
        cls == "C" && /\[[0-9]+: CPU\]/{print; exit}
    '
}

first_device_id_for_class() {
    local class="$1"
    local line
    line="$(first_device_line_for_class "$class")"
    if [[ -n "$line" ]]; then
        echo "$line" | sed -E 's/.*\[([0-9]+):.*/\1/'
    fi
}

if ! command -v pfaces >/dev/null 2>&1; then
    echo "ERROR: pfaces CLI is not available in PATH." >&2
    exit 1
fi

case "$ACC_ONLY" in
    0|1) ;;
    *)
        echo "ERROR: MONOSAFE_ACC_ONLY must be 0 or 1 (got '$ACC_ONLY')." >&2
        exit 2
        ;;
esac

mkdir -p "$OUTPUT_ROOT"

if [[ -n "$PFACES_CLASS" ]]; then
    PFACES_CLASS="$(echo "$PFACES_CLASS" | tr '[:lower:]' '[:upper:]')"
    case "$PFACES_CLASS" in
        G|C) ;;
        *)
            echo "ERROR: PFACES_DEVICE_CLASS must be 'G' or 'C' (got '$PFACES_CLASS')." >&2
            exit 2
            ;;
    esac
else
    gpu_line="$(first_device_line_for_class G)"
    cpu_line="$(first_device_line_for_class C)"
    if [[ -n "$gpu_line" ]]; then
        PFACES_CLASS="G"
    elif [[ -n "$cpu_line" ]]; then
        PFACES_CLASS="C"
        echo "No OpenCL GPU detected. Falling back to CPU mode."
    else
        echo "ERROR: No suitable pFaces devices were detected." >&2
        echo "Checked pFaces GPU list (-G -l) and CPU list (-C -l), but both were empty/unavailable." >&2
        exit 1
    fi
fi

if [[ "$PFACES_CLASS" == "C" && -z "$PFACES_OPENCL_OPTS" ]]; then
    # pFaces Linux build defaults to CL2.0, but common CPU ICDs expose CL1.2.
    PFACES_OPENCL_OPTS="-cl-std=CL1.2"
fi

if [[ -z "$PFACES_DEVICE" ]]; then
    PFACES_DEVICE="$(first_device_id_for_class "$PFACES_CLASS")"
fi
if [[ -z "$PFACES_DEVICE" ]]; then
    echo "ERROR: Could not determine a pFaces device id for class '$PFACES_CLASS'." >&2
    echo "Set PFACES_GPU_DEVICE explicitly after checking 'pfaces -${PFACES_CLASS} -l'." >&2
    exit 1
fi

echo "Output directory: $OUTPUT_ROOT"
echo "Using pFaces class: $PFACES_CLASS"
echo "Using pFaces device id: $PFACES_DEVICE"
if [[ -n "$PFACES_OPENCL_OPTS" ]]; then
    echo "Using OpenCL opts: $PFACES_OPENCL_OPTS"
fi

if command -v clinfo >/dev/null 2>&1; then
    clinfo | sed -n '1,80p' || true
fi

if [[ "$PFACES_CLASS" == "G" ]]; then
    pfaces_gpu_line="$(first_device_line_for_class G)"
    if [[ -z "$pfaces_gpu_line" ]]; then
        echo "ERROR: No OpenCL GPU was detected by pFaces." >&2
        echo "Check GPU/OpenCL visibility for this runtime." >&2
        exit 1
    fi

    pfaces_help_output="$(pfaces -h 2>&1 || true)"
    license_max_pes="$(printf '%s\n' "$pfaces_help_output" | strip_ansi | awk -F: '/Max PEs/{gsub(/ /,"",$2); print $2; exit}')"
    gpu_compute_units=""
    if command -v clinfo >/dev/null 2>&1; then
        gpu_compute_units="$(clinfo 2>/dev/null | awk '
            /Device Type[[:space:]]+GPU/{gpu=1}
            gpu && /Max compute units/{print $NF; exit}
        ')"
    fi
    if [[ -z "${gpu_compute_units:-}" ]]; then
        if [[ "$(uname -s)" == "Darwin" && ! -x "$(command -v clinfo 2>/dev/null || true)" ]]; then
            echo "pFaces GPU detected: $pfaces_gpu_line"
            echo "clinfo is unavailable on this macOS host; skipping compute-unit license preflight."
        elif command -v clinfo >/dev/null 2>&1; then
            echo "ERROR: clinfo did not report an OpenCL GPU, although pFaces listed one." >&2
            echo "Check NVIDIA Container Toolkit setup and OpenCL ICD visibility." >&2
            exit 1
        else
            echo "The 'clinfo' utility is unavailable, so GPU OpenCL capability cannot be validated." >&2
            exit 1
        fi
    fi
    if [[ -n "${license_max_pes:-}" && -n "${gpu_compute_units:-}" ]]; then
        if (( gpu_compute_units > license_max_pes )); then
            echo "ERROR: pFaces demo license allows up to ${license_max_pes} PEs, but this GPU has ${gpu_compute_units} compute units." >&2
            echo "Use a <=${license_max_pes} CU GPU, a non-demo pFaces license, or set PFACES_DEVICE_CLASS=C for CPU ACC-only mode." >&2
            exit 1
        fi
    fi
elif [[ "$PFACES_CLASS" == "C" ]]; then
    if [[ -z "$(first_device_line_for_class C)" ]]; then
        echo "ERROR: CPU mode requested, but pFaces did not detect a CPU OpenCL device." >&2
        exit 1
    fi
    if [[ "$ACC_ONLY" != "1" ]]; then
        echo "ERROR: CPU mode is currently supported for ACC synthesis videos only." >&2
        echo "The two-oncoming RT controller stage is GPU-coupled in this build." >&2
        echo "Re-run with MONOSAFE_ACC_ONLY=1 to generate ACC videos on CPU, or use GPU mode for the full 3-stage pipeline." >&2
        exit 1
    fi
fi

run_acc() {
    local mode="$1"
    local out="$2"
    local run_dir="$OUTPUT_ROOT/acc_${mode}_work"

    mkdir -p "$run_dir"
    cp "$ROOT_DIR/examples/acc/acc.cfg" "$run_dir/acc.cfg"
    cp "$ROOT_DIR/examples/acc/acc_dynamics.cl" "$run_dir/acc_dynamics.cl"

    acc_cmd=(
        "$ROOT_DIR/run_threshold_synthesis.sh"
        --cfg "$run_dir/acc.cfg"
        --mode "$mode"
        --device-class "$PFACES_CLASS"
        --device "$PFACES_DEVICE"
        --output "$out"
        --fps "${MONOSAFE_SYNTH_FPS:-15}"
        --dpi "${MONOSAFE_SYNTH_DPI:-120}"
        --sample "${MONOSAFE_SYNTH_SAMPLE:-2}"
    )
    if [[ -n "$PFACES_OPENCL_OPTS" ]]; then
        acc_cmd+=(--opencl-opts "$PFACES_OPENCL_OPTS")
    fi
    "${acc_cmd[@]}"
}

echo "[1/3] ACC synthesis (basis) + video"
run_acc "basis" "$OUTPUT_ROOT/acc_basis_evolution.mp4"

echo "[2/3] ACC synthesis (threshold table / TT-only) + video"
run_acc "threshold" "$OUTPUT_ROOT/acc_threshold_evolution.mp4"

if [[ "$ACC_ONLY" == "1" ]]; then
    echo "[3/3] Real-time controller skipped (MONOSAFE_ACC_ONLY=1)."
    echo "Done. Generated artifacts:"
    find "$OUTPUT_ROOT" -maxdepth 3 -type f \
        \( -name "*.mp4" -o -name "*.gif" -o -name "metrics.json" -o -name "run.log" \) \
        | sort
    exit 0
fi

echo "[3/3] Real-time controller + plots + animation"
RT_OUTPUT="$OUTPUT_ROOT/rt_two_oncoming"
mkdir -p "$RT_OUTPUT"

TMP_RT_JSON="$RT_OUTPUT/two_oncoming_threshold_rt.runtime.json"
python3 - "$ROOT_DIR/tools/rt_controller/examples/two_oncoming_threshold_rt.json" "$TMP_RT_JSON" "$PFACES_DEVICE" <<'PY'
import json
import sys

src, dst, cli_device = sys.argv[1], sys.argv[2], int(sys.argv[3])
with open(src, "r", encoding="utf-8") as f:
    cfg = json.load(f)
for key in ("synthesis_go", "synthesis_wait"):
    mode = cfg[key].get("mode", "external")
    if mode == "external":
        cfg[key]["device_id"] = cli_device
with open(dst, "w", encoding="utf-8") as f:
    json.dump(cfg, f, indent=2)
    f.write("\n")
PY

rt_args=(
    --config "$TMP_RT_JSON"
    --output "$RT_OUTPUT"
    --animate
)
if [[ "${MONOSAFE_RT_NO_BUILD:-1}" == "1" ]]; then
    rt_args+=(--no-build)
fi

bash "$ROOT_DIR/run_two_oncoming_threshold_rt.sh" "${rt_args[@]}"

echo "Done. Generated artifacts:"
find "$OUTPUT_ROOT" -maxdepth 3 -type f \
    \( -name "*.mp4" -o -name "*.gif" -o -name "metrics.json" -o -name "run.log" \) \
    | sort
