#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_DIR="$(cd "$RT_DIR/../.." && pwd)"

CONFIG="$RT_DIR/examples/two_oncoming_threshold_rt.json"
OUTPUT_DIR=""
BUILD_DIR="${RT_BUILD_DIR:-$RT_DIR/build_rt_threshold}"
DO_BUILD=1
DO_PLOTS=1
DO_ANIMATE=1

usage() {
    cat <<EOF
Usage: ./run_two_oncoming_threshold_rt.sh [options] [config.json] [output_dir]

The runner does not rewrite grid dimensions or threshold_d_star. Put those
values in the .cfg files referenced by the JSON config.

Options:
  --config PATH      JSON config to run
  --output DIR      Output directory
  --build-dir DIR   CMake build directory
  --no-build        Reuse existing build
  --plots           Generate plots after the run
  --no-plots        Skip visualization
  --animate         Generate animation after the run (default)
  --no-animate      Generate static plots only
  -h, --help        Show this help

Default config:
  tools/rt_controller/examples/two_oncoming_threshold_rt.json
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG="$2"; shift 2 ;;
        --output)
            OUTPUT_DIR="$2"; shift 2 ;;
        --build-dir)
            BUILD_DIR="$2"; shift 2 ;;
        --no-build)
            DO_BUILD=0; shift ;;
        --plots)
            DO_PLOTS=1; shift ;;
        --no-plots)
            DO_PLOTS=0; DO_ANIMATE=0; shift ;;
        --animate)
            DO_PLOTS=1; DO_ANIMATE=1; shift ;;
        --no-animate)
            DO_ANIMATE=0; shift ;;
        -h|--help)
            usage; exit 0 ;;
        -*)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2 ;;
        *)
            if [[ "$1" == *.json ]]; then
                CONFIG="$1"
            elif [[ -z "$OUTPUT_DIR" ]]; then
                OUTPUT_DIR="$1"
            else
                echo "Unexpected argument: $1" >&2
                usage >&2
                exit 2
            fi
            shift ;;
    esac
done

if [[ "$CONFIG" != /* ]]; then
    CONFIG="$(cd "$REPO_DIR" && python3 -c 'import os,sys; print(os.path.abspath(sys.argv[1]))' "$CONFIG")"
fi

if [[ -z "${PFACES_SDK_ROOT:-}" && -d /home/yasin/pFaces/pfaces-sdk ]]; then
    export PFACES_SDK_ROOT=/home/yasin/pFaces/pfaces-sdk
fi
if [[ -z "${PFACES_SDK_ROOT:-}" ]]; then
    echo "ERROR: PFACES_SDK_ROOT is not set." >&2
    exit 1
fi

if [[ -z "$OUTPUT_DIR" ]]; then
    timestamp="$(date +%Y%m%d_%H%M%S)"
    OUTPUT_DIR="$RT_DIR/experiments/two_oncoming_threshold_rt_${timestamp}"
fi
mkdir -p "$OUTPUT_DIR"

jobs="$(nproc 2>/dev/null || echo 4)"
if [[ "$DO_BUILD" == 1 ]]; then
    echo "[1/4] Building rt_controller in $BUILD_DIR"
    cmake -S "$RT_DIR" -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Release \
        -DUSE_PFACES_SDK=ON
    cmake --build "$BUILD_DIR" -j"$jobs"
else
    echo "[1/4] Reusing existing build in $BUILD_DIR"
fi

echo "[2/4] Running two-oncoming threshold RT experiment"
echo "  config: $CONFIG"
echo "  output: $OUTPUT_DIR"
cd "$RT_DIR"
/usr/bin/time -f "WALL %e s" "$BUILD_DIR/rt_controller" "$CONFIG" "$OUTPUT_DIR" \
    2>&1 | tee "$OUTPUT_DIR/run.log"

LOG_FILE="$OUTPUT_DIR/sim_log.csv"
if [[ ! -f "$LOG_FILE" ]]; then
    echo "ERROR: expected log not found: $LOG_FILE" >&2
    exit 1
fi

echo "[3/4] Summarizing metrics"
python3 - "$LOG_FILE" "$OUTPUT_DIR/metrics.json" <<'PY' | tee -a "$OUTPUT_DIR/run.log"
import csv
import json
import statistics
import sys
from pathlib import Path

rows = list(csv.DictReader(Path(sys.argv[1]).open()))

def f(row, key):
    try:
        return float(row.get(key, 0.0) or 0.0)
    except ValueError:
        return 0.0

def mean(vals):
    return statistics.fmean(vals) if vals else 0.0

def stdev(vals):
    return statistics.stdev(vals) if len(vals) > 1 else 0.0

synth = [r for r in rows if f(r, "synth_ms") > 0.0]
wait = [r for r in rows if f(r, "synth_wait_ms") > 0.0]
go = [r for r in rows if f(r, "synth_go_ms") > 0.0]
ctrl = [f(r, "ctrl_ms") for r in rows]
online_step = [f(r, "ctrl_ms") + f(r, "synth_ms") for r in rows]

metrics = {
    "steps": len(rows),
    "safe_steps": sum(1 for r in rows if int(float(r.get("is_safe", 0) or 0)) != 0),
    "synth_steps": len(synth),
    "max_synth_ms": max([f(r, "synth_ms") for r in rows], default=0.0),
    "avg_synth_ms_when_run": mean([f(r, "synth_ms") for r in synth]),
    "std_synth_ms_when_run": stdev([f(r, "synth_ms") for r in synth]),
    "wait_synth_steps": len(wait),
    "max_wait_synth_ms": max([f(r, "synth_wait_ms") for r in rows], default=0.0),
    "avg_wait_synth_ms_when_run": mean([f(r, "synth_wait_ms") for r in wait]),
    "go_synth_steps": len(go),
    "max_go_synth_ms": max([f(r, "synth_go_ms") for r in rows], default=0.0),
    "avg_go_synth_ms_when_run": mean([f(r, "synth_go_ms") for r in go]),
    "std_go_synth_ms_when_run": stdev([f(r, "synth_go_ms") for r in go]),
    "max_ctrl_ms": max(ctrl, default=0.0),
    "avg_ctrl_ms": mean(ctrl),
    "max_online_step_ms": max(online_step, default=0.0),
    "avg_query_ns": mean([f(r, "query_ns") for r in rows]),
    "max_query_ns": max([f(r, "query_ns") for r in rows], default=0.0),
    "final_safe_fraction": f(rows[-1], "safe_frac") if rows else 0.0,
}
Path(sys.argv[2]).write_text(json.dumps(metrics, indent=2) + "\n")
print(json.dumps(metrics, indent=2))
PY

if [[ "$DO_PLOTS" == 1 ]]; then
    if [[ "$DO_ANIMATE" == 1 ]]; then
        echo "[4/4] Generating plots and animation"
        python3 "$RT_DIR/scripts/visualize.py" "$LOG_FILE" \
            --save "$OUTPUT_DIR" --animate --dpi "${MONOSAFE_RT_DPI:-220}" 2>&1 | tee -a "$OUTPUT_DIR/run.log"
    else
        echo "[4/4] Generating plots"
        python3 "$RT_DIR/scripts/visualize.py" "$LOG_FILE" \
            --save "$OUTPUT_DIR" --dpi "${MONOSAFE_RT_DPI:-220}" 2>&1 | tee -a "$OUTPUT_DIR/run.log"
    fi
else
    echo "[4/4] Plots skipped"
fi

echo "Done: $OUTPUT_DIR"
