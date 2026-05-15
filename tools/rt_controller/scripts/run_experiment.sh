#!/usr/bin/env bash
# ───────────────────────────────────────────────────────────────────
# run_experiment.sh — Full MonoSafe experiment pipeline
#
# Usage:
#   ./scripts/run_experiment.sh <config.json> [--no-animate]
#
# Example:
#   ./scripts/run_experiment.sh examples/turn_ego_first.json
#
# Pipeline:
#   1. Build rt_controller (CMake Release)
#   2. Create experiment directory: experiments/<scenario>_<timestamp>/
#   3. Copy JSON + CFG configs into experiment directory
#   4. Run closed-loop simulation → sim_log.csv in experiment dir
#   5. Generate static plots + optional MP4 animation
#   6. Save terminal output to experiment log
#
# All outputs for a single run live in one self-contained folder.
# ───────────────────────────────────────────────────────────────────
set -euo pipefail

CFG_INPUT="${1:?Usage: $0 <config.json> [--no-animate]}"
if [[ "$CFG_INPUT" = /* ]]; then
    CFG="$CFG_INPUT"
else
    CFG="$(python3 - "$CFG_INPUT" <<'PY'
import os, sys
print(os.path.abspath(sys.argv[1]))
PY
)"
fi
ANIMATE=true
[[ "${2:-}" == "--no-animate" ]] && ANIMATE=false

# Resolve paths relative to the rt_controller root
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$RT_DIR/build"

# Prefer local Conda packages and common Linux OpenCL locations when available
CMAKE_ARGS=(
    -DCMAKE_BUILD_TYPE=Release
    -DUSE_PFACES_SDK=ON
)
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CONDA_PREFIX")
fi
if [[ -z "${OpenCL_LIBRARY:-}" ]]; then
    for candidate in \
        /lib/x86_64-linux-gnu/libOpenCL.so.1 \
        /usr/lib/x86_64-linux-gnu/libOpenCL.so.1 \
        /usr/local/cuda/targets/x86_64-linux/lib/libOpenCL.so.1
    do
        if [[ -f "$candidate" ]]; then
            CMAKE_ARGS+=("-DOpenCL_LIBRARY=$candidate")
            break
        fi
    done
fi

# Number of parallel jobs
JOBS="$(sysctl -n hw.logicalcpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

# ── Derive experiment directory name ──────────────────────────────
# Extract scenario name from JSON filename (e.g., "turn_ego_first" from "examples/turn_ego_first.json")
SCENARIO="$(basename "$CFG" .json)"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
EXP_DIR="$RT_DIR/experiments/${SCENARIO}_${TIMESTAMP}"

# Extract referenced .cfg file path from JSON
CFG_FILE=$(python3 -c "
import json, sys
with open('$CFG') as f:
    j = json.load(f)
print(j.get('cfg_file', ''))
")

echo "══════════════════════════════════════════════════"
echo "  MonoSafe Experiment Pipeline"
echo "══════════════════════════════════════════════════"
echo "  Config    : $CFG"
echo "  Scenario  : $SCENARIO"
echo "  Experiment: $EXP_DIR"
echo "  CFG ref   : $CFG_FILE"
echo ""

# ── 1. Build ──────────────────────────────────────────────────────
echo "[1/5] Building rt_controller ..."
cmake -S "$RT_DIR" -B "$BUILD_DIR" "${CMAKE_ARGS[@]}" \
      > /dev/null 2>&1 || cmake -S "$RT_DIR" -B "$BUILD_DIR" "${CMAKE_ARGS[@]}"
cmake --build "$BUILD_DIR" -j"$JOBS" 2>&1 | tail -5
echo "  ✓ Build complete"
echo ""

# ── 2. Create experiment directory ────────────────────────────────
echo "[2/5] Creating experiment directory ..."
mkdir -p "$EXP_DIR"

# Copy JSON config
cp "$CFG" "$EXP_DIR/config.json"

# Copy .cfg file if it exists (resolve relative to rt_controller root)
if [[ -n "$CFG_FILE" ]]; then
    RESOLVED_CFG="$CFG_FILE"
    # If relative path, resolve from rt_controller root
    if [[ ! "$RESOLVED_CFG" = /* ]]; then
        RESOLVED_CFG="$RT_DIR/$RESOLVED_CFG"
    fi
    if [[ -f "$RESOLVED_CFG" ]]; then
        cp "$RESOLVED_CFG" "$EXP_DIR/$(basename "$RESOLVED_CFG")"
        echo "  ✓ Copied .cfg: $(basename "$RESOLVED_CFG")"
    else
        echo "  ⚠ .cfg file not found: $RESOLVED_CFG (continuing)"
    fi
fi
echo "  ✓ Experiment dir: $EXP_DIR"
echo ""

# ── 3. Run simulation ────────────────────────────────────────────
echo "[3/5] Running simulation ..."
cd "$RT_DIR"
"$BUILD_DIR/rt_controller" "$CFG" "$EXP_DIR" 2>&1 | tee "$EXP_DIR/run.log"
echo ""

LOG_FILE="$EXP_DIR/sim_log.csv"
if [[ ! -f "$LOG_FILE" ]]; then
    echo "ERROR: Expected log file not found: $LOG_FILE"
    exit 1
fi
STEPS=$(( $(wc -l < "$LOG_FILE") - 1 ))
echo "  ✓ Simulation complete → $LOG_FILE ($STEPS steps)"
echo ""

# ── 4. Visualise (static plots) ──────────────────────────────────
echo "[4/5] Generating static plots → $EXP_DIR ..."
python3 "$RT_DIR/scripts/visualize.py" "$LOG_FILE" --save "$EXP_DIR" 2>&1 | tee -a "$EXP_DIR/run.log"
echo ""

# ── 5. Animation (optional) ──────────────────────────────────────
if $ANIMATE; then
    echo "[5/5] Generating animation → $EXP_DIR ..."
    python3 "$RT_DIR/scripts/visualize.py" "$LOG_FILE" --save "$EXP_DIR" --animate 2>&1 | tee -a "$EXP_DIR/run.log"
else
    echo "[5/5] Animation skipped (--no-animate)"
fi
echo ""

# ── Summary ───────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════"
echo "  Experiment complete."
echo ""
echo "  Directory : $EXP_DIR"
echo "  Contents  :"
ls -1 "$EXP_DIR" | sed 's/^/    /'
echo ""
echo "══════════════════════════════════════════════════"
