#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OUTPUT_ROOT="${1:-${OUTPUT_DIR:-/outputs}}"
PFACES_DEVICE="${PFACES_GPU_DEVICE:-}"
PFACES_CLASS="${PFACES_DEVICE_CLASS:-C}"

mkdir -p "$OUTPUT_ROOT"

if [[ -z "$PFACES_DEVICE" ]]; then
    if [[ "$PFACES_CLASS" == "C" ]]; then
        first_line="$(pfaces -C -l 2>/dev/null | sed -E 's/\x1b\[[0-9;]*m//g' | awk '/\[[0-9]+: CPU\]/{print; exit}')"
    else
        first_line="$(pfaces -G -l 2>/dev/null | sed -E 's/\x1b\[[0-9;]*m//g' | awk '/\[[0-9]+: GPU\]/{print; exit}')"
    fi
    if [[ -n "$first_line" ]]; then
        PFACES_DEVICE="$(echo "$first_line" | sed -E 's/.*\[([0-9]+):.*/\1/')"
    else
        PFACES_DEVICE="1"
    fi
fi

echo "Output directory: $OUTPUT_ROOT"
echo "Using pFaces class: $PFACES_CLASS"
echo "Using pFaces device id: $PFACES_DEVICE"

if command -v clinfo >/dev/null 2>&1; then
    clinfo | sed -n '1,80p' || true
fi

run_acc() {
    local mode="$1"
    local csv="$2"
    local out="$3"
    local co=""
    if [[ "$mode" == "basis" ]]; then
        co="benchmark_count=1,save_transitions=false,use_tt_only=false,use_bitmap_gfp=false,record_basis_evolution=true"
    else
        co="benchmark_count=1,save_transitions=false,use_threshold_table=true,use_tt_only=true,use_bitmap_gfp=false,record_basis_evolution=true"
    fi

    (
        cd "$ROOT_DIR/examples/acc"
        pfaces "-$PFACES_CLASS" -k "mono_synth@$ROOT_DIR/kernel-pack" \
            -cfg "acc.cfg" -d "$PFACES_DEVICE" -p -v1 -co "$co"
    )
    python3 "$ROOT_DIR/tools/generate_video.py" \
        --cfg "$ROOT_DIR/examples/acc/acc.cfg" \
        --csv "$ROOT_DIR/examples/acc/$csv" \
        --mode "$mode" \
        --out "$out" \
        --fps "${MONOSAFE_SYNTH_FPS:-15}" \
        --dpi "${MONOSAFE_SYNTH_DPI:-120}" \
        --sample "${MONOSAFE_SYNTH_SAMPLE:-2}"
}

echo "[1/3] ACC synthesis (basis) + video"
run_acc "basis" "basis_coordinates.csv" "$OUTPUT_ROOT/acc_basis_evolution.mp4"

echo "[2/3] ACC synthesis (threshold table / TT-only) + video"
run_acc "threshold" "threshold_evolution.csv" "$OUTPUT_ROOT/acc_threshold_evolution.mp4"

echo "[3/3] Real-time controller + plots + animation"
RT_OUTPUT="$OUTPUT_ROOT/rt_two_oncoming"
mkdir -p "$RT_OUTPUT"

TMP_RT_JSON="$RT_OUTPUT/two_oncoming_threshold_rt.runtime.json"
python3 - "$ROOT_DIR/tools/rt_controller/examples/two_oncoming_threshold_rt.json" "$TMP_RT_JSON" "$PFACES_DEVICE" <<'PY'
import json
import sys

src, dst, device = sys.argv[1], sys.argv[2], int(sys.argv[3])
with open(src, "r", encoding="utf-8") as f:
    cfg = json.load(f)
cfg["synthesis_go"]["mode"] = "external"
cfg["synthesis_wait"]["mode"] = "external"
cfg["synthesis_go"]["device_id"] = device
cfg["synthesis_wait"]["device_id"] = device
with open(dst, "w", encoding="utf-8") as f:
    json.dump(cfg, f, indent=2)
    f.write("\n")
PY

"$ROOT_DIR/run_two_oncoming_threshold_rt.sh" \
    --config "$TMP_RT_JSON" \
    --output "$RT_OUTPUT" \
    --no-build \
    --animate

echo "Done. Generated artifacts:"
find "$OUTPUT_ROOT" -maxdepth 3 -type f \
    \( -name "*.mp4" -o -name "*.gif" -o -name "metrics.json" -o -name "run.log" \) \
    | sort
