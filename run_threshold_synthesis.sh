#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CFG="examples/acc/acc.cfg"
MODE="threshold"
DEVICE="${PFACES_GPU_DEVICE:-1}"
DO_SYNTH=1
DO_VIDEO=1
TIMING_ONLY=0
GIF=0
OUT=""
FPS=15
DPI=100
SAMPLE=2

usage() {
    cat <<EOF
Usage: ./run_threshold_synthesis.sh [options]

Runs standalone pFaces synthesis and renders the recorded safe-set evolution.

Options:
  --cfg PATH       pFaces cfg file (default: examples/acc/acc.cfg)
  --mode MODE      threshold or basis (default: threshold)
  --device ID      pFaces GPU device id from 'pfaces -G -l' (default: $DEVICE)
  --output PATH    output video path
  --gif            write GIF instead of MP4
  --fps N          video frames per second (default: $FPS)
  --dpi N          video DPI (default: $DPI)
  --sample N       render every Nth iteration (default: $SAMPLE)
  --timing-only    disable CSV/video output and skip basis extraction
  --no-synth       render from existing CSV only
  --no-video       run synthesis only
  -h, --help       show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --cfg)
            CFG="$2"; shift 2 ;;
        --mode)
            MODE="$2"; shift 2 ;;
        --device)
            DEVICE="$2"; shift 2 ;;
        --output)
            OUT="$2"; shift 2 ;;
        --gif)
            GIF=1; shift ;;
        --fps)
            FPS="$2"; shift 2 ;;
        --dpi)
            DPI="$2"; shift 2 ;;
        --sample)
            SAMPLE="$2"; shift 2 ;;
        --timing-only)
            TIMING_ONLY=1; DO_VIDEO=0; shift ;;
        --no-synth)
            DO_SYNTH=0; shift ;;
        --no-video)
            DO_VIDEO=0; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

case "$MODE" in
    threshold|basis) ;;
    *)
        echo "ERROR: --mode must be 'threshold' or 'basis'." >&2
        exit 2 ;;
esac

if [[ "$CFG" != /* ]]; then
    CFG="$SCRIPT_DIR/$CFG"
fi
CFG_DIR="$(cd "$(dirname "$CFG")" && pwd)"
CFG_FILE="$CFG_DIR/$(basename "$CFG")"
KERNEL_PACK="$SCRIPT_DIR/kernel-pack"

if [[ "$MODE" == "threshold" ]]; then
    CSV="$CFG_DIR/threshold_evolution.csv"
    CO="benchmark_count=1,save_transitions=false,use_threshold_table=true,use_tt_only=true,use_bitmap_gfp=false"
else
    CSV="$CFG_DIR/basis_coordinates.csv"
    CO="benchmark_count=1,save_transitions=false,use_tt_only=false,use_bitmap_gfp=false"
fi

if [[ "$TIMING_ONLY" == 1 ]]; then
    CO="$CO,record_basis_evolution=false,extract_basis=false"
else
    CO="$CO,record_basis_evolution=true"
fi

if [[ -z "$OUT" ]]; then
    if [[ "$GIF" == 1 ]]; then
        OUT="$CFG_DIR/${MODE}_evolution.gif"
    else
        OUT="$CFG_DIR/${MODE}_evolution.mp4"
    fi
fi

if [[ "$DO_SYNTH" == 1 ]]; then
    echo "[1/2] Running $MODE synthesis"
    echo "  cfg:    $CFG_FILE"
    echo "  device: $DEVICE"
    (
        cd "$CFG_DIR"
        pfaces -G -k "mono_synth.gpu@$KERNEL_PACK" \
            -cfg "$CFG_FILE" \
            -d "$DEVICE" -p -v1 \
            -co "$CO"
    )
else
    echo "[1/2] Synthesis skipped"
fi

if [[ "$DO_VIDEO" == 1 ]]; then
    if [[ ! -f "$CSV" ]]; then
        echo "ERROR: expected CSV not found: $CSV" >&2
        exit 1
    fi

    echo "[2/2] Rendering $MODE evolution"
    video_args=(
        python3 "$SCRIPT_DIR/tools/generate_video.py"
        --cfg "$CFG_FILE"
        --csv "$CSV"
        --mode "$MODE"
        --out "$OUT"
        --fps "$FPS"
        --dpi "$DPI"
        --sample "$SAMPLE"
    )
    if [[ "$GIF" == 1 ]]; then
        video_args+=(--gif)
    fi
    "${video_args[@]}"
else
    echo "[2/2] Video skipped"
fi

echo "Done."
if [[ "$DO_VIDEO" == 1 ]]; then
    echo "Video: $OUT"
fi
