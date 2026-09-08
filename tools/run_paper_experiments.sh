#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PFACES_BIN="${PFACES_BIN:-pfaces}"
PFACES_DEVICE="${PFACES_DEVICE:-0}"
RESULTS_DIR="${1:-$REPO_ROOT/tools/benchmark_results/paper_scale_$(date +%Y%m%d_%H%M%S)}"

exec python3 "$SCRIPT_DIR/run_full_paper_matrix.py" \
    --pfaces "$PFACES_BIN" \
    --device "$PFACES_DEVICE" \
    --warmups 0 \
    --repetitions 1 \
    --timeout 1200 \
    --max-advance-seconds 60 \
    --output "$RESULTS_DIR"
