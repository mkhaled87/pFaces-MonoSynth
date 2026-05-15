#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec "$SCRIPT_DIR/tools/rt_controller/scripts/run_two_oncoming_threshold_rt.sh" "$@"
