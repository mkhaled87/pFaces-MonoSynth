#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${MONOSAFE_E2E_MODE:-auto}"
OUTPUT_DIR_ARG="${OUTPUT_DIR:-}"
PFACES_RELEASE_TAG="Release_1.4.0d"
PFACES_MAC_ASSET="pFaces-1.4-MacOS26-ARM64.4zip"

usage() {
    cat <<USAGE
Usage: ./scripts/run_e2e.sh [options]

Unified runner for local native (macOS arm64) and Docker (Linux path).

Options:
  --mode MODE     auto | native | docker (default: auto)
  --output DIR    output directory (default: script-specific timestamp path)
  -h, --help      show this help

Environment:
  MONOSAFE_E2E_MODE            default mode override
  MONOSAFE_CACHE_DIR           cache root for native pFaces download
  MONOSAFE_NATIVE_SKIP_BUILD   set to 1 to skip native build steps
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            MODE="$2"; shift 2 ;;
        --output)
            OUTPUT_DIR_ARG="$2"; shift 2 ;;
        -h|--help)
            usage
            exit 0 ;;
        *)
            echo "ERROR: Unknown argument: $1" >&2
            usage >&2
            exit 2 ;;
    esac
done

MODE="$(echo "$MODE" | tr '[:upper:]' '[:lower:]')"
HOST_OS="$(uname -s)"
HOST_ARCH="$(uname -m)"

strip_ansi() {
    sed -E 's/\x1b\[[0-9;]*m//g'
}

resolve_mode() {
    case "$MODE" in
        auto)
            case "${HOST_OS}/${HOST_ARCH}" in
                Darwin/arm64|Darwin/aarch64)
                    echo "native" ;;
                Linux/x86_64|Linux/amd64)
                    echo "docker" ;;
                *)
                    echo "ERROR: Unsupported host ${HOST_OS}/${HOST_ARCH} for auto mode." >&2
                    echo "Auto mode supports macOS arm64 (native) and Linux amd64 (docker)." >&2
                    echo "Use --mode docker manually only if your Docker runtime supports linux/amd64 emulation." >&2
                    exit 1 ;;
            esac
            ;;
        native|docker)
            echo "$MODE" ;;
        *)
            echo "ERROR: --mode must be one of: auto, native, docker." >&2
            exit 2 ;;
    esac
}

ensure_native_prereqs() {
    local missing=0
    for cmd in curl unzip cmake python3; do
        if ! command -v "$cmd" >/dev/null 2>&1; then
            echo "Missing required command for native mode: $cmd" >&2
            missing=1
        fi
    done
    if (( missing != 0 )); then
        exit 1
    fi
}

bootstrap_pfaces_macos() {
    local cache_root
    local install_dir
    cache_root="${MONOSAFE_CACHE_DIR:-$HOME/.cache/monosafe}"
    install_dir="$cache_root/pfaces/$PFACES_RELEASE_TAG/macos-arm64"

    if [[ -x "$install_dir/bin/pfaces" && -d "$install_dir/pfaces-sdk" ]]; then
        echo "Using cached pFaces at: $install_dir"
    else
        echo "Downloading pFaces $PFACES_RELEASE_TAG macOS arm64 asset..."
        mkdir -p "$install_dir"
        local tmp_zip
        tmp_zip="$(mktemp -t pfaces_macos26_arm64.XXXXXX).zip"
        trap 'rm -f "$tmp_zip"' RETURN

        curl -fsSL "https://github.com/parallall/pFaces/releases/download/${PFACES_RELEASE_TAG}/${PFACES_MAC_ASSET}" -o "$tmp_zip"
        rm -rf "$install_dir"/*
        unzip -q "$tmp_zip" -d "$install_dir"
        rm -f "$tmp_zip"
        trap - RETURN
    fi

    export PATH="$install_dir/bin:$PATH"
    export PFACES_SDK_ROOT="$install_dir/pfaces-sdk"

    if [[ ! -x "$install_dir/bin/pfaces" || ! -d "$PFACES_SDK_ROOT" ]]; then
        echo "ERROR: pFaces bootstrap failed at $install_dir." >&2
        exit 1
    fi
}

first_gpu_line() {
    local output
    output="$(pfaces -G -l 2>&1 || true)"
    printf '%s\n' "$output" | strip_ansi | awk '/\[[0-9]+: GPU\]/{print; exit}'
}

run_native() {
    if [[ "$HOST_OS" != "Darwin" || ("$HOST_ARCH" != "arm64" && "$HOST_ARCH" != "aarch64") ]]; then
        echo "ERROR: Native mode requires macOS arm64." >&2
        echo "Use --mode docker on Linux hosts." >&2
        exit 1
    fi

    ensure_native_prereqs
    bootstrap_pfaces_macos

    local gpu_line
    gpu_line="$(first_gpu_line)"
    if [[ -z "$gpu_line" ]]; then
        echo "ERROR: No suitable OpenCL GPU was detected by pFaces on this macOS host." >&2
        echo "This workflow requires GPU/OpenCL support (CPU OpenCL fallback is not expected on macOS arm64)." >&2
        exit 1
    fi

    local detected_device
    detected_device="$(echo "$gpu_line" | sed -E 's/.*\[([0-9]+):.*/\1/')"
    if [[ -z "${PFACES_GPU_DEVICE:-}" ]]; then
        export PFACES_GPU_DEVICE="$detected_device"
    fi

    if [[ -n "${PFACES_DEVICE_CLASS:-}" && "${PFACES_DEVICE_CLASS^^}" != "G" ]]; then
        echo "ERROR: Native mode currently supports GPU execution only. Set PFACES_DEVICE_CLASS=G or unset it." >&2
        exit 1
    fi

    export PFACES_DEVICE_CLASS=G

    if [[ "${MONOSAFE_NATIVE_SKIP_BUILD:-0}" != "1" ]]; then
        echo "Building kernel driver for native host..."
        "$ROOT_DIR/build.sh"

        echo "Building rt_controller for native host..."
        cmake -S "$ROOT_DIR/tools/rt_controller" -B "$ROOT_DIR/tools/rt_controller/build_rt_threshold" \
            -DCMAKE_BUILD_TYPE=Release -DUSE_PFACES_SDK=ON
        cmake --build "$ROOT_DIR/tools/rt_controller/build_rt_threshold" -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
    fi

    export MONOSAFE_RT_NO_BUILD=1
    if [[ -n "$OUTPUT_DIR_ARG" ]]; then
        "$ROOT_DIR/docker/run_all_videos.sh" "$OUTPUT_DIR_ARG"
    else
        "$ROOT_DIR/docker/run_all_videos.sh"
    fi
}

run_docker() {
    if [[ ! -x "$ROOT_DIR/docker/run_with_bind_mounts.sh" ]]; then
        echo "ERROR: docker/run_with_bind_mounts.sh is missing or not executable." >&2
        exit 1
    fi

    if [[ -n "$OUTPUT_DIR_ARG" ]]; then
        "$ROOT_DIR/docker/run_with_bind_mounts.sh" "$OUTPUT_DIR_ARG"
    else
        "$ROOT_DIR/docker/run_with_bind_mounts.sh"
    fi
}

SELECTED_MODE="$(resolve_mode)"
echo "Selected mode: $SELECTED_MODE (${HOST_OS}/${HOST_ARCH})"

if [[ "$SELECTED_MODE" == "native" ]]; then
    run_native
else
    run_docker
fi
