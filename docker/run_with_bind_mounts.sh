#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
EXPERIMENTS_DIR="$ROOT_DIR/tools/rt_controller/experiments"
IMAGE_NAME="${MONOSAFE_DOCKER_IMAGE:-monosafe:latest}"
OUTPUT_DIR_ARG="${OUTPUT_DIR:-}"

usage() {
    cat <<USAGE
Usage: ./docker/run_with_bind_mounts.sh [options] [output_dir]

Builds the Docker image and runs the full video pipeline with bind mounts.

Options:
  --output DIR    output directory
  -h, --help      show this help
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output)
            OUTPUT_DIR_ARG="$2"; shift 2 ;;
        -h|--help)
            usage
            exit 0 ;;
        *)
            if [[ -z "$OUTPUT_DIR_ARG" ]]; then
                OUTPUT_DIR_ARG="$1"
                shift
            else
                echo "ERROR: Unexpected argument: $1" >&2
                usage >&2
                exit 2
            fi
            ;;
    esac
done

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker is not installed or not in PATH." >&2
    exit 1
fi

HOST_OS="$(uname -s)"
HOST_ARCH="$(uname -m)"

if [[ -n "${MONOSAFE_DOCKER_PLATFORM:-}" ]]; then
    DOCKER_PLATFORM="$MONOSAFE_DOCKER_PLATFORM"
else
    case "${HOST_OS}/${HOST_ARCH}" in
        Linux/x86_64|Linux/amd64)
            DOCKER_PLATFORM="linux/amd64"
            ;;
        Darwin/arm64|Darwin/aarch64|Linux/arm64|Linux/aarch64)
            DOCKER_PLATFORM="linux/amd64"
            echo "Host ${HOST_OS}/${HOST_ARCH} detected; forcing ${DOCKER_PLATFORM} for pFaces compatibility."
            ;;
        *)
            echo "ERROR: Unsupported host ${HOST_OS}/${HOST_ARCH}." >&2
            echo "Set MONOSAFE_DOCKER_PLATFORM=linux/amd64 manually if your Docker runtime supports emulation." >&2
            exit 1
            ;;
    esac
fi

GPU_POLICY="${MONOSAFE_DOCKER_USE_GPUS:-auto}"
USE_GPU_FLAG=0
case "$GPU_POLICY" in
    auto)
        if docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
            USE_GPU_FLAG=1
        fi
        ;;
    always)
        USE_GPU_FLAG=1
        ;;
    never)
        USE_GPU_FLAG=0
        ;;
    *)
        echo "ERROR: MONOSAFE_DOCKER_USE_GPUS must be one of: auto, always, never." >&2
        exit 2
        ;;
esac

USER_POLICY="${MONOSAFE_DOCKER_RUN_AS_HOST_USER:-auto}"
USE_HOST_USER=1
case "$USER_POLICY" in
    auto)
        if [[ "${PFACES_DEVICE_CLASS:-}" =~ ^[cC]$ || "${MONOSAFE_ACC_ONLY:-0}" == "1" ]]; then
            USE_HOST_USER=0
        fi
        ;;
    always)
        USE_HOST_USER=1
        ;;
    never)
        USE_HOST_USER=0
        ;;
    *)
        echo "ERROR: MONOSAFE_DOCKER_RUN_AS_HOST_USER must be one of: auto, always, never." >&2
        exit 2
        ;;
esac

mkdir -p "$EXPERIMENTS_DIR"

echo "Building Docker image ${IMAGE_NAME} (${DOCKER_PLATFORM})"
docker build --platform "$DOCKER_PLATFORM" -t "$IMAGE_NAME" "$ROOT_DIR"

run_args=(
    run --rm
    --platform "$DOCKER_PLATFORM"
    -v "$EXPERIMENTS_DIR:/workspace/tools/rt_controller/experiments"
    -e "POCL_CACHE_DIR=/tmp/pocl_cache"
)

if (( USE_HOST_USER == 1 )); then
    run_args+=(--user "$(id -u):$(id -g)")
else
    echo "Running container as default user for runtime stability (CPU/OpenCL path)."
fi

if (( USE_GPU_FLAG == 1 )); then
    run_args+=(--gpus all)
fi

if [[ -n "$OUTPUT_DIR_ARG" ]]; then
    run_args+=(-e "OUTPUT_DIR=$OUTPUT_DIR_ARG")
fi
if [[ -n "${PFACES_DEVICE_CLASS:-}" ]]; then
    run_args+=(-e "PFACES_DEVICE_CLASS=$PFACES_DEVICE_CLASS")
fi
if [[ -n "${PFACES_GPU_DEVICE:-}" ]]; then
    run_args+=(-e "PFACES_GPU_DEVICE=$PFACES_GPU_DEVICE")
fi
if [[ -n "${PFACES_OPENCL_OPTS:-}" ]]; then
    run_args+=(-e "PFACES_OPENCL_OPTS=$PFACES_OPENCL_OPTS")
fi
if [[ -n "${MONOSAFE_ACC_ONLY:-}" ]]; then
    run_args+=(-e "MONOSAFE_ACC_ONLY=$MONOSAFE_ACC_ONLY")
fi

run_args+=("$IMAGE_NAME")

echo "Running ${IMAGE_NAME}"
docker "${run_args[@]}"
