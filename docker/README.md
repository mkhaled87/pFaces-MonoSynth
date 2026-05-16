# Docker Quickstart

This Docker image builds `pFaces-MonoSynth_clean_threshold_table` and runs all three videos in one command:

1. ACC basis synthesis video
2. ACC threshold-table (TT-only) synthesis video
3. Two-oncoming real-time controller video

## Preferred Entry Point

From repo root, use the unified host runner:

```bash
./scripts/run_e2e.sh --mode docker
```

Or call the Docker wrapper directly:

```bash
./docker/run_with_bind_mounts.sh
```

`docker/run_with_bind_mounts.sh` now:
- builds the image first,
- forces `linux/amd64` when needed (for example on Apple Silicon),
- mounts `tools/rt_controller/experiments` back to host,
- passes through optional env overrides.

## Output Artifacts

Artifacts are written on host under:
- `tools/rt_controller/experiments/docker_run_*/acc_basis_evolution.mp4`
- `tools/rt_controller/experiments/docker_run_*/acc_threshold_evolution.mp4`
- `tools/rt_controller/experiments/docker_run_*/rt_two_oncoming/intersection.mp4`

## Environment Overrides

Optional runtime overrides:

```bash
PFACES_DEVICE_CLASS=G|C      # unset => auto (GPU first, CPU fallback if pFaces CPU device exists)
PFACES_GPU_DEVICE=<id>       # override detected pFaces device id
PFACES_OPENCL_OPTS=<opts>    # optional pFaces -op flags (CPU default: -cl-std=CL1.2)
MONOSAFE_ACC_ONLY=1          # CPU path: run ACC videos only, skip RT stage
OUTPUT_DIR=<path>            # choose output directory
MONOSAFE_DOCKER_IMAGE=<tag>  # default: monosafe:latest
MONOSAFE_DOCKER_PLATFORM=<platform>  # default auto; usually linux/amd64
MONOSAFE_DOCKER_USE_GPUS=auto|always|never  # default: auto
MONOSAFE_DOCKER_RUN_AS_HOST_USER=auto|always|never  # default: auto
```

## Important Notes

- `pFaces 1.4.0d` demo license has `Max PEs = 50`.
- GPUs above the demo PE limit fail unless a non-demo license is used.
- CPU ACC fallback uses a CL1.2 override (`-cl-std=CL1.2`) because common CPU ICDs (POCL) do not expose OpenCL C 2.0.
- The RT controller stage is currently GPU-coupled; CPU runs must set `MONOSAFE_ACC_ONLY=1`.
- For CPU runs, `run_with_bind_mounts.sh` automatically avoids host UID remapping (`MONOSAFE_DOCKER_RUN_AS_HOST_USER=auto`) for runtime stability.
- On macOS, Docker can run this image (`linux/amd64`), but it still uses the Linux pFaces asset and often cannot access host GPU OpenCL through Docker Desktop.
