# Docker Quickstart

This image builds `pFaces-MonoSynth_clean_threshold_table` once, then runs all publication videos in one command:

1. ACC basis synthesis video
2. ACC threshold-table (TT-only) synthesis video
3. Two-oncoming real-time controller video

Note: the bundled `pFaces` binary ships with a demo license. On GPUs with more than 50 compute units, pFaces synthesis will fail with a license PE limit. For those machines, use a valid pFaces license file/image variant.

## 1) Build image

From `pFaces-MonoSynth_clean_threshold_table/`:

```bash
docker build -t monosafe:latest .
```

## 2) Run all videos (one command)

```bash
mkdir -p outputs
docker run --rm --gpus all \
  -e PFACES_DEVICE_CLASS=C \
  -e PFACES_GPU_DEVICE=1 \
  -v "$(pwd)/outputs:/outputs" \
  monosafe:latest
```

Artifacts are written to `outputs/` on the host:

- `outputs/acc_basis_evolution.mp4`
- `outputs/acc_threshold_evolution.mp4`
- `outputs/rt_two_oncoming/intersection.mp4`

## 3) Optional: use a prebuilt image from Docker Hub

```bash
docker pull <dockerhub-user>/monosafe:latest
docker run --rm --gpus all \
  -e PFACES_DEVICE_CLASS=C \
  -e PFACES_GPU_DEVICE=1 \
  -v "$(pwd)/outputs:/outputs" \
  <dockerhub-user>/monosafe:latest
```
