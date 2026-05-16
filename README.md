# pFaces-MonoSafe Threshold Experiments

**Real-time invariant-set synthesis for monotone systems.** This repository
contains the threshold-table and real-time controller experiments
behind the paper implementation.

The method stores one threshold height per grid column along
`threshold_d_star`, so safe-set membership is an `O(1)` lookup with
`O(N^(d-1))` storage instead of full-grid storage or basis scans.

In the paper experiments, 3D grids with `10^10` cells
synthesize in 141-221 ms, `10^14` cells complete within 99 s, and online
left-turn control stays below a 100 ms cycle budget.

## Preview

<table>
  <tr>
    <td align="center">
      <b>ACC Basis Evolution</b><br/>
      <video src="./figures/acc_fine_basis_viridis_light.mp4" width="460" controls autoplay loop muted playsinline></video>
    </td>
    <td align="center">
      <b>ACC Threshold Evolution</b><br/>
      <video src="./figures/acc_fine_tt_viridis_light.mp4" width="460" controls autoplay loop muted playsinline></video>
    </td>
  </tr>
</table>

<p align="center">
  <b>Real-Time Left-Turn Controller</b><br/>
  <video src="./figures/intersection.mp4" width="760" controls autoplay loop muted playsinline></video>
</p>

## Method

<img src="./figures/threshold.png" width="460" alt="Threshold-table representation"> 

<img src="./figures/speedups.png" width="460" alt="Threshold speedups"> 

## Setup

Install pFaces and expose both the CLI and SDK:

```bash
export PFACES_SDK_ROOT=/path/to/pfaces-sdk
export PATH=/path/to/pfaces/bin:$PATH
```

Ubuntu packages and Python environment:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git ocl-icd-opencl-dev clinfo \
    libnlopt-cxx-dev python3 python3-venv ffmpeg

python3 -m venv .venv
source .venv/bin/activate
pip install -U pip numpy pandas matplotlib
```

Build the pFaces kernel driver and list the GPU id used by the pFaces CLI:

```bash
sh build.sh
pfaces -G -l
```

## Unified End-To-End Runner

Run the complete pipeline (ACC basis video, ACC threshold video, and RT controller video) using one command:

```bash
./scripts/run_e2e.sh
```

Routing behavior:
- `macOS arm64` -> native run (downloads `pFaces-1.4-MacOS26-ARM64.4zip` into local cache).
- `Linux amd64` -> Docker run.
- Other host combinations fail fast with guidance.

You can still force Docker on macOS with `--mode docker`, but Docker runs Linux containers, so it uses the Linux pFaces asset (not the macOS `.4zip` asset). On Docker Desktop for macOS, GPU OpenCL passthrough is typically unavailable. In that case, ACC videos can run on CPU (`MONOSAFE_ACC_ONLY=1`) with a CL1.2 OpenCL override, while the RT controller stage remains GPU-only.

Optional flags:

```bash
./scripts/run_e2e.sh --mode auto
./scripts/run_e2e.sh --mode native
./scripts/run_e2e.sh --mode docker --output /path/to/output_dir
```

CPU ACC-only Docker run (no RT stage):

```bash
MONOSAFE_ACC_ONLY=1 PFACES_DEVICE_CLASS=C ./scripts/run_e2e.sh --mode docker
```

## Synthesis And Visualization

Run standalone threshold synthesis and render the safe-set evolution:

```bash
./run_threshold_synthesis.sh --cfg examples/acc/acc.cfg --mode threshold --device-class G --device 1
```

The script runs pFaces, records `threshold_evolution.csv`, and writes
`examples/acc/threshold_evolution.mp4`. It also supports basis-mode rendering:

```bash
./run_threshold_synthesis.sh --cfg examples/acc/acc.cfg --mode basis --device-class G --device 1
```

CPU execution is also supported for ACC synthesis:

```bash
./run_threshold_synthesis.sh \
    --cfg examples/acc/acc.cfg \
    --mode threshold \
    --device-class C \
    --device 1 \
    --opencl-opts "-cl-std=CL1.2"
```

For pure synthesis timing with no CSV/video output and no basis extraction:

```bash
./run_threshold_synthesis.sh \
    --cfg examples/turn_oncoming_first/turn_oncoming_first_threshold_rt.cfg \
    --device 1 \
    --timing-only
```

Grid resolution is controlled by `states.eta`; the designated threshold
dimension is controlled by `threshold_d_star` in the cfg file.

## Real-Time Controller

Run the two-oncoming real-time left-turn experiment:

```bash
./run_two_oncoming_threshold_rt.sh
```

The runner builds `tools/rt_controller/build_rt_threshold`, runs
`tools/rt_controller/examples/two_oncoming_threshold_rt.json`, writes logs and
metrics to `tools/rt_controller/experiments/...`, and generates plots plus
`intersection.mp4`.

Fast smoke runs:

```bash
./run_two_oncoming_threshold_rt.sh --no-build --no-plots
./run_two_oncoming_threshold_rt.sh --no-build --no-animate
```

The RT configs are data-driven. Edit `states.eta` and `threshold_d_star` in:

- `examples/turn_oncoming_first/turn_oncoming_first_threshold_rt.cfg`
- `examples/turn_ego_first/turn_ego_first_threshold_rt.cfg`
- `examples/two_oncoming/two_oncoming_threshold_rt.cfg`

The shipped RT phase configs use TT-only GPU synthesis, inline dynamics, no
precomputed transition table, and no basis extraction. On the verified RTX 5090
machine, the default 10.1B-cell phase tables stayed below 100 ms per online
synthesis.

## Bitmap GFP

Validate bitmap GFP against TT-only on ACC:

```bash
python3 tools/check_bitmap_gfp_acc.py
```

Benchmark ACC at large bitmap/threshold grids:

```bash
python3 tools/benchmark_bitmap_gfp_acc.py --sizes 1e8 1e9
```

The broader paper-scaling runner is:

```bash
python3 tools/run_paper_experiments_all_methods.py --help
```
