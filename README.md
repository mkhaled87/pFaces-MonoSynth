# pFaces-MonoSynth Threshold Experiments

**Real-time invariant-set synthesis for monotone systems.** This repository
contains the threshold-table, bitmap-GFP, and real-time controller experiments
behind the paper implementation.

The main idea is simple: a lower-closed safe set does not need to be stored as a
full grid or scanned as a basis. We store one threshold height per column along
a designated dimension `threshold_d_star`, giving `O(N^(d-1))` memory and `O(1)`
safety queries.

<p align="center">
  <img src="../website_figures/threshold.png" width="720" alt="Threshold-table representation">
</p>

**What this enables.** The threshold iteration removes the basis scan and
neighbor-generation bottlenecks in lazy monotone synthesis. In the paper
experiments (`../docs/main.tex`), 3D grids with `10^10` cells synthesize in
141-221 ms, `10^14` cells finish within 99 s, and the online left-turn
controller remains below a 100 ms cycle budget.

## Visual Overview

| Lazy basis iteration | Threshold iteration |
| --- | --- |
| <img src="../website_figures/lazy_alg.png" width="390" alt="Lazy basis algorithm"> | <img src="../website_figures/threshold_alg.png" width="510" alt="Threshold iteration algorithm"> |

| Timing comparison | Speedups |
| --- | --- |
| <img src="../website_figures/comparison.png" width="480" alt="Lazy vs threshold timing comparison"> | <img src="../website_figures/speedups.png" width="330" alt="Threshold speedups"> |

### Videos

- ACC safe-set evolution: [basis](../website_figures/acc_fine_basis_viridis_light.mp4) vs [threshold table](../website_figures/acc_fine_tt_viridis_light.mp4)
- Real-time unprotected left turn: [animation](../website_figures/intersection.mp4)

<p align="center">
  <a href="../website_figures/intersection.mp4">
    <img src="../website_figures/intersection_last_frame.png" width="720" alt="Real-time left-turn controller animation frame">
  </a>
</p>

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

## Synthesis And Visualization

Run standalone threshold synthesis and render the safe-set evolution:

```bash
./run_threshold_synthesis.sh --cfg examples/acc/acc.cfg --mode threshold --device 1
```

The script runs pFaces, records `threshold_evolution.csv`, and writes
`examples/acc/threshold_evolution.mp4`. It also supports basis-mode rendering:

```bash
./run_threshold_synthesis.sh --cfg examples/acc/acc.cfg --mode basis --device 1
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
