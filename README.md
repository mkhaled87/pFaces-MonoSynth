# pFaces-MonoSafe Threshold Experiments

**Real-time invariant-set synthesis for monotone systems.** This repository
contains the threshold-table and real-time controller experiments
behind the paper implementation.

The method stores one threshold height per grid column along
`threshold_d_star`, so safe-set membership is an `O(1)` lookup with
`O(N^(d-1))` storage instead of full-grid storage or basis scans.

The paper runner compares six implementations: host CDC scan, host CDC with
threshold indexing, GPU Automatica with scan or threshold membership, bitmap
GFP, and the proposed column-wise threshold GFP.

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
export PATH=/path/to/pfaces/bin:$PATH$PATH
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

## Reproduce the Paper Experiments

The release runner executes the exact progressive protocol used for the revised
paper: host CDC scan, host CDC with threshold indexing, both GPU Automatica
variants, bitmap GFP, and threshold GFP. It covers ACC, ACC-5D, Turn-Ego, and
Turn-Oncoming from the smallest scale upward.

After installing pFaces and building the kernel driver, select the GPU reported
by `pfaces -G -l` and run:

```bash
export PFACES_SDK_ROOT=/path/to/pfaces-sdk
export PATH=/path/to/pfaces/bin:$PATH
export PFACES_DEVICE=0

sh build.sh
./tools/run_paper_experiments.sh
```

The optional first argument selects the output directory. Reuse it to resume an
interrupted experiment:

```bash
./tools/run_paper_experiments.sh tools/benchmark_results/paper_scale_run
```

The runner performs one measured execution with no warm-up, uses a 20-minute
per-run timeout, and advances a method only when its preceding wall time is at
most 60 seconds. It prepares and validates each reusable successor cache once.
Threshold GFP and bitmap GFP evaluate dynamics inline.

Every run is checkpointed. The result directory contains the complete 156-row
plan, raw logs and timings, validation metadata, CSV summaries, and Markdown
and LaTeX tables. Skips and timeouts remain explicit. Successful methods on a
shared grid must agree in canonical output hash and safe-cell count.

Advanced method, example, and scale selection is documented in
[tools/README.md](tools/README.md).

## Run One Synthesis

Use the standalone launcher for a single configuration:

```bash
./run_threshold_synthesis.sh \
  --cfg examples/acc/acc.cfg \
  --mode threshold \
  --device-class G \
  --device 0
```

Grid resolution is controlled by `states.eta`; `threshold_d_star` selects the
threshold dimension. See [tools/README.md](tools/README.md) for all solver and
benchmark options.

## Real-Time Controller

The C++20 controller combines online threshold synthesis with MPPI and supports
ACC, both single-vehicle turn phases, and the two-oncoming demonstration. Build
and run the complete two-oncoming example with:

```bash
cd tools/rt_controller
./scripts/run_two_oncoming_threshold_rt.sh
```

The script produces CSV logs, plots, and the left-turn animation. Dependencies,
configuration fields, and shorter smoke commands are in the
[RT controller guide](tools/rt_controller/README.md).

The controller uses finite safe-set penalties during optimization. It does not
enforce a hard invariant constraint and does not provide a formal closed-loop
safety guarantee.

## Paper Sources

The submitted manuscript, revised manuscript, rebuttal, and bibliography are in
[`docs/`](docs/). Build the revised paper with:

```bash
cd docs
latexmk -pdf lcss_revised.tex
```

## License

See [LICENSE](LICENSE).
