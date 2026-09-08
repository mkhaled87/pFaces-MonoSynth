# pFaces-MonoSynth: Real-Time Synthesis of Robust Controlled Invariant Sets for Monotone Systems

Safety-critical autonomy needs formal safety certificates — controlled
invariant sets — recomputed online as conditions change. Classical symbolic
greatest-fixed-point (GFP) computation scales poorly with state dimension,
so real-time synthesis is usually out of reach.

For monotone dynamics with lower-closed safety specs, the maximal robust
controlled invariant set is itself lower-closed and fully described by its
boundary. Lazy basis methods exploit this by tracking only the antichain
basis, but every iteration still scans the basis for each membership test
and regenerates neighbors sequentially, which is the main bottleneck at scale.

This repo implements our paper's **threshold-function reformulation**: a
lower-closed set on a `d`-dimensional grid is stored as one column height
per key along a designated axis. One GFP step then becomes an independent
one-dimensional binary search per column. Intuition: instead of maintaining
a moving basis frontier, each column independently asks "how high can this
column stay safe?" with `O(1)` table lookups without basis scans or neighbor
generation.

This method provides:

* **`O(1)` lookup, `O(N^(d-1))` storage** — one integer per column instead of the full `O(N^d)` grid or per-query basis scans.
* **Embarrassingly parallel** — all columns update independently; per-iteration work `O(C·N^(d-1)·log N)` with no sequential neighbor generation (lazy: `O(C·N^(2(d-1)))` sequential over an evolving basis).
* **Real-time capable** — `10^9` cells in under 50 ms, `10^14` cells in under two minutes; `10^10`-cell closed-loop re-synthesis in ~62–77 ms.
* **Thousands-fold speedups** — orders of magnitude faster than basis-based lazy synthesis at matched grids.
* **Real-time example** — Safety-informed MPPI demo that re-synthesizes online.
* **Parallel implementation** — A parallel OpenCL implementation using pFaces for maximum speed on GPUs.

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

<p align="center">
  <img src="./figures/threshold.png" width="460" alt="Threshold-table representation" />
  <img src="./figures/speedups.png" width="460" alt="Paper speedup table" />
</p>

## 1. Install pFaces First

You need an external pFaces installation before anything else:

```bash
export PFACES_SDK_ROOT=/path/to/pfaces-sdk
export PATH=/path/to/pfaces/bin:$PATH
pfaces -G -l   # note your GPU id, e.g. 0
```

Then system deps and this kernel driver:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git ocl-icd-opencl-dev clinfo \
    libnlopt-cxx-dev python3 python3-venv ffmpeg

python3 -m venv .venv
source .venv/bin/activate
pip install -U pip numpy pandas matplotlib

sh build.sh
```

## 2. Try ACC in 2 Minutes (Start Here)

Runs threshold synthesis on the small ACC example and renders the animation:

```bash
./run_threshold_synthesis.sh \
  --cfg examples/acc/acc.cfg \
  --mode threshold \
  --device-class G \
  --device 0 \
  --video
```

Output: `examples/acc/threshold_evolution.mp4` plus the evolution CSV.
Swap `--mode threshold` for `--mode cdc` to see the classical basis version.

## 3. Quick Method Check

A coarse threshold-vs-reference comparison on the same ACC grid:

```bash
python3 tools/run_solver_benchmark.py \
  examples/acc/acc.cfg \
  --methods threshold,bitmap_reference \
  --state-eta 4,2,2 \
  --device 0 \
  --warmups 0 \
  --repetitions 1 \
  --output tools/benchmark_results/acc_smoke
```

Passing means both methods agree on the safe set; timings and logs go to
the output directory. All solver options are in [tools/README.md](tools/README.md).

## 4. Full Paper-Scale Experiments

This command reproduces the paper figure runs (host CDC scan, host
CDC-threshold, two GPU Automatica variants, bitmap GFP, threshold GFP)
over ACC, ACC-5D, Turn-Ego and Turn-Oncoming, smallest scale first:

```bash
export PFACES_SDK_ROOT=/path/to/pfaces-sdk
export PATH=/path/to/pfaces/bin:$PATH
export PFACES_DEVICE=0

./tools/run_paper_experiments.sh
```

Resume an interrupted run by reusing the output directory:

```bash
./tools/run_paper_experiments.sh tools/benchmark_results/paper_scale_run
```

One measured run per case, 20-minute per-run timeout; results, logs, CSVs and LaTeX tables are checkpointed per method.
See [tools/README.md](tools/README.md) for scales and validation rules.

Note: This experiment is expected to take multiple hours depending on your machine.

## 5. Real-Time Controller

You can also run online threshold synthesis + MPPI for the two-oncoming vehicles demo detailed in the paper:

```bash
cd tools/rt_controller
./scripts/run_two_oncoming_threshold_rt.sh
```

Produces CSV logs, plots and the left-turn animation. See the
[RT controller guide](tools/rt_controller/README.md) for deps and options.

Note: the controller uses finite safe-set penalties, not a hard invariant
constraint, and provides no formal closed-loop safety guarantee.

## License

See [LICENSE](LICENSE).
