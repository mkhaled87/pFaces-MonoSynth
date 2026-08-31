# pFaces-MonoSafe Threshold Experiments

**Real-time invariant-set synthesis for monotone systems.** This repository
contains the threshold-table and real-time controller experiments
behind the paper implementation.

The method stores one threshold height per grid column along
`threshold_d_star`, so safe-set membership is an `O(1)` lookup with
`O(N^(d-1))` storage instead of full-grid storage or basis scans.

The repository exposes five production solvers with one explicit configuration
key: CDC scan, CDC with threshold indexing, Automatica with scan membership,
Automatica with threshold membership, and the proposed column-wise threshold
GFP. The full paper matrix reports both CDC-threshold execution backends and
the bitmap reference, giving seven visible implementation rows. CPU threshold
remains an optional correctness reference.

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

## Reproduce the Paper Experiments

The paper release is intended for Linux with an NVIDIA OpenCL device and an
external pFaces installation. Clone the release branch, export the SDK path,
and build the kernel driver:

```bash
git clone --branch paper-experiments \
  https://github.com/mkhaled87/pFaces-MonoSynth.git
cd pFaces-MonoSynth

export PFACES_SDK_ROOT=/path/to/pfaces-sdk
export PATH=/path/to/pfaces/bin:$PATH

sh build.sh
pfaces -G -l
```

Replace device `0` below with the NVIDIA device index reported by pFaces.
Inspect the complete 126-row experiment plan without launching a solver:

```bash
python3 tools/run_full_paper_matrix.py \
  --dry-run \
  --device 0 \
  --output /tmp/paper-matrix-plan
```

Run a coarse ACC smoke test before starting the full matrix:

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

The default full-matrix protocol performs one warm-up and one measured run:

```bash
python3 tools/run_full_paper_matrix.py \
  --pfaces pfaces \
  --device 0 \
  --warmups 1 \
  --repetitions 1
```

For final paper statistics, collect five warmed measurements:

```bash
python3 tools/run_full_paper_matrix.py \
  --pfaces pfaces \
  --device 0 \
  --warmups 1 \
  --repetitions 5 \
  --output tools/benchmark_results/full_paper_five_runs
```

Runs checkpoint after every method and resume by default. Reuse exactly the
same command and output directory to continue an interrupted matrix. Generated
configurations, logs, raw measurements, validation metadata, and Markdown and
LaTeX tables are written below `tools/benchmark_results/` and are ignored by
Git. See [tools/README.md](tools/README.md) for the output schema and validation
rules.

The nominal labels $10^8$--$10^{14}$ identify paper-scale cases. The generated
`run_plan.csv` records the exact grid widths, actual cell count, threshold-table
size, selected transition backend, and any explicit resource skip for every
row. Methods requiring a precomputed successor table are not silently changed
to inline dynamics; threshold and bitmap rows may use their supported inline
backend above the configured precomputation cap.

### Changes relative to the submitted implementation

- The benchmark now separates CDC scan, host and GPU CDC-threshold,
  scan-based and threshold-indexed Automatica, threshold GFP, and bitmap GFP.
- CDC uses frozen pass snapshots with immediate committed basis updates;
  Automatica variants share one frontier state machine and differ only in
  fixed-target membership representation.
- Successor indices are 64-bit, and basis operations use a canonical checked
  `BasisStore` rather than fixed coordinate buckets.
- Favorable domain exits use explicit saturating boundary semantics while
  unfavorable exits map to an unsafe sink. Constrained Runge--Kutta stages are
  projected before reuse so the generated transition relation preserves the
  required order.
- Benchmarks report transition construction, membership, representation,
  threshold maintenance, basis/frontier updates, total solver time, and
  allocated solver buffers separately.
- Complete canonical outputs, safe-cell counts, algorithm-specific counters,
  and transition monotonicity are checked before a result is accepted. Round
  counts remain algorithm-specific and are not treated as equivalent work.
- The online controller demonstration is safety-informed MPPI; it does not
  claim a formal closed-loop safety guarantee.

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

Run standalone threshold synthesis:

```bash
./run_threshold_synthesis.sh --cfg examples/acc/acc.cfg --mode threshold --device-class G --device 1
```

The script defaults to synthesis only. The immediate CDC schedule is selected by:

```bash
./run_threshold_synthesis.sh --cfg examples/acc/acc.cfg --mode cdc --device-class G --device 1
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

The tracked paper/RT configurations explicitly select
`boundary_semantics=favorable_saturating`. The RT adapter selects
`synthesis_method=threshold` with inline dynamics at launch. On the verified RTX 5090
machine, the default 10.1B-cell phase tables stayed below 100 ms per online
synthesis.

## Bitmap GFP

Validate the explicit bitmap and CPU-threshold reference modes on ACC:

```bash
python3 tools/check_bitmap_gfp_acc.py
```

Benchmark ACC at large bitmap/threshold grids:

```bash
python3 tools/benchmark_bitmap_gfp_acc.py --sizes 1e8 1e9
```

Run the fair five-production-method benchmark (one warm-up and five measured runs):

```bash
python3 tools/run_solver_benchmark.py --help
```

Add `--include-references` to record the bitmap and CPU-threshold reference
implementations under the same transition cache, repetitions, timing boundary,
and exact-output checks. They remain labeled reference-only in the summary.
`--experiment-config PATH` loads the method subset, state resolution, device,
warm-up count, measured-run count, timeout, and output directory from one JSON
file; see `tools/benchmark_configs/acc_large_poc.json`.

## Solver configuration

New or generated configurations select exactly one method:

```text
synthesis_method = "threshold";
transition_semantics = "extremal_single_successor";
transition_backend = "precomputed";
cdc_threshold_backend = "host";
boundary_semantics = "favorable_saturating";
threshold_d_star = "-1";
```

Production values are `cdc`, `cdc_threshold`, `automatica_scan`,
`automatica_threshold`, and `threshold`. `cdc_threshold_backend` explicitly
selects `host` or `gpu`. Reference-only values are `bitmap_reference` and
`threshold_cpu_reference`. Legacy `use_*` solver booleans are rejected rather
than translated implicitly. Benchmark and RT launchers generate or apply
explicit enum-based overrides. Fair solver timing uses `precomputed` for every
method; `inline` is reserved for threshold RT/scalability runs and the bitmap
reference. The CPU-threshold reference requires precomputed successors.
`boundary_semantics=favorable_saturating` projects favorable per-axis exits to
the corresponding boundary and maps any unfavorable exit to the unsafe sink.
Use `strict_unsafe` only when every domain exit is intentionally unsafe.
