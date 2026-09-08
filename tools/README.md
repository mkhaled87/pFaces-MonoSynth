# pFaces-MonoSynth Benchmark and Visualization Tools

## Solver benchmark

Use `run_solver_benchmark.py` for production comparisons:

```bash
python3 tools/run_solver_benchmark.py \
  examples/acc/acc.cfg \
  examples/turn_ego_first/turn_ego_first.cfg
```

The default runner includes both plain-CDC backends, both CDC-threshold
backends, both Automatica variants, and threshold GFP. It selects the widest
grid axis with a lowest-index tie break, uses
precomputed 64-bit successors for every method, preserves the base
configuration's explicit boundary semantics,
performs one warm-up plus five measured runs, and rejects the case if
the canonical threshold SHA-256 hashes or safe-cell counts differ. Generated
configs, logs, canonical outputs, raw CSV measurements, and a JSON summary are
stored under `tools/benchmark_results/solver_benchmark` by default.
Each case also emits `CASE.paper_table.tex`, retaining all four CDC calibration
variants and formatting the median phase breakdown for the paper.
Before accepting a row, the runner also checks every adjacent transition for
order preservation, requires stable counters across repetitions, requires the
same outer-round count for the synchronous methods, requires identical CDC
pass traces across host/GPU scan and host/GPU threshold execution, and
requires the same frontier-batch
count for the two Automatica membership backends.

Pass `--include-references` to run `bitmap_reference` and
`threshold_cpu_reference` with the same transition cache, warm-up, five measured
runs, output-equality gate, and timing boundary. They are marked
`reference_only` in the JSON summary and are not counted among the five
production algorithms.

For a fully file-driven run, pass a JSON experiment specification. It controls
the base configurations, method subset, common grid resolution, device,
warm-ups, measured runs, timeout, verbosity, and output directory:

```bash
python3 tools/run_solver_benchmark.py \
  --experiment-config tools/benchmark_configs/acc_six_method_paper.json
```

The solvers are deterministic, so there is no algorithmic random seed. Use
`measured_runs` to control independent timing repetitions; the summary records
this explicitly instead of presenting repetitions as stochastic seeds.

For a quick production-kernel comparison on a coarser version of a case, pass
one resolution per state dimension, for example `--state-eta 4,2,2` for ACC.
The override is applied identically to every selected method.
Strict validation is the default. `--allow-nonmonotone-diagnostic` is available
only to diagnose legacy paper examples; its JSON summary sets
`strict_acceptance=false`, and those timings must not be used as paper evidence.

## Full paper scale matrix

Use `run_full_paper_matrix.py` on the paper GPU to progress through ACC,
ACC-5D, Turn-Ego, and Turn-Oncoming scales. By default it compares exactly six
implementations: host CDC scan, host CDC-threshold, both GPU Automatica
representations, bitmap GFP, and threshold GFP (ours). Threshold GFP always
uses inline dynamics in this matrix. All eight calibration variants remain
addressable through `--methods`.

The exact runner keys are:

```text
cdc_host
cdc_gpu
cdc_threshold_host
cdc_threshold_gpu
automatica_scan
automatica_threshold
threshold
bitmap_reference
```

```bash
python3 tools/run_full_paper_matrix.py \
  --pfaces /path/to/pfaces \
  --device 0
```

Choose an arbitrary target scale with `--N`, where the nominal state count is
`|X| = 10^N`. For example, this runs the six defaults once for ACC at `10^8`
with no warm-up:

```bash
python3 tools/run_full_paper_matrix.py \
  --examples acc \
  --N 8 \
  --pfaces /home/yasin/pFaces/bin/pfaces \
  --device 1 \
  --warmups 0 \
  --repetitions 1 \
  --output tools/benchmark_results/acc_1e8_six_methods
```

Change both `--N` and the output directory for another experiment. Multiple
scales can be requested as `--N 7,8,9`. For exponents without a hand-tuned
paper grid, the runner creates a near-isotropic grid whose actual cell count is
recorded in `run_plan.csv`. Existing resource caps remain active, so methods
that cannot fit at a large scale are reported as `skipped_resource` rather than
silently omitted.

The experiment matrix, method list, repetition policy, timeouts, and resource
caps are deliberately grouped at the top of the script. Base example files are
read-only; all derived configurations and results go to
`tools/benchmark_results/full_paper_optimized/`. The run checkpoints after
every method and resumes by default. Use a different `--output` directory after
changing any experiment definition.

The default performs no warm-up and one measured run with a 1,200-second
per-run timeout. Scales are always processed in ascending order. A method
advances within an example only when its measured wall time is at most 60
seconds. A slower run, timeout, or failure produces `skipped_progression` rows
at every larger scale for that example; other methods continue independently.
Both limits can be overridden with `--timeout` and `--max-advance-seconds`.
Resume reconstructs the same decisions from `all_runs.csv`.

The built-in ladders are:

```text
ACC:             6,7,8,9,10,12,14
ACC-5D:          6,7,8,9,11
Turn-Ego:        6,7,8,9,10,12,14
Turn-Oncoming:   6,7,8,9,10,12,14
```

Run the complete progressive experiment with:

```bash
python3 tools/run_full_paper_matrix.py \
  --pfaces /home/yasin/pFaces/bin/pfaces \
  --device 1 \
  --warmups 0 \
  --repetitions 1 \
  --timeout 1200 \
  --max-advance-seconds 60 \
  --output tools/benchmark_results/progressive_six_methods_1min
```

The runner creates `run_plan.csv`, raw `all_runs.csv`, a combined summary CSV,
one large Markdown table, a landscape LaTeX longtable, per-run logs, a manifest
with input hashes, and per-case validation metadata. Successful methods must
match complete canonical hashes and algorithm-specific counters. Methods that
cannot fit a precomputed successor table, threshold table, or bitmap under the
configured caps remain visible as `skipped_resource`. Threshold GFP and bitmap
GFP always use inline dynamics in this matrix. Thus transition backend
selection is explicit in every row.

For every case with a precomputed method, the runner first launches the utility
mode `synthesis_method="precompute_only"`. This mode executes only the parallel
transition kernel, reads and saves the successor table, emits no controller,
and exits before any solver schedule. `case_metadata.json` and the combined CSV
record its kernel/read time, total process wall time, validation result, and
log independently. CDC and Automatica then load that exact cache; their own
logs must show no transition-kernel launch. Inline threshold and bitmap methods
do not load it. Validated caches are retained on disk so an interrupted run or
later analysis can reuse the same table without recomputing it.

`run_plan.csv` records the nominal scale, exact grid widths, actual cell count,
threshold entries, designated axis, transition backend, and estimated major
buffer sizes for every row. The nominal labels such as `1e14` are therefore not
substitutes for the generated geometry. Timing summaries separate shared
transition construction, membership, representation construction, threshold
maintenance, basis/frontier updates, total fixed-point time, wall time, and
allocated solver buffers.

A case is accepted only when successful methods agree on complete canonical
threshold bytes, SHA-256, and safe-cell count. Host/GPU plain CDC and both
CDC-threshold backends must also agree on passes, mutations, and pass hashes;
both Automatica variants must agree on rounds and frontier batches; synchronous methods must
agree on GFP rounds. Precomputed transition caches receive geometry and file
size checks and, below the configured audit cap, an exhaustive adjacent-state
monotonicity check.

Inspect the complete plan without launching pFaces:

```bash
python3 tools/run_full_paper_matrix.py --dry-run --output /tmp/paper-matrix-plan
```

Resume an interrupted run by repeating the identical command and output path.
The manifest fingerprint prevents accidental resume after changing the code,
matrix, device, repetition policy, or resource limits. Use `--no-resume` only
when targeting a new, empty output directory.

`benchmark_bitmap_gfp_acc.py` and `check_bitmap_gfp_acc.py` remain focused tools
for the explicit `bitmap_reference` and `threshold_cpu_reference` modes.
The equality checker also accepts a custom monotone fixture through
`--cfg PATH --output-dir PATH`.

`threshold_cpu_reference` is intentionally omitted from the default visible
paper matrix. Run `run_solver_benchmark.py --include-references` when an
independent CPU threshold oracle is desired on a manageable grid.

## Video generation

This tool generates 3D visualizations of the safe set evolution during the monotonicity-based synthesis process.

## Usage

The script `generate_video.py` is located in the `tools/` directory. It requires a `basis_coordinates.csv` file (generated by setting `record_basis_evolution = "true"` in your `.cfg` file) and the corresponding pFaces `.cfg` file.

### Basic Setup

```bash
# Automatically finds examples/acc/basis_coordinates.csv and saves to examples/acc/safe_set_evolution.mp4
python tools/generate_video.py --cfg examples/acc/acc.cfg
```

### GIF-only Mode

```bash
# Generates ONLY a GIF and saves to examples/acc/safe_set_evolution.gif
python tools/generate_video.py --cfg examples/acc/acc.cfg --gif
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--cfg` | **Required**. Path to the pFaces `.cfg` file. | N/A |
| `--csv` | Path to the `basis_coordinates.csv`. | `[cfg_folder]/basis_coordinates.csv` |
| `--out` | Output file name. | `[cfg_folder]/safe_set_evolution.[mp4/gif]` |
| `--gif` | Generate **ONLY** a GIF version. | False (generates MP4) |
| `--hold`| Number of frames to hold each iteration. Higher = slower evolution but smooth rotation. | 2 |
| `--fps` | Frames per second for the output video. | 25 |
| `--dpi` | Resolution quality (Dots Per Inch). | 150 |
| `--sample`| Render every Nth iteration. | 1 |
| `--rotations`| Number of 360-degree rotations. | 1 |
| `--cmap` | Matplotlib colormap (e.g., `plasma`, `viridis`, `turbo`). | `plasma` |
| `--theme`| UI theme (`light` or `dark`). | `light` |

## Requirements

- Python 3.x
- `matplotlib`
- `numpy`
- `ffmpeg` (installed on your system for MP4 support)

## Customizing Labels

The script attempts to extract axis labels from the `.cfg` file by looking for a comment like:
`# State Space: [headway, ego_velocity, lead_velocity]`

If not found, it uses generic labels ("State 1", etc.).

## Example: Generating for Turn Ego First

```bash
python tools/generate_video.py \
    --cfg examples/turn_ego_first/turn_ego_first.cfg \
    --csv examples/turn_ego_first/basis_coordinates.csv \
    --out examples/turn_ego_first/turn_ego.mp4 \
    --theme dark \
    --gif
```
