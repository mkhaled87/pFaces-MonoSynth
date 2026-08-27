# CDC-Threshold and ACC Benchmark Status

Date: 2026-08-27

## Reproducible protocol

The validated four-method checkpoint is commit `e8f99e2` on
`baseline/four-method-validated-2026-08-27`. Development continues locally on
`feature/cdc-threshold`; neither branch has been pushed.

The paper comparison is fully specified by:

```bash
python3 tools/run_solver_benchmark.py \
  --experiment-config tools/benchmark_configs/acc_six_method_paper.json
```

This uses the unchanged ACC example with resolution `1.0,0.5,0.5`, hence an
`81x41x41` grid (136,161 states). It prepares and validates one shared
precomputed successor cache, performs one warm-up and five measured runs in a
deterministically rotated order, and runs:

1. CDC scan;
2. CDC-threshold on the host;
3. CDC-threshold on the GPU;
4. Automatica scan;
5. Automatica threshold;
6. threshold GFP;
7. bitmap GFP;
8. a hidden CPU-threshold correctness oracle.

The visible CDC-threshold row is selected only after the run, using the lower
five-run median of its two validated backends. Every accepted row must have the
same complete threshold bytes and safe-cell count. CDC scan and both
CDC-threshold backends must additionally have identical pass counts, mutation
counts, and pass-level basis hashes.

The scalable stress configuration is:

```bash
python3 tools/run_solver_benchmark.py \
  --experiment-config tools/benchmark_configs/acc_large_scalable_six_method.json
```

It preserves the previous `321x161x161` grid (8,320,641 states), excludes scan
CDC, uses one measured run, and labels explicitly permitted timeouts as
incomplete. The provenance configuration `acc_large_poc.json` is unchanged.

## Validation completed here

- C++ driver and tests build successfully.
- CTest passes `basis_store` and `solver_method_oracle`.
- The independent Python oracle exhaustively checks all 84 monotone `2x2`
  single-successor systems.
- CDC scan, host CDC-threshold, and modeled GPU CDC-threshold have identical
  final sets, pass traces, and mutation counts in the exhaustive oracle.
- Each threshold deletion is checked against the explicit lower set; the
  complete immediate-successor maximality predicate and projection-indexed
  scratch basis are checked after every pass.
- Both tracked ACC experiment specifications dry-run to their intended grids.
- `reviews/main copy.tex` compiles to six pages with no overfull boxes.

## Numerical results

No numerical row is recorded yet. This execution environment reports no
suitable OpenCL CPU or GPU device, so production-kernel timing and GPU-backend
trace validation cannot be performed here. The manuscript table deliberately
contains evidence-gated entries rather than stale or inferred measurements.

| Method | Steps | Membership | Representation | Threshold maintenance | Basis/frontier update | GFP total | Allocated buffers |
|---|---:|---:|---:|---:|---:|---:|---:|
| CDC | pending | pending | pending | -- | pending | pending | pending |
| CDC + threshold | pending | pending | pending | pending | pending | pending | pending |
| Automatica scan | pending | pending | pending | -- | pending | pending | pending |
| Automatica + threshold | pending | pending | pending | -- | pending | pending | pending |
| Threshold GFP | pending | pending | pending | -- | -- | pending | pending |
| Bitmap GFP | pending | pending | pending | -- | -- | pending | pending |

Generated CSV, JSON, logs, canonical outputs, pass hashes, and transition-cache
validation are written under the output directory named by each JSON file.
