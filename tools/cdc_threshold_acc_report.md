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

The paper configuration was executed on an Apple M1 Pro GPU. The common
successor precomputation took 0.870 ms. The table reports five-run medians after
one warm-up; solver time excludes that common precomputation. Times are in ms.

| Method | Steps | Membership | Representation | Threshold maintenance | Basis/frontier update | GFP total | Allocated buffers |
|---|---:|---:|---:|---:|---:|---:|---:|
| CDC | 129/67,550 | 45,337.100 | 0.000 | 0.000 | 3,975.430 | 51,652.200 | 2.26 MiB |
| CDC + threshold (host) | 129/67,550 | 4.669 | 0.003 | 21.929 | 16.416 | 52.295 | 1.12 MiB |
| Automatica scan | 33/398 | 224.509 | 11.203 | 0.000 | 240.447 | 514.770 | 2.26 MiB |
| Automatica + threshold | 33/398 | 209.816 | 36.560 | 0.000 | 240.229 | 521.423 | 2.27 MiB |
| Threshold GFP | 33 | 18.593 | 0.840 | 0.000 | 0.000 | 19.403 | 1.13 MiB |
| Bitmap GFP | 33 | 63.215 | 0.579 | 0.000 | 0.000 | 63.810 | 1.16 MiB |

The host CDC-threshold backend was selected because its median was 52.295 ms,
versus 45,686.500 ms for the exact GPU backend. The GPU backend's fine-grained
launch and synchronization costs dominate its 27,240.200 ms membership and
16,195.500 ms threshold-maintenance phases. The hidden CPU-threshold oracle
had a 3.898 ms median on this moderate grid; it is a correctness reference and
is not included in the visible six-method table.

## Independent result checks

- All 48 solver runs completed successfully (eight variants, one warm-up and
  five measured repetitions).
- All 49 canonical outputs, including the transition-cache preparation run,
  have 6,724 bytes and SHA-256
  `8ff06923de1e88e2de1edffae49e598c2eb1c5798ed7c8f85b28f04a9640c04e`.
- Every method reports exactly 68,611 safe cells.
- CDC scan and both CDC-threshold backends have identical 129-pass traces and
  exactly 67,550 mutations in every run.
- Both Automatica implementations have 33 outer rounds and 398 frontier
  batches. Automatica, threshold GFP, bitmap GFP, and the CPU oracle all have
  33 outer GFP rounds.
- The shared transition table contains 136,161 states and has zero violations
  over 400,160 adjacent monotonicity checks.
- The runner independently accepted the complete output bytes, counts, traces,
  and counters before writing the summary and paper-table fragment.

Generated CSV, JSON, logs, canonical outputs, pass hashes, transition-cache
validation, and the ready-to-paste LaTeX rows are under
`tools/benchmark_results/acc_six_method_paper/`.
