# Four-method solver proof-of-concept report

Date: 2026-08-26  
Device: Apple M1 Pro GPU, pFaces 1.3, OpenCL 1.2  
Protocol: one warm-up followed by five measured runs; shared precomputed
64-bit successor table; median solver time reported; result extraction and
the final threshold-table read excluded from solver time.

## Implementation assessment

The four methods share the same dynamics, transition table, grid, threshold
axis, canonical output format, and timing boundary. Their remaining serial and
parallel work is intrinsic to the selected schedule:

| Method | Parallel work | Required serial work | Low-risk optimizations applied |
|---|---|---|---|
| `cdc` | All current generators are checked concurrently; each query uses a 64-work-item parallel basis scan. | Only the lexicographically first unsafe generator is committed, followed by an immediate restart. Removing this commit dependency would change CDC. | Exact incremental antichain mutation replaces full-basis re-canonicalization; speculative flags after the first unsafe generator are discarded. |
| `automatica_scan` | Every frontier batch is checked concurrently; each query scans the fixed target basis in parallel. | Frontier batches remain sequential because one batch exposes the next. | The target basis is uploaded once per outer GFP round, not once per frontier batch; unsafe frontier generators are removed in one exact batch. |
| `automatica_threshold` | The target threshold table is built by parallel clear/scatter/prefix kernels; every frontier query then performs one parallel constant-time lookup. | It uses exactly the same sequential frontier batches as `automatica_scan`. | The target table is constructed only once per outer GFP round and stays frozen through all inner batches. |
| `threshold` | One work-item per threshold column performs a quick maximum check and, when needed, a binary search. | Only the scalar convergence decision is serial. | Both tables remain resident; only parity and the changed flag move per round; exactly one final table is read after timing. |

This is the fastest minimal implementation found without altering the four
algorithmic contracts. It is not a claim of hardware-level optimality. In
particular, literal CDC necessarily retains one sequential mutation per epoch,
and both Automatica variants necessarily retain sequential frontier batches.

## Timing and equality results

All values below are production GPU executions. `A-scan` and `A-threshold`
denote the two Automatica membership backends. Speedups are relative to the
proposed `threshold` method.

| Case | Grid | Status | Safe cells | CDC ms (x) | A-scan ms (x) | A-threshold ms (x) | Threshold ms | Output SHA-256 prefix |
|---|---:|---|---:|---:|---:|---:|---:|---|
| Monotone fixture, coarse (final rebuild) | 13x9x7 = 819 | Strict pass | 60 | 348.190 (65.65x) | 41.642 (7.85x) | 46.959 (8.85x) | 5.304 | `9ccea060a038` |
| Monotone fixture, fine | 25x17x13 = 5,525 | Strict pass | 480 | 2,302.120 (506.21x) | 87.434 (19.23x) | 89.636 (19.71x) | 4.548 | `38f8c6d5260a` |
| Paper `turn_oncoming_first`, coarsened copy | 11x6x11 = 726 | Strict pass | 126 | 322.328 (89.21x) | 62.391 (17.27x) | 51.935 (14.37x) | 3.613 | `5fed9a301c22` |
| Paper `turn_ego_first`, coarsened copy | 11x6x11 = 726 | Strict pass | 66 | 332.810 (83.86x) | 51.310 (12.93x) | 55.216 (13.91x) | 3.969 | `c5e18241a2da` |
| Paper ACC, coarsened copy | 21x11x11 = 2,541 | Strict pass | 231 | 1,174.690 (84.15x) | 89.237 (6.39x) | 107.127 (7.67x) | 13.959 | `f59fba0a5dba` |

For every row, all four final threshold files had the same complete SHA-256
hash and safe-cell count. The short prefixes are shown only to keep the table
readable.

The optional reference modes were run under the same warm-up/five-run
protocol on the three coarsened paper cases:

| Case | Bitmap-reference median | CPU-threshold-reference median | Reference GFP rounds |
|---|---:|---:|---:|
| `turn_oncoming_first` | 11.791 ms | 0.032 ms | 8 |
| `turn_ego_first` | 11.732 ms | 0.030 ms | 8 |
| ACC | 43.024 ms | 0.225 ms | 31 |

Both references produced the same complete canonical bytes and safe-cell count
as all four production methods. They are correctness references rather than
additional production claims; in particular, the tiny CPU tables avoid the GPU
launch and synchronization overhead that dominates these coarse cases.

## Algorithm counters and validation

| Case | CDC epochs / mutations | GFP rounds: A-scan / A-threshold / threshold | Automatica frontier batches | Adjacent transition-order violations |
|---|---:|---:|---:|---:|
| Monotone fixture, coarse | 760 / 759 | 9 / 9 / 9 | 102 / 102 | 0 of 2,186 |
| Monotone fixture, fine | 5,046 / 5,045 | 9 / 9 / 9 | 207 / 207 | 0 of 15,604 |
| `turn_oncoming_first`, coarsened | 601 / 600 | 8 / 8 / 8 | 104 / 104 | 0 of 1,925 |
| `turn_ego_first`, coarsened | 661 / 660 | 8 / 8 / 8 | 110 / 110 | 0 of 1,925 |
| ACC, coarsened | 2,311 / 2,310 | 31 / 31 / 31 | 187 / 187 | 0 of 7,040 |

The strict rows satisfy all current acceptance gates: an order-preserving
successor table, stable counters across repetitions, identical Automatica
frontier-batch counts, identical methods 2--4 outer-round counts, and identical
final canonical bytes across all four methods.

The coarse monotone row was rerun after the final build from a clean output
directory. All 24 production executions (one warm-up and five measured runs
per method) produced the complete SHA-256 value
`9ccea060a03828cef13649494114c2a4726ac7ffd11f3776832b39e40b2406e2`.
The independent `bitmap_reference` and `threshold_cpu_reference` modes also
produced those exact bytes in a separate GPU smoke test.

The earlier paper-case failures were caused by mapping both favorable and
unfavorable domain exits to the same unsafe top state. The shared transition
projection now saturates favorable per-axis exits to their corresponding
in-domain boundary while retaining the other coordinates, and maps any
unfavorable exit to the unsafe sink. This rule is selected explicitly by
`boundary_semantics=favorable_saturating`; all three rows now pass the adjacent
order check and the stronger outer-iterate check.

## Reproduction on the more powerful machine

The tracked example dynamics and state-space bounds were not modified. Their
configurations now explicitly select the common favorable-boundary semantics.
The runner creates copies in its output directory and changes only
`synthesis_method` between the four production configurations (plus the two
explicit reference modes when `--include-references` is requested).

Start with a full-resolution acceptance run:

```bash
python3 tools/run_solver_benchmark.py \
  examples/turn_oncoming_first/turn_oncoming_first.cfg \
  --device 1 \
  --include-references \
  --output tools/benchmark_results/four_methods/turn_oncoming_full
```

For a quick coarsened proof of concept using the same paper dynamics and bounds:

```bash
python3 tools/run_solver_benchmark.py \
  examples/turn_oncoming_first/turn_oncoming_first.cfg \
  --state-eta 5,2,5 \
  --device 1 \
  --output tools/benchmark_results/four_methods/turn_oncoming_poc
```

The runner aborts before accepting a row if transition monotonicity, repeated
counters, outer-round equality, final SHA-256 hashes, output sizes, or safe-cell
counts disagree. `--allow-nonmonotone-diagnostic` remains available only for
diagnosis and sets `strict_acceptance=false`; such timings should not enter the
paper. The boundary proof obligation, exact former counterexamples, finer-grid
validation, phase timings, and limits of the modeling assumption are documented
in [`monotonicity_diagnosis.md`](monotonicity_diagnosis.md).
