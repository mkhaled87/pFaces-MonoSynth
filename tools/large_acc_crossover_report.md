# Large ACC crossover proof of concept

## Purpose

This experiment tests two scale-dependent expectations on one larger instance:

1. GPU threshold GFP should outperform the CPU threshold reference.
2. Threshold membership should outperform scan membership inside the otherwise
   identical Automatica frontier algorithm.

This is a deterministic, single-run proof of concept, not a publication-quality
timing study.  It establishes correctness and the existence of both crossovers;
multiple warmed repetitions on the target machine are still required for final
reported performance values.

## Configuration and protocol

The complete protocol is stored in
[`benchmark_configs/acc_large_poc.json`](benchmark_configs/acc_large_poc.json).
It uses the unchanged ACC example dynamics and bounds, with only the grid spacing
overridden:

| Quantity | Value |
|---|---:|
| Grid spacing | `(0.25, 0.125, 0.125)` |
| Grid widths | `(321, 161, 161)` |
| Abstract states | 8,320,641 |
| Threshold columns | 25,921 |
| Transition table | 66,565,128 bytes |
| Warm-up runs | 0 |
| Measured runs | 1 |
| Per-process timeout | 300 s |

All four solvers used the same precomputed `uint64` successor relation,
threshold axis, boundary semantics, and generated configuration.  They are
deterministic, so stochastic seeds do not apply.  The first method generated the
transition cache; later methods reused it.  `solver_ms`, rather than process wall
time, is therefore the fair solver comparison.

Reproduction command:

```sh
python3 tools/run_four_method_benchmark.py \
  --experiment-config tools/benchmark_configs/acc_large_poc.json
```

## Correctness checks

The runner exhaustively checked every adjacent ordered pair in the precomputed
transition relation:

| Check | Result |
|---|---:|
| Adjacent transition pairs checked | 24,832,640 |
| Transition-order violations | 0 |
| GFP rounds, every method | 26 |
| Safe cells, every method | 5,884,036 |
| Output bytes, every method | 103,684 |
| Common SHA-256 | `4ab0a9542ac29fc6e99d6b549a54262c5fb87ea390dc5643990680705c9e451b` |
| Strict acceptance | passed |

Both Automatica variants also executed exactly 1,181 frontier batches and
2,708,833 membership queries.  Thus the scan and threshold variants ran the
same frontier state machine; only their fixed-target membership representation
differed.

## Timing results

| Method | Solver time (ms) | Solver time (s) | Speedup vs. GPU threshold |
|---|---:|---:|---:|
| Automatica + scan membership | 71,063.2 | 71.063 | 2,478.62x |
| Automatica + threshold membership | 68,355.2 | 68.355 | 2,384.16x |
| CPU threshold reference | 134.707 | 0.135 | 4.70x |
| GPU threshold GFP | 28.6705 | 0.029 | 1.00x |

The GPU threshold implementation was **4.70x faster** than the CPU threshold
reference in solver time.  Automatica with threshold membership was **1.040x
faster** than Automatica with scan membership, saving 2.708 s or 3.81% of total
solver time.

The Automatica phase breakdown explains why the second end-to-end crossover is
modest:

| Automatica phase (ms) | Scan | Threshold |
|---|---:|---:|
| Membership | 3,592.36 | 862.751 |
| Threshold construction | 0 | 43.4945 |
| Basis update | 66,238.8 | 66,235.5 |
| Total solver | 71,063.2 | 68,355.2 |

Including its once-per-GFP-round table construction, threshold membership took
906.246 ms versus 3,592.36 ms for scanning, a **3.96x membership-phase
speedup**.  However, the common host-side frontier and basis-update work took
about 66.24 s in both methods.  It therefore dominated total Automatica time and
limited the end-to-end improvement to 3.81%.  Scaling alone does not imply a
large total Automatica speedup when this shared phase dominates.

## Why earlier large attempts stopped

Three distinct events were separated during diagnosis:

- An approximately 128-million-state attempt exceeded the configured basis
  capacity (`max_basis_elements=50000`).  This was a genuine, explicit capacity
  failure, not a correctness result.
- An approximately 16-million-state attempt reached the configured 300-second
  timeout in `automatica_scan`.  It was therefore too large for the requested
  under-five-minute proof of concept on this machine.
- Some initially "silent" stops were caused by the orchestration client losing
  the returned long-running session identifier.  The process was then closed
  with the session; there was no solver crash or operating-system memory kill.

The selected 8.32-million-state case completed both Automatica methods in about
70 seconds, safely below the five-minute limit.

## Correctness fixes made before accepting the result

### Constrained RK4 stages

The exact transition checker exposed a real ACC order violation in the previous
integrator.  For the ordered states

```text
q = (h=80, v_ego=0, v_lead=0.2)
r = (h=80, v_ego=0, v_lead=0.0)
```

the old RK4 code allowed intermediate negative velocity before applying the
state constraint only at the end of integration.  It produced abstract
successors `(1,0,100)` and `(0,0,100)`, respectively, reversing the expected
order in headway.  The general fix projects every constrained RK4 intermediate
stage and every substep result before its next use.  The transition-cache
fingerprint was changed so old caches cannot be reused.  The final large cache
then passed all 24,832,640 adjacent-pair checks.

### Automatica canonicalization

The first correct large run revealed that most Automatica time was spent
re-canonicalizing vectors already known mathematically to be antichains.  The
implementation now uses a checked `assign_antichain` path for:

- the controlled subset of the current canonical work basis; and
- the next frontier, which is obtained by filtering that same basis.

Filtering an antichain preserves incomparability, and the controlled/frontier
union is disjoint by construction.  Debug builds still verify this precondition
pairwise.  This removes redundant quadratic dominance work without changing the
Automatica state machine, membership decisions, iterates, or output.

## Conclusion

The experiment supports both narrow claims on this machine and grid:

- GPU threshold GFP crossed over clearly against the CPU reference (4.70x).
- Threshold membership crossed over against basis scanning inside the same
  Automatica algorithm (3.96x for the representation-dependent phase), but the
  complete Automatica solver improved by only 3.81% because common sequential
  basis maintenance dominated.

The strongest defensible manuscript interpretation is therefore that threshold
indexing accelerates Automatica membership, while the proposed column-wise GFP
method obtains its much larger end-to-end gain by avoiding the frontier/basis
maintenance schedule altogether.  Final paper timings should use warmed,
repeated runs on the target system and report phase times alongside totals.
