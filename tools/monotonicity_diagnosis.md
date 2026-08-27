# Boundary-monotonicity diagnosis and finer-grid validation

Date: 2026-08-26  
Device: Apple M1 Pro GPU, pFaces 1.3, OpenCL 1.2

## What failed

The abstraction uses one-based internal coordinates ordered componentwise.
Physical axes with priority `0` are reversed when mapped to these coordinates;
priority `1` axes are not reversed. Consequently, if `q <= r`, monotonicity
requires `F(q) <= F(r)`. An out-of-domain successor is represented by
`ULONG_MAX`, the artificial unsafe top element.

Every observed violation had the same form:

```text
q <= r,  F(q) = ULONG_MAX,  F(r) is a valid grid state.
```

Thus a physically better state was classified as unsafe merely because its
successor crossed a favorable grid boundary, while an adjacent worse state
remained in the grid.

### Exact ego-first counterexample

The coarsened grid used `eta=(5,2,5)`, bounds
`[-40,10] x [0,10] x [-40,10]`, and priorities `(0,0,1)`.

| Quantity | Better state `q` | Adjacent worse state `r` |
|---|---:|---:|
| Zero-based flat index | 1 | 12 |
| Internal coordinate | `(2,1,1)` | `(2,2,1)` |
| Physical state `(s_ego,v_ego,s_oncoming)` | `(5,10,-40)` | `(5,8,-40)` |
| Continuous RK4 successor | approximately `(6.0067,10.1334,-38.8)` | approximately `(5.8067,8.1346,-38.8)` |
| Abstract successor | `ULONG_MAX` | internal `(2,2,2)`, represented by `(5,8,-35)` |

Here `q <= r`, but `F(q)` is top while `F(r)` is finite. The cause is the
mismatch between the grid bound `v_ego <= 10` and the dynamics saturation
constant `V_MAX=12`: the better high-velocity state crosses `v_ego=10`.

There were 60 adjacent violations. All 60 occurred along the ego-velocity
dimension and all were `invalid-left/valid-right`; there were no finite-to-
finite order reversals.

### Exact ACC counterexample

The coarsened grid used `eta=(4,2,2)`, bounds
`[0,80] x [0,20] x [0,20]`, and priorities `(0,1,0)`.

| Quantity | Better state `q` | Adjacent worse state `r` |
|---|---:|---:|
| Zero-based flat index | 1 | 2 |
| Internal coordinate | `(2,1,1)` | `(3,1,1)` |
| Physical state `(h,v_ego,v_lead)` | `(76,0,20)` | `(72,0,20)` |
| Continuous RK4 successor | approximately `(83.6560,0,18.2814)` | approximately `(79.6560,0,18.2814)` |
| Abstract successor | `ULONG_MAX` | internal `(2,1,2)`, represented by `(76,0,18)` |

The larger headway is physically favorable, but crossing the artificial upper
bound `h=80` is encoded exactly like an unsafe collision-side exit. There were
85 adjacent violations: 55 along headway, 15 along ego velocity, and 15 along
lead velocity. All 85 were `invalid-left/valid-right`; the common underlying
cause was the favorable headway exit.

## Should the GFP round counts agree?

Yes for `automatica_scan`, `automatica_threshold`, and `threshold`, provided
the transition abstraction is monotone and all three counters include the final
no-change round, as this implementation does. Each outer round then computes
the same map

```text
K_(i+1) = {q in K_i : F(q) is in K_i}.
```

Starting from the same `K_0`, equality of every outer iterate follows by
induction. The two Automatica modes also execute the same host state machine,
so their frontier-batch counts must agree.

CDC epochs are not comparable: one CDC epoch commits one unsafe maximal-element
deletion, whereas one outer GFP round computes a complete predecessor update.
Only CDC's final set is required to agree.

The original coarsened ACC counts `30/30/28` therefore indicated a violated
threshold-prefix/binary-search premise. The two Automatica modes still matched
each other because only their membership representation differs. Their final
agreement with the proposed method was coincidental and was not sufficient to
validate the nonmonotone case.

## Recommended boundary semantics

The general fix is to distinguish favorable and unfavorable domain exits using
the declared axis priority:

| Priority | Favorable exit | Unfavorable exit |
|---|---|---|
| `0` (larger physical value is better) | `x > ub`: saturate to `ub` | `x < lb`: unsafe sink |
| `1` (smaller physical value is better) | `x < lb`: saturate to `lb` | `x > ub`: unsafe sink |

The production implementation now provides this rule as
`boundary_semantics=favorable_saturating`. Coordinate-wise, its projection is

```text
priority 0: P_i(x) = unsafe if x < lb_i, otherwise min(x, ub_i)
priority 1: P_i(x) = unsafe if x > ub_i, otherwise max(x, lb_i).
```

If any coordinate takes its unfavorable exit, the complete successor is the
existing artificial unsafe top state (`ULONG_MAX`). Otherwise, each favorable
exit is projected to its corresponding in-domain boundary while every other
coordinate is retained. This is preferable to a single global favorable sink:
crossing a favorable boundary in one coordinate does not make the remaining
safety-relevant coordinates irrelevant.

The scalar projection is order preserving in the declared priority order.
Consequently, if the continuous successor map is order preserving, its
composition with this product projection is also order preserving. This does
not make arbitrary dynamics monotone; the benchmark runner still validates
every adjacent abstract transition and rejects any violation.

The projection is also a modeling commitment: it is sound only when values
beyond a favorable artificial bound are intended to be at least as safe as the
boundary representative. When every physical domain exit is genuinely unsafe,
`boundary_semantics=strict_unsafe` remains available; the grid must then be
chosen so that the resulting abstraction satisfies the required order.

The rule is implemented once in `state_to_idx`, so it is shared by precomputed
transitions and the inline threshold/bitmap paths. The selected boundary mode
is part of the transition-cache fingerprint. All eight tracked example
configurations now explicitly select `favorable_saturating`; no example
dynamics or bounds were changed. The real-time `SafeSet` query path follows the
same rule: favorable continuous query exits are clamped, while unfavorable
exits return unsafe instead of being silently clamped into the table.

## Finer-grid stress tests

These are single-run correctness stress tests using a shared precomputed
transition table; they are not five-run paper timing estimates. Result
extraction and transition precomputation are excluded from `solver_ms`.

| Case | Grid | CDC | Automatica scan | Automatica threshold | Threshold | GFP rounds | Safe cells | Adjacent violations |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Oncoming-first | `101x11x101 = 112,211` | 52.655 s | 2.733 s | 2.686 s | 29.295 ms | `62/62/62` | 7,111 | 0 of 324,210 |
| Ego-first | `101x11x101 = 112,211` | 53.322 s | 1.061 s | 1.060 s | 11.054 ms | `22/22/22` | 5,948 | 0 of 324,210 |
| ACC | `81x41x41 = 136,161` | 37.609 s | 750.497 ms | 753.856 ms | 16.074 ms | `33/33/33` | 68,572 | 0 of 400,160 |

For each row, the four production methods produced byte-for-byte identical
canonical threshold tables:

| Case | Complete SHA-256 |
|---|---|
| Oncoming-first | `5a22eb91e7b7fa499b81f019a327875ae0d0d50431f5493be3d1f17bdc46bb85` |
| Ego-first | `8e4665e5ae50c041b5c4ecf37bd5eea3239854691e72157b076919ade53680cc` |
| ACC | `a4b8416497a43feb2b14c6b0293830bff0a9f14f5f9d70b71dceca35400b0ea4` |

The independent full-grid references produced the same complete hashes. Their
single-run solver times were:

| Case | Bitmap reference | CPU threshold reference | GFP rounds | Safe cells |
|---|---:|---:|---:|---:|
| Oncoming-first | 107.023 ms | 4.607 ms | 62 | 7,111 |
| Ego-first | 39.944 ms | 1.521 ms | 22 | 5,948 |
| ACC | 66.546 ms | 4.212 ms | 33 | 68,572 |

These are correctness references, not additional production baselines. The CPU
reference is particularly effective at these medium table sizes because it
avoids per-round GPU scheduling and synchronization; this result should not be
extrapolated to the paper's very large tables.

## Why the two Automatica times are nearly equal

The threshold lookup has a better per-query asymptotic cost, but both methods
execute exactly the same outer GFP and inner frontier state machine. The
measured phase costs show what remains unchanged:

Let $D$ be the dimension, $B$ the fixed target-basis size in one outer
round, $Q$ the total number of frontier queries in that round, $J$ the
number of sequential frontier batches, and
$T=\prod_{d\ne d^*}N_d$ the threshold-table size. Ignoring hardware constants:

| Backend | Membership work per outer round | Extra representation work | Sequential batch depth |
|---|---:|---:|---:|
| Scan | `Theta(Q B D)` | none | `Theta(J)` |
| Threshold | `Theta(Q D)` | `Theta(B D + (D-1)T)` | `Theta(J)` |

The scan implementation assigns 64 work-items to each query, so its idealized
membership depth is closer to `ceil(B/64) D` per batch than its total-work
bound suggests. The threshold representation has the better asymptote when
`Q B D` dominates table construction, but it cannot reduce `J` or the shared
host basis-update work.

| Case and backend | Solver | Membership phase | Table construction | Basis update | Pure predecessor kernel |
|---|---:|---:|---:|---:|---:|
| Ego, scan | 1,061.090 ms | 895.445 ms | 0 | 138.758 ms | 1.809 ms |
| Ego, threshold | 1,059.980 ms | 882.637 ms | 19.359 ms | 136.807 ms | 1.641 ms |
| ACC, scan | 750.497 ms | 203.941 ms | 0 | 293.286 ms | 0.838 ms |
| ACC, threshold | 753.856 ms | 182.511 ms | 33.571 ms | 293.654 ms | 0.363 ms |
| Oncoming, scan | 2,732.640 ms | 2,564.390 ms | 0 | 130.572 ms | 5.512 ms |
| Oncoming, threshold | 2,685.550 ms | 2,478.500 ms | 57.517 ms | 129.040 ms | 4.362 ms |

Here `membership_ms` is phase wall time, including transfers and
synchronization; the final column isolates profiler-reported device execution.
The two variants also performed identical work counts: 2,275/2,275 frontier
batches and 106,809/106,809 queries for ego, 396/396 and 91,461/91,461 for ACC,
and 6,420/6,420 and 105,313/105,313 for oncoming.

The result is therefore expected for these grids:

1. the scan kernel distributes each sparse-basis scan over a 64-work-item
   workgroup, so the measured device scan is already very small;
2. the threshold variant removes that scan but does not remove the many
   sequential frontier batches, device synchronizations, flag transfers, or
   host antichain updates;
3. it additionally clears, scatters, and prefix-propagates a threshold table
   once per outer round; and
4. one work-item per threshold query can expose less GPU parallelism than the
   64-work-item scan for small frontier batches.

Thus constant-time membership is not a guarantee of lower end-to-end time. It
should win only when avoided basis-scan work is large enough to amortize table
construction and the common frontier overhead. The proposed column-wise GFP is
much faster here because it removes the frontier schedule itself, not merely
because it changes the membership data structure.

## Acceptance rule for future experiments

A paper timing row should be accepted only if all of the following hold:

1. every adjacent transition pair passes the order check;
2. both Automatica modes have identical frontier-batch counts;
3. methods 2--4 have identical outer GFP round counts;
4. all four production outputs have identical complete bytes and safe-cell
   counts;
5. representative small and medium cases also match the full-grid bitmap
   reference.
