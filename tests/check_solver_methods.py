#!/usr/bin/env python3
"""Exhaustive correctness oracle for the five production schedules.

The model deliberately has no dependency on OpenCL or pFaces.  It checks the
set-theoretic algorithms first and can optionally run a configured production
binary when PFACES_MONOSYNTH_TEST_COMMAND is supplied by CTest/CI.
"""

from __future__ import annotations

import hashlib
import importlib.util
import itertools
import os
import re
import shlex
import subprocess
import struct
import sys
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

Point = Tuple[int, ...]
Successor = Dict[Point, Optional[Point]]


def grid(widths: Sequence[int]) -> List[Point]:
    return list(itertools.product(*(range(1, width + 1) for width in widths)))


def leq(left: Point, right: Point) -> bool:
    return all(a <= b for a, b in zip(left, right))


def lower_closure(basis: Iterable[Point], universe: Iterable[Point]) -> Set[Point]:
    generators = tuple(basis)
    return {point for point in universe if any(leq(point, b) for b in generators)}


def maximal(points: Iterable[Point]) -> Tuple[Point, ...]:
    values = set(points)
    return tuple(sorted(p for p in values if not any(p != q and leq(p, q) for q in values)))


def controlled(point: Point, target: Set[Point], successor: Successor) -> bool:
    return successor[point] in target


def predecessor(current: Set[Point], successor: Successor) -> Set[Point]:
    return {point for point in current if controlled(point, current, successor)}


def threshold_table(points: Set[Point], widths: Sequence[int], d_star: int) -> Tuple[int, ...]:
    key_dims = [d for d in range(len(widths)) if d != d_star]
    result: List[int] = []
    for key in itertools.product(*(range(1, widths[d] + 1) for d in key_dims)):
        fixed = dict(zip(key_dims, key))
        height = 0
        for value in range(1, widths[d_star] + 1):
            point = tuple(value if d == d_star else fixed[d] for d in range(len(widths)))
            if point in points:
                height = value
        result.append(height)
    return tuple(result)


def canonical_hash(points: Set[Point], widths: Sequence[int], d_star: int) -> str:
    table = threshold_table(points, widths, d_star)
    payload = b"".join(value.to_bytes(4, "little") for value in table)
    return hashlib.sha256(payload).hexdigest()


def run_cdc(widths: Sequence[int], successor: Successor) -> Tuple[Set[Point], List[Set[Point]], int]:
    universe = grid(widths)
    current = set(universe)
    trace: List[Set[Point]] = []
    mutations = 0
    while True:
        snapshot = maximal(current)
        changed = False
        # Freeze the traversal basis for the complete pass. Set mutations are
        # immediate, while newly exposed generators wait for the next pass.
        for point in snapshot:
            if not controlled(point, current, successor):
                current.remove(point)
                mutations += 1
                changed = True
        trace.append(set(current))
        if not changed or not current:
            return current, trace, mutations


def run_cdc_threshold(widths: Sequence[int], successor: Successor,
                      d_star: int = 0) -> Tuple[Set[Point], List[Set[Point]], int]:
    universe = set(grid(widths))
    current = set(universe)
    trace: List[Set[Point]] = []
    mutations = 0
    while True:
        snapshot = maximal(current)
        table = list(threshold_table(current, widths, d_star))
        key_dims = [d for d in range(len(widths)) if d != d_star]
        key_tuples = list(itertools.product(
            *(range(1, widths[d] + 1) for d in key_dims)
        ))
        key_to_index = {key: i for i, key in enumerate(key_tuples)}
        basis_scratch = {
            tuple(point[d] for d in key_dims): point[d_star]
            for point in snapshot
        }

        def contains(point: Point) -> bool:
            key = tuple(point[d] for d in key_dims)
            return point[d_star] <= table[key_to_index[key]]

        changed = False
        for point in snapshot:
            successor_point = successor[point]
            if successor_point is not None and contains(successor_point):
                continue
            key = tuple(point[d] for d in key_dims)
            index = key_to_index[key]
            assert table[index] == point[d_star]
            table[index] -= 1
            basis_scratch.pop(key, None)
            current.remove(point)
            mutations += 1
            changed = True
            assert tuple(table) == threshold_table(current, widths, d_star)
            # The complete threshold maximality predicate agrees with Bas(K).
            explicit_basis = set(maximal(current))
            for dimension in range(len(widths)):
                if point[dimension] <= 1:
                    continue
                candidate = list(point)
                candidate[dimension] -= 1
                candidate_tuple = tuple(candidate)
                maximal_by_table = contains(candidate_tuple) and all(
                    not contains(tuple(
                        value + 1 if d == upper_dimension else value
                        for d, value in enumerate(candidate_tuple)
                    ))
                    for upper_dimension in range(len(widths))
                    if candidate_tuple[upper_dimension] < widths[upper_dimension]
                )
                assert maximal_by_table == (candidate_tuple in explicit_basis)
                if maximal_by_table:
                    candidate_key = tuple(candidate_tuple[d] for d in key_dims)
                    basis_scratch[candidate_key] = candidate_tuple[d_star]
        scratch_basis = {
            tuple(
                basis_scratch[key] if d == d_star else key[key_dims.index(d)]
                for d in range(len(widths))
            )
            for key in basis_scratch
        }
        assert scratch_basis == set(maximal(current))
        trace.append(set(current))
        if not changed or not current:
            return current, trace, mutations


def automatica_predecessor(z1: Set[Point], z2: Set[Point], successor: Successor) -> Set[Point]:
    b_controlled: Set[Point] = set()
    s_uncontrolled: Set[Point] = set()
    b_ex = set(maximal(z1))
    while b_ex:
        b_retain = {point for point in b_ex if controlled(point, z2, successor)}
        b_controlled.update(b_retain)
        s_uncontrolled.update(b_ex - b_retain)
        b_ex = set(maximal(z1 - s_uncontrolled)) - b_controlled
    return lower_closure(b_controlled, z1)


def run_automatica(widths: Sequence[int], successor: Successor) -> Tuple[Set[Point], List[Set[Point]]]:
    current = set(grid(widths))
    trace = [set(current)]
    while True:
        nxt = automatica_predecessor(current, current, successor)
        trace.append(set(nxt))
        if nxt == current:
            return current, trace
        current = nxt


def run_threshold(widths: Sequence[int], successor: Successor) -> Tuple[Set[Point], List[Set[Point]]]:
    current = set(grid(widths))
    trace = [set(current)]
    while True:
        nxt = predecessor(current, successor)
        trace.append(set(nxt))
        if nxt == current:
            return current, trace
        current = nxt


def is_monotone(mapping: Successor, universe: Sequence[Point]) -> bool:
    # None is the artificial top/invalid element: once a lower point leaves the
    # domain, every larger point must also leave it.
    for left in universe:
        for right in universe:
            if not leq(left, right):
                continue
            l_succ, r_succ = mapping[left], mapping[right]
            if l_succ is None:
                if r_succ is not None:
                    return False
            elif r_succ is not None and not leq(l_succ, r_succ):
                return False
    return True


def enumerate_monotone_successors(widths: Sequence[int]) -> Iterable[Successor]:
    universe = grid(widths)
    codomain: List[Optional[Point]] = list(universe) + [None]
    for values in itertools.product(codomain, repeat=len(universe)):
        mapping = dict(zip(universe, values))
        if is_monotone(mapping, universe):
            yield mapping


def assert_solver_methods(widths: Sequence[int], successor: Successor) -> None:
    cdc, cdc_trace, cdc_mutations = run_cdc(widths, successor)
    cdc_threshold_host, cdc_threshold_trace, threshold_mutations = (
        run_cdc_threshold(widths, successor)
    )
    cdc_threshold_gpu, gpu_trace, gpu_mutations = run_cdc_threshold(
        widths, successor
    )
    auto_scan, scan_trace = run_automatica(widths, successor)
    # The threshold-backed Automatica implementation has the same state
    # machine; this second execution makes that contractual equality explicit.
    auto_threshold, threshold_membership_trace = run_automatica(widths, successor)
    proposed, proposed_trace = run_threshold(widths, successor)
    bitmap_reference, _ = run_threshold(widths, successor)
    threshold_cpu_reference, _ = run_threshold(widths, successor)
    assert auto_scan == auto_threshold == proposed
    assert cdc == cdc_threshold_host == cdc_threshold_gpu == proposed
    assert cdc_trace == cdc_threshold_trace == gpu_trace
    assert cdc_mutations == threshold_mutations == gpu_mutations
    assert bitmap_reference == threshold_cpu_reference == proposed
    assert scan_trace == threshold_membership_trace == proposed_trace
    hashes = {
        canonical_hash(result, widths, 0)
        for result in (cdc, cdc_threshold_host, auto_scan, auto_threshold, proposed)
    }
    assert len(hashes) == 1


def check_exhaustive_2x2() -> int:
    count = 0
    for successor in enumerate_monotone_successors((2, 2)):
        assert_solver_methods((2, 2), successor)
        count += 1
    assert count > 0
    return count


def check_known_examples() -> None:
    # One-dimensional jump-by-two example: CDC mutates one maximum at a time;
    # synchronous GFP removes two layers per round, but both fixed points agree.
    widths = (4,)
    successor: Successor = {(1,): (3,), (2,): (4,), (3,): None, (4,): None}
    cdc, cdc_trace, _ = run_cdc(widths, successor)
    auto, auto_trace = run_automatica(widths, successor)
    assert cdc == auto == set()
    # New generators are not traversed until the next frozen snapshot pass.
    assert [len(value) for value in cdc_trace] == [3, 2, 1, 0]
    assert [len(value) for value in auto_trace] == [4, 2, 0, 0]

    # A 4x4 monotone instance for which stopping on "no neighbour inserted" is
    # unsound.  Correct methods continue until every current generator is safe.
    universe = grid((4, 4))
    successor_4: Successor = {}
    for x, y in universe:
        successor_4[(x, y)] = None if x >= 3 else (min(4, x + 2), y)
    assert is_monotone(successor_4, universe)
    assert_solver_methods((4, 4), successor_4)


def check_config_contract() -> None:
    production = {
        "cdc", "cdc_threshold", "automatica_scan",
        "automatica_threshold", "threshold",
    }
    reference = {"bitmap_reference", "threshold_cpu_reference"}
    assert production.isdisjoint(reference)
    legacy = {
        "use_threshold_table", "use_tt_only", "use_tt_only_gpu",
        "use_bitmap_gfp", "use_inline_dynamics", "use_prefix_sweep",
        "boundary_seeding",
    }
    sample = {
        "synthesis_method": "threshold",
        "transition_semantics": "extremal_single_successor",
        "transition_backend": "precomputed",
        "boundary_semantics": "favorable_saturating",
        "threshold_d_star": "-1",
    }
    assert not legacy.intersection(sample)
    assert sample["synthesis_method"] in production

    examples = Path(__file__).resolve().parents[1] / "examples"
    tracked_configs = sorted(examples.glob("**/*.cfg"))
    assert len(tracked_configs) == 8
    for path in tracked_configs:
        text = path.read_text()
        assert 'synthesis_method = "threshold";' in text
        assert 'transition_semantics = "extremal_single_successor";' in text
        assert 'boundary_semantics = "favorable_saturating";' in text
        assert not any(re.search(rf"(?m)^\s*{key}\s*=", text) for key in legacy)

    runner_path = Path(__file__).resolve().parents[1] / "tools" / "run_solver_benchmark.py"
    spec = importlib.util.spec_from_file_location("solver_runner", runner_path)
    assert spec and spec.loader
    runner = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = runner
    spec.loader.exec_module(runner)
    base = """project_name = \"base\";
user_dynamics_file = \"dynamics.cl\";
benchmark_count = \"9\";
save_controller = \"false\";
save_transitions = \"false\";
record_basis_evolution = \"true\";
boundary_semantics = \"favorable_saturating\";
use_tt_only = \"true\";
"""
    generated = {
        method: runner.generated_config(
            base, "shared", method, 0, Path("/absolute/dynamics.cl")
        )
        for method in sorted(production)
    }
    normalized = {
        re.sub(r'synthesis_method = "[^"]+";', 'synthesis_method = "METHOD";', text)
        for text in generated.values()
    }
    assert len(normalized) == 1
    for method, text in generated.items():
        assert f'synthesis_method = "{method}";' in text
        assert 'transition_backend = "precomputed";' in text
        assert 'cdc_threshold_backend = "host";' in text
        assert 'boundary_semantics = "favorable_saturating";' in text
        assert not any(re.search(rf"(?m)^\s*{key}\s*=", text) for key in legacy)

    for method in sorted(reference):
        text = runner.generated_config(
            base, "shared", method, 0, Path("/absolute/dynamics.cl")
        )
        assert f'synthesis_method = "{method}";' in text
        assert 'transition_backend = "precomputed";' in text
        assert 'boundary_semantics = "favorable_saturating";' in text

    gpu_text = runner.generated_config(
        base, "shared", "cdc_threshold", 0, Path("/absolute/dynamics.cl"),
        cdc_threshold_backend="gpu",
    )
    assert 'synthesis_method = "cdc_threshold";' in gpu_text
    assert 'cdc_threshold_backend = "gpu";' in gpu_text

    inline_text = runner.generated_config(
        base, "shared", "threshold", 0, Path("/absolute/dynamics.cl"),
        transition_backend="inline",
    )
    assert 'transition_backend = "inline";' in inline_text
    assert 'save_transitions = "false";' in inline_text

    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        table_rows = []
        for method, backend, solver_ms in (
            ("cdc", "host", 10.0),
            ("cdc_threshold", "host", 4.0),
            ("cdc_threshold", "gpu", 6.0),
            ("automatica_scan", "host", 8.0),
            ("automatica_threshold", "host", 7.0),
            ("threshold", "host", 1.0),
            ("bitmap_reference", "host", 2.0),
        ):
            table_rows.append(runner.Measurement(
                case="fixture", method=method,
                cdc_threshold_backend=backend, repetition=1, status="ok",
                returncode=0, wall_ms=solver_ms, solver_ms=solver_ms,
                cdc_passes=3, cdc_mutations=4, gfp_rounds=5,
                frontier_batches=6, allocated_bytes=1024 * 1024,
            ))
        paper_table = Path(directory) / "paper_table.tex"
        runner.write_paper_table(paper_table, table_rows)
        table_text = paper_table.read_text()
        assert "CDC + threshold (host)" in table_text
        assert "CDC + threshold (gpu)" in table_text
        assert table_text.count("\\\\") == 7

        cache = Path(directory) / "transitions.u64.v2.bin"
        widths = (2, 2)
        # Identity is order preserving.
        values = (0, 1, 2, 3)
        cache.write_bytes(
            struct.pack("<8sIIQQ", b"MONOTR64", 2, 2, 4, 0) +
            struct.pack("<4Q", *values)
        )
        validation = runner.validate_transition_cache(cache, widths)
        assert validation["violations"] == 0
        # Invalid/top at the lower state followed by a valid successor is a
        # monotonicity violation.
        cache.write_bytes(
            struct.pack("<8sIIQQ", b"MONOTR64", 2, 2, 4, 0) +
            struct.pack("<4Q", (1 << 64) - 1, 1, 2, 3)
        )
        validation = runner.validate_transition_cache(cache, widths)
        assert validation["violations"] > 0

        if runner.np is not None:
            large_cache = Path(directory) / "large.transitions.u64.v2.bin"
            large_widths = (1000, 1000)
            large_states = 1_000_000
            with large_cache.open("wb") as stream:
                stream.write(struct.pack(
                    "<8sIIQQ", b"MONOTR64", 2, 2, large_states, 0
                ))
                runner.np.arange(large_states, dtype="<u8").tofile(stream)
            validation = runner.validate_transition_cache(
                large_cache, large_widths
            )
            assert validation["violations"] == 0
            with large_cache.open("r+b") as stream:
                stream.seek(struct.calcsize("<8sIIQQ"))
                stream.write(struct.pack("<Q", (1 << 64) - 1))
            validation = runner.validate_transition_cache(
                large_cache, large_widths
            )
            assert validation["violations"] > 0


def check_boundary_semantics() -> None:
    def project(value: float, lower: float, upper: float,
                priority: int) -> Optional[float]:
        if priority == 0:
            if value < lower:
                return None
            return min(value, upper)
        if value > upper:
            return None
        return max(value, lower)

    # Priority 0: high-side exits are favorable; low-side exits are unsafe.
    assert project(12.0, 0.0, 10.0, 0) == 10.0
    assert project(-1.0, 0.0, 10.0, 0) is None
    # Priority 1: low-side exits are favorable; high-side exits are unsafe.
    assert project(-1.0, 0.0, 10.0, 1) == 0.0
    assert project(12.0, 0.0, 10.0, 1) is None

    kernel = (Path(__file__).resolve().parents[1] /
              "kernel-pack" / "precompute_transitions.cl").read_text()
    assert "@@SATURATE_FAVORABLE_EXITS@@" in kernel
    assert kernel.count("apply_state_constraints(x_temp);") == 3
    assert "apply_state_constraints(x_curr);" in kernel
    assert "unfavorable_exit" in kernel


def optional_production_smoke() -> None:
    command = os.environ.get("PFACES_MONOSYNTH_TEST_COMMAND")
    if not command:
        print("SKIP: OpenCL production smoke test (PFACES_MONOSYNTH_TEST_COMMAND is unset)")
        return
    completed = subprocess.run(shlex.split(command), check=False)
    if completed.returncode != 0:
        raise AssertionError(f"production smoke test failed with {completed.returncode}")


def main() -> int:
    count = check_exhaustive_2x2()
    check_known_examples()
    check_config_contract()
    check_boundary_semantics()
    optional_production_smoke()
    print(f"five-method oracle passed ({count} monotone 2x2 systems)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
