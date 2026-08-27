#!/usr/bin/env python3
"""Validated configurable solver benchmark for MonoSynth.

Every case uses one generated configuration, one 64-bit successor cache, one
deterministic threshold axis, and precomputed transitions.  A result row is
written only if every measured run produces the same canonical threshold bytes.
The default remains the fair four-production-method, one-warm-up/five-run
protocol.  ``--experiment-config`` may select a validated method subset and a
different proof-of-concept repetition count from one JSON file.
"""

from __future__ import annotations

import argparse
import array
import csv
import hashlib
import json
import re
import shlex
import shutil
import statistics
import struct
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

try:
    import numpy as np
except ImportError:  # Small-case standard-library fallback remains available.
    np = None

ROOT = Path(__file__).resolve().parents[1]
KERNEL_PACK = ROOT / "kernel-pack"
PRODUCTION_METHODS = (
    "cdc", "automatica_scan", "automatica_threshold", "threshold"
)
REFERENCE_METHODS = ("bitmap_reference", "threshold_cpu_reference")
ALL_METHODS = PRODUCTION_METHODS + REFERENCE_METHODS
CLI_METHODS = {
    "cdc": "cdc",
    "automatica-scan": "automatica_scan",
    "automatica_scan": "automatica_scan",
    "automatica-threshold": "automatica_threshold",
    "automatica_threshold": "automatica_threshold",
    "threshold": "threshold",
    "bitmap-reference": "bitmap_reference",
    "bitmap_reference": "bitmap_reference",
    "threshold-cpu-reference": "threshold_cpu_reference",
    "threshold_cpu_reference": "threshold_cpu_reference",
}


@dataclass
class Measurement:
    case: str
    method: str
    repetition: int
    status: str
    returncode: int
    wall_ms: float
    output_sha256: str = ""
    output_bytes: int = 0
    transition_bytes: int = 0
    allocated_bytes: int = 0
    safe_cells: int = 0
    cdc_epochs: int = 0
    cdc_mutations: int = 0
    gfp_rounds: int = 0
    frontier_batches: int = 0
    membership_queries: int = 0
    binary_search_probes: int = 0
    transition_ms: float = 0.0
    membership_ms: float = 0.0
    representation_ms: float = 0.0
    basis_update_ms: float = 0.0
    solver_ms: float = 0.0
    log: str = ""
    output: str = ""
    error: str = ""


def parse_methods(text: str) -> List[str]:
    requested: List[str] = []
    for item in text.split(","):
        key = item.strip().lower()
        if key not in CLI_METHODS:
            raise argparse.ArgumentTypeError(f"unknown method: {item}")
        method = CLI_METHODS[key]
        if method not in requested:
            requested.append(method)
    if not requested:
        raise argparse.ArgumentTypeError("at least one method is required")
    return requested


def repository_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def apply_experiment_config(args: argparse.Namespace,
                            parser: argparse.ArgumentParser) -> None:
    if args.experiment_config is None:
        return
    if args.configs:
        parser.error("do not combine positional configs with --experiment-config")
    path = args.experiment_config.resolve()
    try:
        spec = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as error:
        parser.error(f"cannot read experiment config {path}: {error}")
    if not isinstance(spec, dict):
        parser.error("experiment config must contain one JSON object")
    allowed = {
        "version", "name", "configs", "methods", "state_eta", "warmup_runs",
        "measured_runs", "device", "timeout_seconds", "verbose", "output",
        "pfaces", "allow_nonmonotone_diagnostic", "config_overrides",
    }
    unknown = sorted(set(spec) - allowed)
    if unknown:
        parser.error(f"unknown experiment-config keys: {', '.join(unknown)}")
    if spec.get("version", 1) != 1:
        parser.error("experiment config version must be 1")
    configs = spec.get("configs")
    if not isinstance(configs, list) or not configs or not all(
            isinstance(item, str) for item in configs):
        parser.error("experiment config 'configs' must be a nonempty string list")
    methods = spec.get("methods")
    if not isinstance(methods, list) or not methods or not all(
            isinstance(item, str) for item in methods):
        parser.error("experiment config 'methods' must be a nonempty string list")
    try:
        args.methods = parse_methods(",".join(methods))
    except argparse.ArgumentTypeError as error:
        parser.error(str(error))
    args.configs = [repository_path(item) for item in configs]
    if "state_eta" in spec:
        if not isinstance(spec["state_eta"], str):
            parser.error("experiment config 'state_eta' must be a string")
        args.state_eta = spec["state_eta"]
    args.warmups = spec.get("warmup_runs", args.warmups)
    args.repetitions = spec.get("measured_runs", args.repetitions)
    args.device = str(spec.get("device", args.device))
    args.timeout = spec.get("timeout_seconds", args.timeout)
    args.verbose = spec.get("verbose", args.verbose)
    args.pfaces = spec.get("pfaces", args.pfaces)
    args.allow_nonmonotone_diagnostic = spec.get(
        "allow_nonmonotone_diagnostic", args.allow_nonmonotone_diagnostic
    )
    if "output" in spec:
        if not isinstance(spec["output"], str):
            parser.error("experiment config 'output' must be a string")
        args.output = repository_path(spec["output"])
    overrides = spec.get("config_overrides", {})
    if not isinstance(overrides, dict) or not all(
            isinstance(key, str) and isinstance(value, (str, int, float, bool))
            for key, value in overrides.items()):
        parser.error("experiment config 'config_overrides' must be a scalar map")
    reserved = {
        "project_name", "synthesis_method", "transition_semantics",
        "transition_backend", "boundary_semantics", "threshold_d_star",
        "benchmark_count", "record_basis_evolution", "extract_basis",
        "save_controller", "save_transitions", "user_dynamics_file",
    }
    conflict = sorted(set(overrides) & reserved)
    if conflict:
        parser.error(
            "config_overrides cannot replace benchmark-controlled keys: " +
            ", ".join(conflict)
        )
    args.config_overrides = {
        key: str(value).lower() if isinstance(value, bool) else str(value)
        for key, value in overrides.items()
    }


def replace_assignment(text: str, key: str, value: str) -> str:
    pattern = re.compile(rf"(?m)^\s*{re.escape(key)}\s*=\s*\"[^\"]*\"\s*;.*$")
    replacement = f'{key} = "{value}";'
    if pattern.search(text):
        return pattern.sub(replacement, text, count=1)
    return text.rstrip() + "\n" + replacement + "\n"


def generated_config(base: str, project: str, method: str, d_star: int,
                     dynamics_file: Path,
                     common_overrides: Optional[Dict[str, str]] = None) -> str:
    text = base
    boundary_match = re.search(
        r'(?m)^\s*boundary_semantics\s*=\s*"([^"]+)"\s*;', base
    )
    boundary_semantics = boundary_match.group(1) if boundary_match else "strict_unsafe"
    values = {
        "project_name": project,
        "synthesis_method": method,
        "transition_semantics": "extremal_single_successor",
        "transition_backend": "precomputed",
        "boundary_semantics": boundary_semantics,
        "threshold_d_star": str(d_star),
        "benchmark_count": "1",
        "record_basis_evolution": "false",
        "extract_basis": "false",
        "save_controller": "true",
        "save_transitions": "true",
        "user_dynamics_file": str(dynamics_file),
    }
    legacy = (
        "use_threshold_table", "use_tt_only", "use_tt_only_gpu",
        "use_bitmap_gfp", "use_inline_dynamics", "use_prefix_sweep",
        "boundary_seeding",
    )
    for key in legacy:
        text = re.sub(
            rf"(?m)^\s*{re.escape(key)}\s*=\s*\"[^\"]*\"\s*;.*$\n?", "", text
        )
    for key, value in values.items():
        text = replace_assignment(text, key, value)
    for key, value in (common_overrides or {}).items():
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.]*", key):
            raise ValueError(f"invalid configuration override key: {key}")
        text = replace_assignment(text, key, value)
    return text


def override_state_eta(config: str, eta: str) -> str:
    values = [item.strip() for item in eta.split(",")]
    if not values or any(float(item) <= 0 for item in values):
        raise ValueError("--state-eta values must be positive")
    block = re.search(r"states\s*\{(.*?)\}", config, re.DOTALL)
    if not block:
        raise ValueError("states block is missing")
    dimension = re.search(r'dim\s*=\s*"(\d+)"', block.group(1))
    if not dimension or len(values) != int(dimension.group(1)):
        raise ValueError("--state-eta must provide one value per state dimension")
    replacement = ",".join(values)
    return re.sub(
        r'(states\s*\{.*?eta\s*=\s*)"[^"]*"',
        rf'\g<1>"{replacement}"', config, count=1, flags=re.DOTALL,
    )


def parse_widths(config: str) -> List[int]:
    block = re.search(r"states\s*\{(.*?)\}", config, re.DOTALL)
    if not block:
        raise ValueError("states block is missing")
    def values(key: str) -> List[float]:
        match = re.search(rf'{key}\s*=\s*"([^"]+)"', block.group(1))
        if not match:
            raise ValueError(f"states.{key} is missing")
        return [float(item.strip()) for item in match.group(1).split(",")]
    lower, upper, eta = values("lb"), values("ub"), values("eta")
    return [int(round((hi - lo) / step)) + 1
            for lo, hi, step in zip(lower, upper, eta)]


def deterministic_axis(widths: Sequence[int]) -> int:
    maximum = max(widths)
    return next(index for index, value in enumerate(widths) if value == maximum)


def product(values: Iterable[int]) -> int:
    result = 1
    for value in values:
        result *= value
    return result


def parse_stats(output: str) -> Dict[str, str]:
    lines = [line for line in output.splitlines() if "MONOSYNTH_STATS" in line]
    if not lines:
        raise ValueError("MONOSYNTH_STATS line is missing")
    return dict(re.findall(r"([a-z_]+)=([^\s]+)", lines[-1]))


def validate_transition_cache(path: Path, widths: Sequence[int]) -> Dict[str, int]:
    header_size = struct.calcsize("<8sIIQQ")
    with path.open("rb") as stream:
        header = stream.read(header_size)
    if len(header) < header_size:
        raise ValueError("transition cache is shorter than its header")
    magic, version, dimensions, states, _ = struct.unpack_from(
        "<8sIIQQ", header
    )
    if magic != b"MONOTR64" or version != 2:
        raise ValueError("transition cache has an unsupported format")
    if dimensions != len(widths) or states != product(widths):
        raise ValueError("transition cache geometry does not match the case")
    expected_bytes = header_size + 8 * states
    if path.stat().st_size != expected_bytes:
        raise ValueError("transition cache payload has the wrong size")
    if np is not None and states >= 1_000_000:
        successors = np.memmap(
            path, mode="r", dtype="<u8", offset=header_size, shape=(states,)
        )
        invalid = np.uint64((1 << 64) - 1)
        strides = [1]
        for width in widths[:-1]:
            strides.append(strides[-1] * width)
        adjacent_pairs = sum(states * (width - 1) // width for width in widths)
        violations = 0
        first_state = -1
        first_dimension = -1
        chunk_size = 1_000_000
        for dimension, (width, stride) in enumerate(zip(widths, strides)):
            for start in range(0, states, chunk_size):
                stop = min(states, start + chunk_size)
                indices = np.arange(start, stop, dtype=np.uint64)
                active = ((indices // np.uint64(stride)) % np.uint64(width)) + 1 < width
                indices = indices[active]
                if indices.size == 0:
                    continue
                positions = indices.astype(np.intp, copy=False)
                left = np.asarray(successors[positions], dtype=np.uint64)
                right = np.asarray(successors[positions + stride], dtype=np.uint64)
                right_finite = right != invalid
                left_finite = left != invalid
                bad = right_finite & ~left_finite
                comparable = right_finite & left_finite
                left_decode = left.copy()
                right_decode = right.copy()
                for successor_width in widths:
                    bad |= comparable & (
                        left_decode % successor_width > right_decode % successor_width
                    )
                    left_decode //= successor_width
                    right_decode //= successor_width
                count = int(np.count_nonzero(bad))
                violations += count
                if count:
                    candidate = int(indices[int(np.flatnonzero(bad)[0])])
                    if first_state < 0 or (candidate, dimension) < (
                            first_state, first_dimension):
                        first_state = candidate
                        first_dimension = dimension
        return {
            "states": states,
            "adjacent_pairs": adjacent_pairs,
            "violations": violations,
            "first_state": first_state,
            "first_dimension": first_dimension,
        }

    with path.open("rb") as stream:
        stream.seek(header_size)
        payload = stream.read()
    successors = array.array("Q")
    if successors.itemsize != 8:
        raise RuntimeError("Python unsigned-long-long is not 64 bits")
    successors.frombytes(payload)
    if sys.byteorder != "little":
        successors.byteswap()
    invalid = (1 << 64) - 1

    def successor_leq(left: int, right: int) -> bool:
        # The invalid successor is the artificial top/unsafe state.
        if left == invalid:
            return right == invalid
        if right == invalid:
            return True
        for width in widths:
            if left % width > right % width:
                return False
            left //= width
            right //= width
        return True

    strides = [1]
    for width in widths[:-1]:
        strides.append(strides[-1] * width)
    adjacent_pairs = 0
    violations = 0
    first_state = -1
    first_dimension = -1
    for state in range(states):
        for dimension, (width, stride) in enumerate(zip(widths, strides)):
            coordinate = (state // stride) % width
            if coordinate + 1 >= width:
                continue
            adjacent_pairs += 1
            upper = state + stride
            if not successor_leq(successors[state], successors[upper]):
                violations += 1
                if first_state < 0:
                    first_state = state
                    first_dimension = dimension
    return {
        "states": states,
        "adjacent_pairs": adjacent_pairs,
        "violations": violations,
        "first_state": first_state,
        "first_dimension": first_dimension,
    }


def pfaces_command(executable: str, config: Path, device: str, verbose: int) -> List[str]:
    return [
        executable, "-GH", "-k", f"mono_synth.gpu@{KERNEL_PACK}",
        "-cfg", str(config), "-d", device, "-p", f"-v{verbose}",
    ]


def run_once(*, case: str, method: str, repetition: int, config: Path,
             project: str, output_dir: Path, executable: str, device: str,
             verbose: int, timeout: int) -> Measurement:
    log_path = output_dir / f"{case}.{method}.run{repetition}.log"
    canonical_source = output_dir / f"{project}.threshold.u32.bin"
    canonical_target = output_dir / f"{case}.{method}.run{repetition}.threshold.u32.bin"
    canonical_source.unlink(missing_ok=True)
    command = pfaces_command(executable, config, device, verbose)
    shell_command = " ".join(shlex.quote(part) for part in command)
    start = time.perf_counter()
    try:
        completed = subprocess.run(
            ["bash", "-lc", shell_command], cwd=output_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, timeout=timeout, check=False,
        )
    except subprocess.TimeoutExpired as error:
        captured = error.stdout or ""
        if isinstance(captured, bytes):
            captured = captured.decode("utf-8", errors="replace")
        log_path.write_text(captured + "\nTIMEOUT\n")
        return Measurement(case, method, repetition, "timeout", -1,
                           (time.perf_counter() - start) * 1000,
                           log=str(log_path), error="timeout")
    wall_ms = (time.perf_counter() - start) * 1000
    log_path.write_text(completed.stdout)
    measurement = Measurement(case, method, repetition, "failed",
                              completed.returncode, wall_ms, log=str(log_path))
    if completed.returncode != 0:
        measurement.error = "pFaces returned a non-zero exit status"
        return measurement
    if not canonical_source.exists():
        measurement.error = "canonical threshold output is missing"
        return measurement
    shutil.move(str(canonical_source), canonical_target)
    payload = canonical_target.read_bytes()
    stats = parse_stats(completed.stdout)
    measurement.status = "ok"
    measurement.output = str(canonical_target)
    measurement.output_sha256 = hashlib.sha256(payload).hexdigest()
    measurement.output_bytes = len(payload)
    integer_fields = (
        "safe_cells", "cdc_epochs", "cdc_mutations", "gfp_rounds",
        "frontier_batches", "membership_queries", "binary_search_probes",
        "allocated_bytes",
    )
    float_fields = (
        "transition_ms", "membership_ms", "representation_ms",
        "basis_update_ms", "solver_ms",
    )
    for field in integer_fields:
        setattr(measurement, field, int(stats.get(field, "0")))
    for field in float_fields:
        setattr(measurement, field, float(stats.get(field, "0")))
    return measurement


def validate_case(measurements: Iterable[Measurement],
                  require_outer_equality: bool = True) -> None:
    rows = list(measurements)
    failures = [row for row in rows if row.status != "ok"]
    if failures:
        raise RuntimeError("at least one method/run failed: " +
                           ", ".join(f"{r.method}:{r.error}" for r in failures))
    hashes = {row.output_sha256 for row in rows}
    safe_counts = {row.safe_cells for row in rows}
    output_sizes = {row.output_bytes for row in rows}
    if len(hashes) != 1 or len(safe_counts) != 1 or len(output_sizes) != 1:
        raise RuntimeError(
            f"canonical output mismatch: hashes={hashes}, "
            f"safe_cells={safe_counts}, bytes={output_sizes}"
        )
    validated_methods = tuple(
        method for method in ALL_METHODS
        if any(row.method == method for row in rows)
    )
    by_method = {
        method: [row for row in rows if row.method == method]
        for method in validated_methods
    }
    for method, method_rows in by_method.items():
        counters = {
            (row.cdc_epochs, row.cdc_mutations, row.gfp_rounds,
             row.frontier_batches)
            for row in method_rows
        }
        if len(counters) != 1:
            raise RuntimeError(f"{method} counters vary across repetitions")
    outer_methods = tuple(
        method for method in (
            "automatica_scan", "automatica_threshold", "threshold",
            "bitmap_reference", "threshold_cpu_reference",
        ) if method in by_method
    )
    outer_rounds = {by_method[method][0].gfp_rounds for method in outer_methods}
    if require_outer_equality and len(outer_methods) > 1 and len(outer_rounds) != 1:
        raise RuntimeError(
            "methods 2--4 do not have identical outer GFP round counts: "
            f"{sorted(outer_rounds)}"
        )
    auto_methods = tuple(
        method for method in ("automatica_scan", "automatica_threshold")
        if method in by_method
    )
    auto_batches = {by_method[method][0].frontier_batches for method in auto_methods}
    if len(auto_methods) > 1 and len(auto_batches) != 1:
        raise RuntimeError(
            "the two Automatica membership backends do not have identical "
            "frontier-batch counts"
        )


def write_csv(path: Path, rows: Sequence[Measurement]) -> None:
    fields = list(asdict(rows[0]).keys()) if rows else list(Measurement.__dataclass_fields__)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(asdict(row) for row in rows)


def write_summary(path: Path, rows: Sequence[Measurement],
                  transition_validation: Dict[str, int], warmups: int,
                  measured_runs: int) -> None:
    summary = {}
    reported_methods = tuple(
        method for method in ALL_METHODS
        if any(row.method == method for row in rows)
    )
    for method in reported_methods:
        measured = [row for row in rows if row.method == method and row.repetition > 0]
        summary[method] = {
            "reference_only": method in REFERENCE_METHODS,
            "median_solver_ms": statistics.median(row.solver_ms for row in measured),
            "min_solver_ms": min(row.solver_ms for row in measured),
            "max_solver_ms": max(row.solver_ms for row in measured),
            "canonical_sha256": measured[0].output_sha256,
            "safe_cells": measured[0].safe_cells,
            "allocated_bytes": measured[0].allocated_bytes,
            "transition_bytes": measured[0].transition_bytes,
            "algorithm_counters": {
                "cdc_epochs": measured[0].cdc_epochs,
                "cdc_mutations": measured[0].cdc_mutations,
                "gfp_rounds": measured[0].gfp_rounds,
                "frontier_batches": measured[0].frontier_batches,
            },
        }
    threshold_time = summary.get("threshold", {}).get("median_solver_ms", 0.0)
    summary["speedup_scope"] = "evaluated complete precomputed implementations"
    summary["benchmark_protocol"] = {
        "methods": list(reported_methods),
        "warmup_runs": warmups,
        "measured_runs": measured_runs,
        "stochastic_seeds": "not applicable; all solvers are deterministic",
    }
    summary["transition_monotonicity"] = transition_validation
    outer_methods = tuple(
        method for method in (
            "automatica_scan", "automatica_threshold", "threshold",
            "bitmap_reference", "threshold_cpu_reference",
        ) if method in summary
    )
    outer_rounds = {
        summary[method]["algorithm_counters"]["gfp_rounds"]
        for method in outer_methods
    }
    summary["strict_acceptance"] = (
        transition_validation["violations"] == 0 and len(outer_rounds) == 1
    )
    summary["speedups_vs_threshold"] = {
        method: summary[method]["median_solver_ms"] / threshold_time
        for method in reported_methods if method != "threshold" and threshold_time > 0
    }
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("configs", nargs="*", type=Path,
                        help="Tracked base configurations to benchmark")
    parser.add_argument(
        "--experiment-config", type=Path,
        help="Load methods, grids, run counts, device, timeout, and output from JSON",
    )
    parser.add_argument(
        "--methods", default=list(PRODUCTION_METHODS), type=parse_methods,
        help="Comma-separated solver subset (default: all four production methods)",
    )
    parser.add_argument("--output", type=Path,
                        default=ROOT / "tools" / "benchmark_results" / "four_methods")
    parser.add_argument("--pfaces", default="pfaces")
    parser.add_argument("--device", default="1")
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--repetitions", type=int, default=5)
    parser.add_argument("--timeout", type=int, default=3600)
    parser.add_argument("--verbose", type=int, default=1)
    parser.add_argument(
        "--state-eta",
        help="Optional shared state-grid resolution override for small tests",
    )
    parser.add_argument(
        "--allow-nonmonotone-diagnostic", action="store_true",
        help=("Continue only as a clearly marked diagnostic when the shared "
              "successor table violates the monotone abstraction premise"),
    )
    parser.add_argument(
        "--include-references", action="store_true",
        help=("also run bitmap_reference and threshold_cpu_reference under "
              "the same warm-up/five-run protocol and equality gate"),
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.set_defaults(config_overrides={})
    args = parser.parse_args()
    apply_experiment_config(args, parser)
    if not args.configs:
        parser.error("provide at least one config or use --experiment-config")
    if args.warmups < 0:
        parser.error("warmup count must be nonnegative")
    if args.repetitions <= 0:
        parser.error("measured-run count must be positive")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    if args.verbose < 0:
        parser.error("verbose level must be nonnegative")

    output_dir = args.output.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    all_rows: List[Measurement] = []
    for base_path in args.configs:
        base_path = base_path.resolve()
        base = base_path.read_text()
        if args.state_eta:
            try:
                base = override_state_eta(base, args.state_eta)
            except ValueError as error:
                parser.error(str(error))
        widths = parse_widths(base)
        d_star = deterministic_axis(widths)
        transition_bytes = 8 * product(widths)
        case = base_path.stem
        project = f"four_method_{case}"
        cache_path = output_dir / f"{project}.transitions.u64.v2.bin"
        dynamics_match = re.search(r'user_dynamics_file\s*=\s*"([^"]+)"', base)
        if not dynamics_match:
            parser.error(f"user_dynamics_file is missing in {base_path}")
        dynamics_file = Path(dynamics_match.group(1))
        if not dynamics_file.is_absolute():
            dynamics_file = (base_path.parent / dynamics_file).resolve()
        methods_to_run = list(args.methods)
        if args.include_references:
            methods_to_run.extend(
                method for method in REFERENCE_METHODS
                if method not in methods_to_run
            )
        generated_paths = {}
        for method in methods_to_run:
            text = generated_config(
                base, project, method, d_star, dynamics_file,
                args.config_overrides,
            )
            path = output_dir / f"{case}.{method}.cfg"
            path.write_text(text)
            generated_paths[method] = path
        if args.dry_run:
            print(f"{case}: widths={widths}, d*={d_star}")
            continue

        cache_path.unlink(missing_ok=True)
        case_rows: List[Measurement] = []
        transition_validation = None
        run_ids = list(range(1 - args.warmups, 1)) + list(
            range(1, args.repetitions + 1)
        )
        for method in methods_to_run:
            for repetition in run_ids:  # nonpositive = warm-up; positive = measured
                row = run_once(
                    case=case, method=method, repetition=repetition,
                    config=generated_paths[method], project=project,
                    output_dir=output_dir, executable=args.pfaces,
                    device=args.device, verbose=args.verbose, timeout=args.timeout,
                )
                row.transition_bytes = transition_bytes
                case_rows.append(row)
                print(f"{case} {method} run={repetition}: {row.status} "
                      f"solver_ms={row.solver_ms:.3f} hash={row.output_sha256[:12]}",
                      flush=True)
                if row.status != "ok":
                    raise RuntimeError(
                        f"{case} {method} run={repetition} failed; see {row.log}: "
                        f"{row.error}"
                    )
                if transition_validation is None:
                    transition_validation = validate_transition_cache(
                        cache_path, widths
                    )
                    if transition_validation["violations"]:
                        message = (
                            "successor table is not order preserving: "
                            f"{transition_validation['violations']} adjacent "
                            "violations; first at state "
                            f"{transition_validation['first_state']}, dimension "
                            f"{transition_validation['first_dimension']}"
                        )
                        if not args.allow_nonmonotone_diagnostic:
                            raise RuntimeError(message)
                        print(f"WARNING: {message}; diagnostic timings only",
                              flush=True)
        validate_case(
            case_rows,
            require_outer_equality=not args.allow_nonmonotone_diagnostic,
        )
        assert transition_validation is not None
        all_rows.extend(case_rows)
        write_csv(output_dir / f"{case}.measurements.csv", case_rows)
        write_summary(output_dir / f"{case}.summary.json", case_rows,
                      transition_validation, args.warmups, args.repetitions)

    if all_rows:
        write_csv(output_dir / "all_measurements.csv", all_rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
