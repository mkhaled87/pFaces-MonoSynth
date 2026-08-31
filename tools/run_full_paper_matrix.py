#!/usr/bin/env python3
"""Run the original paper scale matrix with all seven validated implementations.

Edit the four configuration blocks immediately below to change experiments,
methods, repetition policy, or resource limits.  Base example files are never
modified: generated configurations, logs, hashes, and tables live under the
selected output directory.

The runner distinguishes two regimes explicitly:

* At scales admitted by ``max_precomputed_states``, every method uses one shared
  validated 64-bit successor cache.
* Beyond that cap, only methods that implement inline dynamics (threshold GFP
  and bitmap GFP) may run.  Other rows are retained as resource skips rather
  than being silently omitted.  Bitmap GFP also has its own full-grid cap.

Successful methods on the same grid must produce identical complete canonical
threshold hashes, byte counts, and safe-cell counts.  CDC traces, Automatica
frontier counts, and synchronous GFP rounds receive additional equality checks.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import statistics
import struct
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import run_solver_benchmark as core


# ---------------------------------------------------------------------------
# USER-EDITABLE PAPER EXPERIMENT CONFIGURATION
# ---------------------------------------------------------------------------

PAPER_METHODS = (
    {"key": "cdc", "label": "CDC", "solver": "cdc", "cdc_backend": "host", "transition": "precomputed"},
    {"key": "cdc_threshold_host", "label": "CDC + threshold (host)", "solver": "cdc_threshold", "cdc_backend": "host", "transition": "precomputed"},
    {"key": "cdc_threshold_gpu", "label": "CDC + threshold (GPU)", "solver": "cdc_threshold", "cdc_backend": "gpu", "transition": "precomputed"},
    {"key": "automatica_scan", "label": "Automatica scan", "solver": "automatica_scan", "cdc_backend": "host", "transition": "precomputed"},
    {"key": "automatica_threshold", "label": "Automatica + threshold", "solver": "automatica_threshold", "cdc_backend": "host", "transition": "precomputed"},
    {"key": "threshold", "label": "Threshold GFP", "solver": "threshold", "cdc_backend": "host", "transition": "auto"},
    {"key": "bitmap_reference", "label": "Bitmap GFP", "solver": "bitmap_reference", "cdc_backend": "host", "transition": "auto"},
)

PAPER_EXPERIMENTS = (
    {
        "key": "acc", "label": "ACC", "config": "examples/acc/acc.cfg",
        "scales": (8, 9, 10, 12, 14),
        # These are the established near-isotropic ACC grids used to avoid the
        # empty-set discretization resonance on equal-width refinements.
        "preferred_widths": {
            9: (1033, 984, 984), 10: (2432, 2027, 2027),
            12: (11293, 9410, 9410), 14: (62501, 40001, 40001),
        },
    },
    {
        "key": "acc_5d", "label": "ACC-5D", "config": "examples/acc_5d/acc_5d.cfg",
        "scales": (8, 9, 11), "preferred_widths": {},
    },
    {
        "key": "turn_ego", "label": "Turn-Ego", "config": "examples/turn_ego_first/turn_ego_first.cfg",
        "scales": (8, 9, 10, 12, 14), "preferred_widths": {},
    },
    {
        "key": "turn_oncoming", "label": "Turn-Onc", "config": "examples/turn_oncoming_first/turn_oncoming_first.cfg",
        "scales": (8, 9, 10, 12, 14), "preferred_widths": {},
    },
)

RUN_POLICY = {
    "device": "1",
    "pfaces": "pfaces",
    "warmup_runs": 1,
    "measured_runs": 1,
    "verbose": 1,
    "resume": True,
    "retry_non_ok_on_resume": False,
    "timeouts_seconds": {
        "cdc": 3600,
        "cdc_threshold_host": 3600,
        "cdc_threshold_gpu": 3600,
        "automatica_scan": 3600,
        "automatica_threshold": 3600,
        "threshold": 1800,
        "bitmap_reference": 1800,
    },
}

RESOURCE_POLICY = {
    # One 64-bit successor per full-grid state. Increase this only when the GPU
    # can hold the cache and every solver buffer required by the selected case.
    "max_precomputed_states": 1_200_000_000,
    # Bitmap GFP stores the full state set even when dynamics are inline.
    "max_bitmap_inline_states": 12_000_000_000,
    # The canonical threshold uses two uint32 buffers during GFP iteration.
    "max_threshold_entries": 2_500_000_000,
    "max_basis_elements": 5_000_000,
    # Full transition monotonicity auditing is exact but O(d|X|). Larger caches
    # receive exact header/geometry/file-size checks and are labeled accordingly.
    "max_exhaustive_transition_audit_states": 10_000_000,
    "keep_transition_caches": False,
    "keep_canonical_outputs": False,
}

DEFAULT_OUTPUT = core.ROOT / "tools" / "benchmark_results" / "full_paper_seven_methods"


# ---------------------------------------------------------------------------
# Grid and configuration helpers
# ---------------------------------------------------------------------------

def product(values: Iterable[int]) -> int:
    result = 1
    for value in values:
        result *= int(value)
    return result


def parse_geometry(config: str) -> Tuple[List[float], List[float], int]:
    block = re.search(r"states\s*\{(.*?)\}", config, re.DOTALL)
    if not block:
        raise ValueError("states block is missing")

    def vector(name: str) -> List[float]:
        match = re.search(rf'{name}\s*=\s*"([^"]+)"', block.group(1))
        if not match:
            raise ValueError(f"states.{name} is missing")
        return [float(item.strip()) for item in match.group(1).split(",")]

    dimension = re.search(r'dim\s*=\s*"(\d+)"', block.group(1))
    if not dimension:
        raise ValueError("states.dim is missing")
    return vector("lb"), vector("ub"), int(dimension.group(1))


def balanced_widths(target_cells: int, dimension: int) -> Tuple[int, ...]:
    width = max(2, int(round(target_cells ** (1.0 / dimension))))
    return tuple(width for _ in range(dimension))


def experiment_widths(experiment: Dict[str, object], scale: int,
                      dimension: int) -> Tuple[int, ...]:
    preferred = experiment.get("preferred_widths", {})
    assert isinstance(preferred, dict)
    widths = preferred.get(scale)
    return tuple(widths) if widths is not None else balanced_widths(10 ** scale, dimension)


def eta_for_widths(lower: Sequence[float], upper: Sequence[float],
                   widths: Sequence[int]) -> str:
    return ",".join(
        f"{(hi - lo) / (width - 1):.18g}"
        for lo, hi, width in zip(lower, upper, widths)
    )


def method_by_key(key: str) -> Dict[str, str]:
    for method in PAPER_METHODS:
        if method["key"] == key:
            return method
    raise KeyError(key)


def transition_backend(method: Dict[str, str], states: int) -> Tuple[str, str]:
    policy = method["transition"]
    precomputed_ok = states <= int(RESOURCE_POLICY["max_precomputed_states"])
    if policy == "precomputed":
        if not precomputed_ok:
            return "skip", "precomputed successor table exceeds configured state cap"
        return "precomputed", ""
    if method["key"] == "bitmap_reference" and states > int(
            RESOURCE_POLICY["max_bitmap_inline_states"]):
        return "skip", "full-grid bitmap exceeds configured state cap"
    return ("precomputed", "") if precomputed_ok else ("inline", "")


def plan_method(method: Dict[str, str], widths: Sequence[int], d_star: int) -> Dict[str, object]:
    states = product(widths)
    entries = product(width for index, width in enumerate(widths) if index != d_star)
    backend, reason = transition_backend(method, states)
    if entries > int(RESOURCE_POLICY["max_threshold_entries"]):
        backend = "skip"
        reason = "canonical threshold table exceeds configured entry cap"
    return {
        "run_method": method["key"],
        "method": method["solver"],
        "label": method["label"],
        "cdc_threshold_backend": method["cdc_backend"],
        "transition_backend": backend,
        "status": "planned" if backend != "skip" else "skipped_resource",
        "reason": reason,
        "threshold_entries": entries,
        "estimated_transition_bytes": 8 * states if backend == "precomputed" else 0,
        "estimated_threshold_bytes": 8 * entries,
    }


def config_dynamics_path(base_path: Path, config: str) -> Path:
    match = re.search(r'user_dynamics_file\s*=\s*"([^"]+)"', config)
    if not match:
        raise ValueError(f"user_dynamics_file is missing in {base_path}")
    path = Path(match.group(1))
    return path if path.is_absolute() else (base_path.parent / path).resolve()


def validate_cache_geometry(path: Path, widths: Sequence[int]) -> Dict[str, object]:
    header_size = struct.calcsize("<8sIIQQ")
    with path.open("rb") as stream:
        header = stream.read(header_size)
    if len(header) != header_size:
        raise RuntimeError("transition cache header is incomplete")
    magic, version, dimensions, states, _ = struct.unpack("<8sIIQQ", header)
    expected_states = product(widths)
    expected_bytes = header_size + 8 * expected_states
    if magic != b"MONOTR64" or version != 2:
        raise RuntimeError("transition cache format is invalid")
    if dimensions != len(widths) or states != expected_states:
        raise RuntimeError("transition cache geometry is inconsistent")
    if path.stat().st_size != expected_bytes:
        raise RuntimeError("transition cache size is inconsistent")
    return {
        "states": expected_states,
        "bytes": expected_bytes,
        "geometry": "pass",
    }


# ---------------------------------------------------------------------------
# Checkpointing, validation, and summaries
# ---------------------------------------------------------------------------

def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    fields: List[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def read_csv(path: Path) -> List[Dict[str, object]]:
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return [dict(row) for row in csv.DictReader(stream)]


def as_int(row: Dict[str, object], field: str) -> int:
    value = row.get(field, 0)
    return int(value) if value not in (None, "") else 0


def as_float(row: Dict[str, object], field: str) -> float:
    value = row.get(field, 0.0)
    return float(value) if value not in (None, "") else 0.0


def raw_key(row: Dict[str, object]) -> Tuple[str, str, int]:
    return str(row["case"]), str(row["run_method"]), as_int(row, "repetition")


def validate_successful_rows(rows: Sequence[Dict[str, object]]) -> Dict[str, object]:
    measured = [row for row in rows if row.get("status") == "ok" and as_int(row, "repetition") > 0]
    if not measured:
        return {"status": "no_successful_method"}

    by_method: Dict[str, List[Dict[str, object]]] = {}
    for row in measured:
        by_method.setdefault(str(row["run_method"]), []).append(row)
    for method, method_rows in by_method.items():
        signatures = {
            (row["output_sha256"], as_int(row, "output_bytes"), as_int(row, "safe_cells"),
             as_int(row, "cdc_passes"), as_int(row, "cdc_mutations"),
             as_int(row, "gfp_rounds"), as_int(row, "frontier_batches"))
            for row in method_rows
        }
        if len(signatures) != 1:
            raise RuntimeError(f"{method} outputs or counters vary across repetitions")

    outputs = {
        (row["output_sha256"], as_int(row, "output_bytes"), as_int(row, "safe_cells"))
        for row in measured
    }
    if len(outputs) != 1:
        raise RuntimeError(f"cross-method canonical output mismatch: {outputs}")

    cdc_keys = [key for key in ("cdc", "cdc_threshold_host", "cdc_threshold_gpu") if key in by_method]
    if len(cdc_keys) > 1:
        traces = {
            (as_int(by_method[key][0], "cdc_passes"),
             as_int(by_method[key][0], "cdc_mutations"),
             by_method[key][0].get("cdc_pass_hashes", ""))
            for key in cdc_keys
        }
        if len(traces) != 1:
            raise RuntimeError(f"CDC pass-trace mismatch: {traces}")

    auto_keys = [key for key in ("automatica_scan", "automatica_threshold") if key in by_method]
    if len(auto_keys) > 1:
        counters = {
            (as_int(by_method[key][0], "gfp_rounds"),
             as_int(by_method[key][0], "frontier_batches"))
            for key in auto_keys
        }
        if len(counters) != 1:
            raise RuntimeError(f"Automatica counter mismatch: {counters}")

    synchronous = [
        key for key in ("automatica_scan", "automatica_threshold", "threshold", "bitmap_reference")
        if key in by_method
    ]
    rounds = {as_int(by_method[key][0], "gfp_rounds") for key in synchronous}
    if len(rounds) > 1:
        raise RuntimeError(f"synchronous GFP round mismatch: {rounds}")

    output = next(iter(outputs))
    return {
        "status": "exact_match" if len(by_method) > 1 else "single_method_only",
        "successful_methods": sorted(by_method),
        "canonical_sha256": output[0],
        "output_bytes": output[1],
        "safe_cells": output[2],
    }


def step_text(row: Dict[str, object]) -> str:
    method = str(row["method"])
    if method in ("cdc", "cdc_threshold"):
        return f"{as_int(row, 'cdc_passes')}/{as_int(row, 'cdc_mutations')}"
    if method.startswith("automatica"):
        return f"{as_int(row, 'gfp_rounds')}/{as_int(row, 'frontier_batches')}"
    return str(as_int(row, "gfp_rounds"))


def summarize(plans: Sequence[Dict[str, object]], raw_rows: Sequence[Dict[str, object]],
              case_metadata: Dict[str, Dict[str, object]], measured_runs: int) -> List[Dict[str, object]]:
    summaries: List[Dict[str, object]] = []
    for plan in plans:
        matching = [row for row in raw_rows if row.get("case") == plan["case"] and row.get("run_method") == plan["run_method"]]
        measured = [row for row in matching if row.get("status") == "ok" and as_int(row, "repetition") > 0]
        statuses = {str(row.get("status", "")) for row in matching}
        status = str(plan["status"])
        if status != "skipped_resource":
            if len(measured) == measured_runs:
                status = "ok"
            elif measured:
                status = "partial"
            elif "timeout" in statuses:
                status = "timeout"
            elif matching:
                status = "failed"
        row: Dict[str, object] = dict(plan)
        row["status"] = status
        row["successful_measured_runs"] = len(measured)
        metadata = case_metadata.get(str(plan["case"]), {})
        if plan["transition_backend"] == "precomputed":
            row["shared_transition_ms"] = metadata.get("transition_ms", "")
        elif plan["transition_backend"] == "inline":
            row["shared_transition_ms"] = "inline"
        else:
            row["shared_transition_ms"] = ""
        row["equality_status"] = metadata.get("equality_status", "not_run")
        if measured:
            first = measured[0]
            row["steps"] = step_text(first)
            row["safe_cells"] = as_int(first, "safe_cells")
            row["canonical_sha256"] = first.get("output_sha256", "")
            row["output_bytes"] = as_int(first, "output_bytes")
            row["allocated_bytes"] = as_int(first, "allocated_bytes")
            for field in (
                "membership_ms", "representation_ms", "threshold_maintenance_ms",
                "basis_update_ms", "solver_ms", "wall_ms",
            ):
                row[field] = statistics.median(as_float(item, field) for item in measured)
        summaries.append(row)
    return summaries


def fmt_ms(value: object) -> str:
    if value in (None, ""):
        return "--"
    if value == "inline":
        return "inline"
    return f"{float(value):.3f}"


def write_markdown(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    lines = [
        "# Seven-Method Full Paper Matrix",
        "",
        "Times are medians in milliseconds. `Shared trans.` is common successor precomputation and is excluded from solver total. CDC steps are passes/mutations; Automatica steps are rounds/frontier batches.",
        "",
        "| Example | Target | Actual cells | Grid | Method | Backend | Status | Equality | Steps | Shared trans. | Membership | Representation | Threshold maint. | Basis/frontier | Solver total | Allocated MiB | Safe cells |",
        "|---|---:|---:|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        allocated = "--" if not row.get("allocated_bytes") else f"{as_int(row, 'allocated_bytes') / (1024 * 1024):.2f}"
        lines.append(
            f"| {row['example']} | 1e{row['scale']} | {row['actual_cells']} | {row['widths']} | "
            f"{row['label']} | {row['transition_backend']} | {row['status']} | {row.get('equality_status', '--')} | "
            f"{row.get('steps', '--')} | {fmt_ms(row.get('shared_transition_ms'))} | "
            f"{fmt_ms(row.get('membership_ms'))} | {fmt_ms(row.get('representation_ms'))} | "
            f"{fmt_ms(row.get('threshold_maintenance_ms'))} | {fmt_ms(row.get('basis_update_ms'))} | "
            f"{fmt_ms(row.get('solver_ms'))} | {allocated} | {row.get('safe_cells', '--')} |"
        )
    path.write_text("\n".join(lines) + "\n")


def tex_escape(text: object) -> str:
    return str(text).replace("_", r"\_").replace("+", r"$+$")


def write_latex(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    lines = [
        "% Generated by tools/run_full_paper_matrix.py.",
        "% Requires: \\usepackage{booktabs,longtable,pdflscape}",
        r"\begin{landscape}",
        r"\scriptsize",
        r"\setlength{\tabcolsep}{2.3pt}",
        r"\begin{longtable}{@{}llrlcllrrrrrrrr@{}}",
        r"\caption{Seven-implementation paper-scale matrix. Times are medians in ms; shared transition construction is excluded from solver total.}\label{tab:full-seven-method-matrix}\\",
        r"\toprule",
        r"Example & Scale & $|X|$ & Grid & Method & Trans. & Status & Steps & Shared & Memb. & Repr. & Maint. & Basis & Total & MiB \\",
        r"\midrule\endfirsthead",
        r"\toprule",
        r"Example & Scale & $|X|$ & Grid & Method & Trans. & Status & Steps & Shared & Memb. & Repr. & Maint. & Basis & Total & MiB \\",
        r"\midrule\endhead",
    ]
    for row in rows:
        allocated = "--" if not row.get("allocated_bytes") else f"{as_int(row, 'allocated_bytes') / (1024 * 1024):.2f}"
        lines.append(
            f"{tex_escape(row['example'])} & $10^{{{row['scale']}}}$ & {row['actual_cells']} & "
            f"{tex_escape(row['widths'])} & {tex_escape(row['label'])} & "
            f"{tex_escape(row['transition_backend'])} & {tex_escape(row['status'])} & "
            f"{tex_escape(row.get('steps', '--'))} & {fmt_ms(row.get('shared_transition_ms'))} & "
            f"{fmt_ms(row.get('membership_ms'))} & {fmt_ms(row.get('representation_ms'))} & "
            f"{fmt_ms(row.get('threshold_maintenance_ms'))} & {fmt_ms(row.get('basis_update_ms'))} & "
            f"{fmt_ms(row.get('solver_ms'))} & {allocated} \\\\"
        )
    lines.extend([r"\bottomrule", r"\end{longtable}", r"\end{landscape}", ""])
    path.write_text("\n".join(lines))


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def parse_csv_filter(value: Optional[str]) -> Optional[set]:
    return None if not value else {item.strip() for item in value.split(",") if item.strip()}


def parse_scale_filter(value: Optional[str]) -> Optional[set]:
    if not value:
        return None
    return {
        int(item.strip().lower().removeprefix("1e"))
        for item in value.split(",") if item.strip()
    }


def build_manifest(experiments: Sequence[Dict[str, object]], methods: Sequence[Dict[str, str]],
                   args: argparse.Namespace) -> Dict[str, object]:
    inputs = {}
    for experiment in experiments:
        path = core.repository_path(str(experiment["config"])).resolve()
        inputs[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=core.ROOT, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True,
        ).stdout.strip()
    except subprocess.CalledProcessError:
        commit = "unknown"
    manifest = {
        "version": 1,
        "code_commit": commit,
        "runner_sha256": {
            str(Path(__file__).resolve()): hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            str(Path(core.__file__).resolve()): hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest(),
        },
        "base_config_sha256": inputs,
        "experiments": experiments,
        "methods": methods,
        "run_policy": RUN_POLICY | {
            "device": args.device, "pfaces": args.pfaces,
            "warmup_runs": args.warmups, "measured_runs": args.repetitions,
        },
        "resource_policy": RESOURCE_POLICY,
        "filters": {"examples": args.examples, "scales": args.scales, "methods": args.methods},
    }
    encoded = json.dumps(manifest, sort_keys=True, default=list).encode()
    manifest["fingerprint"] = hashlib.sha256(encoded).hexdigest()
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--pfaces", default=str(RUN_POLICY["pfaces"]))
    parser.add_argument("--device", default=str(RUN_POLICY["device"]))
    parser.add_argument("--warmups", type=int, default=int(RUN_POLICY["warmup_runs"]))
    parser.add_argument("--repetitions", type=int, default=int(RUN_POLICY["measured_runs"]))
    parser.add_argument("--timeout", type=int, help="Override every per-method timeout")
    parser.add_argument("--examples", help="Comma-separated keys: acc,acc_5d,turn_ego,turn_oncoming")
    parser.add_argument("--scales", help="Comma-separated exponents such as 8,9,10 or 1e8,1e9")
    parser.add_argument("--methods", help="Comma-separated method keys from PAPER_METHODS")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-resume", action="store_true")
    parser.add_argument("--keep-outputs", action="store_true")
    args = parser.parse_args()
    if args.warmups < 0 or args.repetitions <= 0:
        parser.error("warmups must be nonnegative and repetitions must be positive")

    example_filter = parse_csv_filter(args.examples)
    scale_filter = parse_scale_filter(args.scales)
    method_filter = parse_csv_filter(args.methods)
    experiments = [
        experiment for experiment in PAPER_EXPERIMENTS
        if example_filter is None or experiment["key"] in example_filter
    ]
    methods = [
        method for method in PAPER_METHODS
        if method_filter is None or method["key"] in method_filter
    ]
    unknown_examples = (example_filter or set()) - {str(item["key"]) for item in PAPER_EXPERIMENTS}
    unknown_methods = (method_filter or set()) - {str(item["key"]) for item in PAPER_METHODS}
    if unknown_examples or unknown_methods or not experiments or not methods:
        parser.error(f"invalid selection; unknown examples={sorted(unknown_examples)}, methods={sorted(unknown_methods)}")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = output / "manifest.json"
    manifest = build_manifest(experiments, methods, args)
    resume = bool(RUN_POLICY["resume"]) and not args.no_resume
    if manifest_path.exists():
        existing = json.loads(manifest_path.read_text())
        if existing.get("fingerprint") != manifest["fingerprint"]:
            raise RuntimeError(
                f"{output} contains a different experiment manifest; select a new --output directory"
            )
        if not resume:
            raise RuntimeError(f"{output} already exists; use resume or select a new --output directory")
    else:
        manifest_path.write_text(json.dumps(manifest, indent=2, default=list) + "\n")

    raw_path = output / "all_runs.csv"
    plan_path = output / "run_plan.csv"
    metadata_path = output / "case_metadata.json"
    raw_rows = read_csv(raw_path) if resume else []
    case_metadata: Dict[str, Dict[str, object]] = (
        json.loads(metadata_path.read_text()) if resume and metadata_path.exists() else {}
    )
    plans: List[Dict[str, object]] = []

    for experiment in experiments:
        base_path = core.repository_path(str(experiment["config"])).resolve()
        base_original = base_path.read_text()
        lower, upper, dimension = parse_geometry(base_original)
        dynamics = config_dynamics_path(base_path, base_original)
        scales = [scale for scale in experiment["scales"] if scale_filter is None or scale in scale_filter]
        for scale in scales:
            widths = experiment_widths(experiment, scale, dimension)
            eta = eta_for_widths(lower, upper, widths)
            base = core.override_state_eta(base_original, eta)
            parsed_widths = tuple(core.parse_widths(base))
            if parsed_widths != widths:
                raise RuntimeError(f"eta round-trip changed grid {widths} to {parsed_widths}")
            states = product(widths)
            d_star = core.deterministic_axis(widths)
            case = f"{experiment['key']}_1e{scale}"
            project = f"paper_matrix_{case}"
            case_dir = output / case
            case_dir.mkdir(parents=True, exist_ok=True)

            generated: Dict[str, Path] = {}
            case_plans: List[Dict[str, object]] = []
            for method in methods:
                plan = plan_method(method, widths, d_star)
                plan.update({
                    "case": case,
                    "example": experiment["label"],
                    "scale": scale,
                    "target_cells": 10 ** scale,
                    "actual_cells": states,
                    "widths": "x".join(str(width) for width in widths),
                    "d_star": d_star,
                    "base_config": str(base_path),
                })
                plans.append(plan)
                case_plans.append(plan)
                if plan["status"] == "skipped_resource":
                    continue
                method_spec = method_by_key(str(plan["run_method"]))
                config = core.generated_config(
                    base, project, str(plan["method"]), d_star, dynamics,
                    {"max_basis_elements": str(RESOURCE_POLICY["max_basis_elements"])},
                    str(method_spec["cdc_backend"]), str(plan["transition_backend"]),
                )
                config_path = case_dir / f"{case}.{plan['run_method']}.cfg"
                config_path.write_text(config)
                generated[str(plan["run_method"])] = config_path

            print(f"[case] {experiment['label']} 1e{scale}: widths={widths}, |X|={states}, d*={d_star}", flush=True)
            for plan in case_plans:
                print(
                    f"  [{plan['status']}] {plan['run_method']}: {plan['transition_backend']}"
                    + (f" ({plan['reason']})" if plan["reason"] else ""),
                    flush=True,
                )
            write_csv(plan_path, plans)
            if args.dry_run:
                continue

            run_ids = list(range(1 - args.warmups, 1)) + list(range(1, args.repetitions + 1))
            runnable = [str(plan["run_method"]) for plan in case_plans if plan["status"] == "planned"]
            existing_by_key = {raw_key(row): row for row in raw_rows}
            pending = []
            for sequence, repetition in enumerate(run_ids):
                order = runnable[sequence % len(runnable):] + runnable[:sequence % len(runnable)]
                for run_method in order:
                    key = (case, run_method, repetition)
                    existing = existing_by_key.get(key)
                    if existing is None or (
                        bool(RUN_POLICY["retry_non_ok_on_resume"]) and existing.get("status") != "ok"
                    ):
                        pending.append((run_method, repetition))

            cache_path = case_dir / f"{project}.transitions.u64.v2.bin"
            needs_cache = any(
                next(plan for plan in case_plans if plan["run_method"] == run_method)["transition_backend"] == "precomputed"
                for run_method, _ in pending
            )
            if needs_cache:
                geometry = None
                if cache_path.exists():
                    geometry = validate_cache_geometry(cache_path, widths)
                else:
                    cache_method = next(
                        str(plan["run_method"]) for plan in case_plans
                        if plan["status"] == "planned" and plan["transition_backend"] == "precomputed"
                        and plan["run_method"] == "threshold"
                    ) if any(
                        plan["status"] == "planned" and plan["transition_backend"] == "precomputed"
                        and plan["run_method"] == "threshold" for plan in case_plans
                    ) else next(
                        str(plan["run_method"]) for plan in case_plans
                        if plan["status"] == "planned" and plan["transition_backend"] == "precomputed"
                    )
                    prep = core.run_once(
                        case=case, run_method=cache_method, repetition=-999,
                        config=generated[cache_method], project=project,
                        output_dir=case_dir, executable=args.pfaces, device=args.device,
                        verbose=int(RUN_POLICY["verbose"]),
                        timeout=args.timeout or int(RUN_POLICY["timeouts_seconds"][cache_method]),
                    )
                    if prep.status != "ok":
                        raise RuntimeError(f"{case} transition-cache preparation failed; see {prep.log}")
                    geometry = validate_cache_geometry(cache_path, widths)
                    case_metadata.setdefault(case, {})["transition_ms"] = prep.transition_ms
                    case_metadata[case]["cache_prepare_sha256"] = prep.output_sha256
                    if not args.keep_outputs and not bool(RESOURCE_POLICY["keep_canonical_outputs"]):
                        Path(prep.output).unlink(missing_ok=True)
                assert geometry is not None
                if states <= int(RESOURCE_POLICY["max_exhaustive_transition_audit_states"]):
                    audit = core.validate_transition_cache(cache_path, widths)
                    if audit["violations"]:
                        raise RuntimeError(f"{case} transition monotonicity violations: {audit}")
                    geometry["monotonicity_audit"] = audit
                else:
                    geometry["monotonicity_audit"] = "not_exhaustive_above_configured_cap"
                case_metadata.setdefault(case, {})["transition_validation"] = geometry
                metadata_path.write_text(json.dumps(case_metadata, indent=2) + "\n")

            for run_method, repetition in pending:
                plan = next(plan for plan in case_plans if plan["run_method"] == run_method)
                timeout = args.timeout or int(RUN_POLICY["timeouts_seconds"][run_method])
                measurement = core.run_once(
                    case=case, run_method=run_method, repetition=repetition,
                    config=generated[run_method], project=project,
                    output_dir=case_dir, executable=args.pfaces, device=args.device,
                    verbose=int(RUN_POLICY["verbose"]), timeout=timeout,
                )
                measurement.transition_bytes = (
                    8 * states if plan["transition_backend"] == "precomputed" else 0
                )
                row: Dict[str, object] = asdict(measurement)
                row.update({
                    "run_method": run_method,
                    "transition_backend": plan["transition_backend"],
                    "example": experiment["label"], "scale": scale,
                    "widths": plan["widths"], "actual_cells": states,
                })
                key = raw_key(row)
                raw_rows = [existing for existing in raw_rows if raw_key(existing) != key]
                raw_rows.append(row)
                write_csv(raw_path, raw_rows)
                print(
                    f"[{measurement.status}] {case} {run_method} run={repetition}: "
                    f"solver_ms={measurement.solver_ms:.3f} hash={measurement.output_sha256[:12]}",
                    flush=True,
                )
                if measurement.status == "ok" and not args.keep_outputs and not bool(
                        RESOURCE_POLICY["keep_canonical_outputs"]):
                    Path(measurement.output).unlink(missing_ok=True)

            case_rows = [row for row in raw_rows if row.get("case") == case]
            validation = validate_successful_rows(case_rows)
            case_metadata.setdefault(case, {}).update({
                "equality_status": validation["status"],
                "output_validation": validation,
            })
            metadata_path.write_text(json.dumps(case_metadata, indent=2) + "\n")
            if cache_path.exists() and not bool(RESOURCE_POLICY["keep_transition_caches"]):
                cache_path.unlink()

            summary = summarize(plans, raw_rows, case_metadata, args.repetitions)
            write_csv(output / "full_matrix_summary.csv", summary)
            write_markdown(output / "full_matrix_table.md", summary)
            write_latex(output / "full_matrix_table.tex", summary)

    summary = summarize(plans, raw_rows, case_metadata, args.repetitions)
    write_csv(output / "full_matrix_summary.csv", summary)
    write_markdown(output / "full_matrix_table.md", summary)
    write_latex(output / "full_matrix_table.tex", summary)
    print(f"[done] plan: {plan_path}")
    print(f"[done] raw runs: {raw_path}")
    print(f"[done] combined table: {output / 'full_matrix_table.md'}")
    print(f"[done] LaTeX table: {output / 'full_matrix_table.tex'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
