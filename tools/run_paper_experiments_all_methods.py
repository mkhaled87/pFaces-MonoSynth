#!/usr/bin/env python3
"""Run the paper scaling experiments across lazy, threshold, and bitmap GFP.

The runner is intentionally conservative:

* one generated grid is shared by every method for a case;
* threshold-table mode is timed for every possible designated dimension d*;
* the fastest nonempty threshold result selects d* for the final row;
* zero-safe grids are rejected and nearby bin counts are tried deterministically;
* bitmap/lazy runs are skipped when their full-grid memory requirement is outside
  configured limits, rather than silently changing the grid;
* bitmap-inline runs use the same bit-packed GFP but compute successors inside
  the bitmap kernel, avoiding the uint32 transition-table limit.

Default cases mirror the active experiment table in ../docs/main.tex:
ACC, ACC-5D, Turn-Ego, and Turn-Onc.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Sequence


ROOT = Path(__file__).resolve().parents[1]
KERNEL_PACK = ROOT / "kernel-pack"
DEFAULT_OUT_PARENT = ROOT / "tools" / "benchmark_results" / "paper_all_methods"

AUTO_BEGIN = "% BEGIN AUTO-GENERATED ALL-METHOD PAPER EXPERIMENT TABLE"
AUTO_END = "% END AUTO-GENERATED ALL-METHOD PAPER EXPERIMENT TABLE"

UINT32_MAX_CELLS = 4_294_967_295


@dataclass(frozen=True)
class Example:
    key: str
    label: str
    cfg: Path
    default_scales: tuple[int, ...]
    max_basis_elements: int


@dataclass(frozen=True)
class GridCandidate:
    widths: tuple[int, ...]
    target_cells: int
    actual_cells: int
    rel_error_pct: float
    anisotropy: float
    source: str


@dataclass
class RunResult:
    example: str
    scale: str
    candidate_idx: int
    method: str
    d_star: str
    widths: tuple[int, ...]
    cfg_path: Path
    log_path: Path
    command: str
    status: str
    returncode: int | None
    wall_s: float
    iterations: int | None = None
    time_ms: int | None = None
    safe_cells: int | None = None
    safe_total: int | None = None
    safe_pct: float | None = None
    precompute_ms: int | None = None
    mem_alloc_mb: float | None = None
    tt_entries_reported: int | None = None
    tt_bytes_reported: int | None = None
    error: str = ""


EXAMPLES: dict[str, Example] = {
    "ACC": Example(
        key="ACC",
        label="ACC",
        cfg=ROOT / "examples" / "acc" / "acc.cfg",
        default_scales=(8, 9, 10, 11, 12, 13, 14),
        max_basis_elements=5_000_000,
    ),
    "ACC-5D": Example(
        key="ACC-5D",
        label="ACC-5D",
        cfg=ROOT / "examples" / "acc_5d" / "acc_5d.cfg",
        default_scales=(8, 9, 10, 11),
        max_basis_elements=5_000_000,
    ),
    "Turn-Ego": Example(
        key="Turn-Ego",
        label="Turn-Ego",
        cfg=ROOT / "examples" / "turn_ego_first" / "turn_ego_first.cfg",
        default_scales=(8, 9, 10, 11, 12, 13, 14),
        max_basis_elements=5_000_000,
    ),
    "Turn-Onc": Example(
        key="Turn-Onc",
        label="Turn-Onc",
        cfg=ROOT / "examples" / "turn_oncoming_first" / "turn_oncoming_first.cfg",
        default_scales=(8, 9, 10, 11, 12, 13, 14),
        max_basis_elements=5_000_000,
    ),
}

ACC_PREFERRED_WIDTHS: dict[int, tuple[int, int, int]] = {
    # Equal-width ACC refinements at these scales can collapse to an empty
    # safe set because of a discretization resonance. These deterministic
    # near-isotropic alternatives stretch headway, the physical clearance axis.
    9: (1033, 984, 984),
    10: (2432, 2027, 2027),
    11: (5241, 4368, 4368),
    12: (11293, 9410, 9410),
    13: (24329, 20274, 20274),
    14: (62501, 40001, 40001),
}


def strip_ansi(text: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*m", "", text)


def parse_state_block(cfg_text: str) -> dict[str, object]:
    match = re.search(r"states\s*\{([^}]*)\}", cfg_text, re.DOTALL)
    if not match:
        raise RuntimeError("states block not found")
    block = match.group(1)

    def read_list(name: str) -> list[float]:
        item = re.search(rf'{name}\s*=\s*"([^"]+)"', block)
        if not item:
            raise RuntimeError(f"states.{name} not found")
        return [float(x.strip()) for x in item.group(1).split(",")]

    dim_match = re.search(r'dim\s*=\s*"([^"]+)"', block)
    if not dim_match:
        raise RuntimeError("states.dim not found")
    return {
        "dim": int(dim_match.group(1)),
        "lb": read_list("lb"),
        "ub": read_list("ub"),
        "eta": read_list("eta"),
    }


def widths_from_eta(lb: Sequence[float], ub: Sequence[float], eta: Sequence[float]) -> tuple[int, ...]:
    return tuple(int(round((u - l) / e)) + 1 for l, u, e in zip(lb, ub, eta))


def eta_from_widths(lb: Sequence[float], ub: Sequence[float], widths: Sequence[int]) -> list[float]:
    return [(u - l) / (n - 1) for l, u, n in zip(lb, ub, widths)]


def product(values: Iterable[int]) -> int:
    out = 1
    for value in values:
        out *= int(value)
    return out


def table_entries(widths: Sequence[int], d_star: int) -> int:
    return product(w for i, w in enumerate(widths) if i != d_star)


def sci_scale(exp: int) -> str:
    return f"1e{exp}"


def fmt_int(value: int | None) -> str:
    return "" if value is None else str(value)


def fmt_float(value: float | None, digits: int = 3) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def make_grid_candidate(widths: Sequence[int], target_cells: int, source: str) -> GridCandidate | None:
    if any(w < 2 for w in widths):
        return None
    widths_tuple = tuple(int(w) for w in widths)
    actual = product(widths_tuple)
    rel_error = 100.0 * (actual - target_cells) / target_cells
    anisotropy = max(widths_tuple) / min(widths_tuple)
    return GridCandidate(
        widths=widths_tuple,
        target_cells=target_cells,
        actual_cells=actual,
        rel_error_pct=rel_error,
        anisotropy=anisotropy,
        source=source,
    )


def balanced_widths(target_cells: int, dim: int, max_tt_entries: int) -> list[tuple[int, ...]]:
    """Return near-isotropic seed grids that keep at least one d* table feasible."""
    root = target_cells ** (1.0 / dim)
    n0 = max(2, int(round(root)))
    seeds: list[tuple[int, ...]] = []

    equal = tuple([n0] * dim)
    if min(table_entries(equal, d) for d in range(dim)) <= max_tt_entries:
        seeds.append(equal)
    else:
        # Keep the non-designated table just below the configured cap and put the
        # remaining cells into one stretched axis. This is the closest safe shape
        # to equal-width when the threshold table itself is the limiting object.
        table_side = max(2, int(math.floor(max_tt_entries ** (1.0 / (dim - 1)))))
        table_dims = [table_side] * (dim - 1)
        table_prod = product(table_dims)
        long_dim = max(2, int(round(target_cells / table_prod)))
        for axis in range(dim):
            widths = list(table_dims)
            widths.insert(axis, long_dim)
            seeds.append(tuple(widths))

    # Add floor/ceil equal-width seeds too; these often break discretization
    # resonances without changing the grid family materially.
    for n in {max(2, int(math.floor(root))), max(2, int(math.ceil(root)))}:
        widths = tuple([n] * dim)
        if min(table_entries(widths, d) for d in range(dim)) <= max_tt_entries:
            seeds.append(widths)

    # Add deterministic near-isotropic stretched seeds. Some ACC refinements
    # exhibit zero-safe discretization resonances on exactly equal-width grids;
    # these preserve the same target cell count while trying mild anisotropy.
    for stretch in (1.05, 1.10, 1.20, 1.50, 2.00):
        short_side = max(2, int(round((target_cells / stretch) ** (1.0 / dim))))
        long_side = max(2, int(round(stretch * short_side)))
        for axis in range(dim):
            widths = [short_side] * dim
            widths[axis] = long_side
            widths_tuple = tuple(widths)
            if min(table_entries(widths_tuple, d) for d in range(dim)) <= max_tt_entries:
                seeds.append(widths_tuple)

    return seeds


def perturb_widths(
    seeds: Sequence[tuple[int, ...]],
    target_cells: int,
    max_tt_entries: int,
    max_anisotropy: float,
    perturb_radius: int,
    max_candidates: int,
) -> list[GridCandidate]:
    candidates: dict[tuple[int, ...], GridCandidate] = {}

    def add(widths: Sequence[int], source: str) -> None:
        item = make_grid_candidate(widths, target_cells, source)
        if item is None:
            return
        if item.anisotropy > max_anisotropy:
            return
        if min(table_entries(item.widths, d) for d in range(len(item.widths))) > max_tt_entries:
            return
        candidates.setdefault(item.widths, item)

    for seed_idx, seed in enumerate(seeds):
        add(seed, f"seed{seed_idx}")

        # Small per-axis bin changes.
        for radius in range(1, perturb_radius + 1):
            for deltas in itertools.product(range(-radius, radius + 1), repeat=len(seed)):
                if max(abs(delta) for delta in deltas) != radius:
                    continue
                widths = [max(2, w + d) for w, d in zip(seed, deltas)]
                add(widths, f"seed{seed_idx}:delta{deltas}")

        # When one axis is stretched to satisfy table memory, keep the smaller
        # dimensions close together and recompute the stretched axis to preserve
        # the target cell count.
        for axis in range(len(seed)):
            others = [i for i in range(len(seed)) if i != axis]
            for delta in range(-perturb_radius, perturb_radius + 1):
                widths = list(seed)
                for i in others:
                    widths[i] = max(2, widths[i] + delta)
                other_prod = product(widths[i] for i in others)
                widths[axis] = max(2, int(round(target_cells / other_prod)))
                add(widths, f"seed{seed_idx}:balanced_axis{axis}_delta{delta}")

    ordered = sorted(
        candidates.values(),
        key=lambda c: (":" in c.source, c.anisotropy, abs(c.rel_error_pct), c.actual_cells, c.widths),
    )
    return ordered[:max_candidates]


def default_main_tex_path() -> Path:
    for candidate in (ROOT / "docs" / "main.tex", ROOT.parent / "docs" / "main.tex"):
        if candidate.exists():
            return candidate
    return ROOT.parent / "docs" / "main.tex"


def read_paper_case_matrix(main_tex: Path) -> dict[str, set[int]]:
    """Best-effort parser for the active tab:vs-lazy table in main.tex."""
    if not main_tex.exists():
        return {}
    text = main_tex.read_text()
    label_pos = text.find(r"\label{tab:vs-lazy}")
    if label_pos < 0:
        return {}
    begin_pos = text.rfind(r"\begin{table", 0, label_pos)
    end_pos = text.find(r"\end{table}", label_pos)
    if begin_pos < 0 or end_pos < 0:
        return {}
    table = text[begin_pos:end_pos]
    found: dict[str, set[int]] = {}
    for line in table.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("%"):
            continue
        match = re.match(r"([A-Za-z0-9-]+)\s*&\s*\$10\^\{(\d+)\}\$", stripped)
        if match:
            found.setdefault(match.group(1), set()).add(int(match.group(2)))
    return found


def patch_cfg(
    base_text: str,
    *,
    project_name: str,
    dynamics_file: Path,
    eta: Sequence[float],
    method: str,
    benchmark_count: int,
    d_star: int | None,
    max_basis_elements: int,
    tt_inline: bool,
    prefix_sweep: bool,
) -> str:
    strip_keys = {
        "project_name",
        "user_dynamics_file",
        "benchmark_count",
        "record_basis_evolution",
        "save_transitions",
        "save_controller",
        "use_threshold_table",
        "use_tt_only",
        "use_tt_only_gpu",
        "use_inline_dynamics",
        "use_prefix_sweep",
        "use_bitmap_gfp",
        "threshold_d_star",
        "max_basis_elements",
    }

    out: list[str] = [
        f'project_name = "{project_name}";\n',
        f'user_dynamics_file = "{dynamics_file.resolve()}";\n',
    ]
    in_states = False
    eta_done = False
    for line in base_text.splitlines(keepends=True):
        stripped = line.strip()
        key_match = re.match(r"^([A-Za-z_]\w*)\s*=", stripped)
        if key_match and key_match.group(1) in strip_keys:
            continue

        if re.match(r"states\s*\{", stripped):
            in_states = True
        elif in_states and stripped == "}":
            in_states = False

        if in_states and not eta_done and re.match(r'eta\s*=\s*"', stripped):
            indent = line[: len(line) - len(line.lstrip())]
            eta_str = ",".join(f"{x:.18g}" for x in eta)
            out.append(f'{indent}eta = "{eta_str}";\n')
            eta_done = True
        else:
            out.append(line)

    out.extend(
        [
            "\n# Generated by tools/run_paper_experiments_all_methods.py\n",
            f'benchmark_count = "{benchmark_count}";\n',
            'record_basis_evolution = "false";\n',
            'save_transitions = "false";\n',
            'save_controller = "false";\n',
            f'max_basis_elements = "{max_basis_elements}";\n',
        ]
    )
    if d_star is not None:
        out.append(f'threshold_d_star = "{d_star}";\n')

    if method == "threshold":
        out.extend(
            [
                'use_threshold_table = "true";\n',
                'use_tt_only = "true";\n',
                'use_tt_only_gpu = "true";\n',
                f'use_inline_dynamics = "{str(tt_inline).lower()}";\n',
                f'use_prefix_sweep = "{str(prefix_sweep).lower()}";\n',
                'use_bitmap_gfp = "false";\n',
            ]
        )
    elif method == "bitmap":
        out.extend(
            [
                'use_threshold_table = "false";\n',
                'use_tt_only = "false";\n',
                'use_tt_only_gpu = "true";\n',
                'use_inline_dynamics = "false";\n',
                'use_prefix_sweep = "false";\n',
                'use_bitmap_gfp = "true";\n',
            ]
        )
    elif method == "bitmap_inline":
        out.extend(
            [
                'use_threshold_table = "false";\n',
                'use_tt_only = "false";\n',
                'use_tt_only_gpu = "true";\n',
                'use_inline_dynamics = "true";\n',
                'use_prefix_sweep = "false";\n',
                'use_bitmap_gfp = "true";\n',
            ]
        )
    elif method == "lazy":
        out.extend(
            [
                'use_threshold_table = "true";\n',
                'use_tt_only = "false";\n',
                'use_tt_only_gpu = "false";\n',
                'use_inline_dynamics = "false";\n',
                'use_prefix_sweep = "false";\n',
                'use_bitmap_gfp = "false";\n',
            ]
        )
    else:
        raise ValueError(method)

    return "".join(out)


def pfaces_command(cfg_path: Path, device: str, verbose: int) -> list[str]:
    return [
        "pfaces",
        "-G",
        "-k",
        f"mono_synth.gpu@{KERNEL_PACK.resolve()}",
        "-cfg",
        str(cfg_path.resolve()),
        "-d",
        device,
        "-p",
        f"-v{verbose}",
    ]


def parse_output(output: str) -> dict[str, object]:
    text = strip_ansi(output)
    metrics: dict[str, object] = {}

    avg = re.search(r"Average:\s*(\d+)\s+iterations,\s*(\d+)\s+ms", text)
    run = re.findall(r"Run\s+\d+/\d+:\s+(\d+)\s+iterations(?:\s+\([^)]+\))?.*?,\s*(\d+)\s+ms", text)
    if avg:
        metrics["iterations"] = int(avg.group(1))
        metrics["time_ms"] = int(avg.group(2))
    elif run:
        metrics["iterations"] = int(run[-1][0])
        metrics["time_ms"] = int(run[-1][1])

    safe = re.findall(r"Safe cells:\s*(\d+)/(\d+)\s*\(([-+0-9.eE]+)%\)", text)
    if safe:
        metrics["safe_cells"] = int(safe[-1][0])
        metrics["safe_total"] = int(safe[-1][1])
        metrics["safe_pct"] = float(safe[-1][2])

    pre = re.search(r"Precompute phase:\s*(\d+)\s+ms", text)
    if pre:
        metrics["precompute_ms"] = int(pre.group(1))

    mem = re.search(r"Total Alloc\.\s+\(MB\):\s*([0-9.]+)", text)
    if mem:
        metrics["mem_alloc_mb"] = float(mem.group(1))

    tt = re.search(r"table=(\d+)\s+entries,\s+(\d+)\s+bytes", text)
    if tt:
        metrics["tt_entries_reported"] = int(tt.group(1))
        metrics["tt_bytes_reported"] = int(tt.group(2))

    return metrics


def run_case(
    *,
    example: Example,
    scale: str,
    candidate_idx: int,
    widths: tuple[int, ...],
    method: str,
    d_star: int | None,
    cfg_text: str,
    out_dir: Path,
    device: str,
    timeout_s: int,
    dry_run: bool,
    verbose: int,
) -> RunResult:
    safe_name = example.key.lower().replace("-", "_")
    d_part = "na" if d_star is None else str(d_star)
    stem = f"{safe_name}_{scale}_cand{candidate_idx:02d}_{method}_d{d_part}"
    cfg_path = out_dir / f"{stem}.cfg"
    log_path = out_dir / f"{stem}.log"
    cfg_path.write_text(cfg_text)

    cmd = pfaces_command(cfg_path, device, verbose)
    shell_cmd = " ".join(shlex.quote(x) for x in cmd)

    if dry_run:
        return RunResult(
            example=example.key,
            scale=scale,
            candidate_idx=candidate_idx,
            method=method,
            d_star=d_part,
            widths=widths,
            cfg_path=cfg_path,
            log_path=log_path,
            command=shell_cmd,
            status="dry-run",
            returncode=None,
            wall_s=0.0,
        )

    stats_path = out_dir / "iteration_stats.csv"
    if stats_path.exists():
        stats_path.unlink()

    start = time.time()
    proc: subprocess.Popen[str] | None = None
    try:
        proc = subprocess.Popen(
            ["bash", "-lc", shell_cmd],
            cwd=out_dir,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        stdout, _ = proc.communicate(timeout=timeout_s)
        wall_s = time.time() - start
        output = stdout or ""
        log_path.write_text(output)
        metrics = parse_output(output)
        status = "ok" if proc.returncode == 0 else "error"
        if status == "ok" and metrics.get("safe_cells") == 0:
            status = "zero-safe"
        if status == "ok" and (
            "iterations" not in metrics or "time_ms" not in metrics or "safe_cells" not in metrics
        ):
            status = "parse-error"
        result = RunResult(
            example=example.key,
            scale=scale,
            candidate_idx=candidate_idx,
            method=method,
            d_star=d_part,
            widths=widths,
            cfg_path=cfg_path,
            log_path=log_path,
            command=shell_cmd,
            status=status,
            returncode=proc.returncode,
            wall_s=wall_s,
            error="" if status in {"ok", "zero-safe"} else "pfaces failed or output was incomplete",
        )
        for key, value in metrics.items():
            setattr(result, key, value)
        return result
    except subprocess.TimeoutExpired:
        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stdout, _ = proc.communicate()
        else:
            stdout = ""
        wall_s = time.time() - start
        log_path.write_text((stdout or "") + f"\nTIMEOUT after {timeout_s} seconds\n")
        return RunResult(
            example=example.key,
            scale=scale,
            candidate_idx=candidate_idx,
            method=method,
            d_star=d_part,
            widths=widths,
            cfg_path=cfg_path,
            log_path=log_path,
            command=shell_cmd,
            status="timeout",
            returncode=None,
            wall_s=wall_s,
            error=f"timeout after {timeout_s}s",
        )
    finally:
        if stats_path.exists():
            stats_path.rename(out_dir / f"{stem}_iteration_stats.csv")


def result_to_row(result: RunResult, target_cells: int, actual_cells: int, anisotropy: float) -> dict[str, str]:
    return {
        "example": result.example,
        "scale": result.scale,
        "candidate_idx": str(result.candidate_idx),
        "method": result.method,
        "d_star": result.d_star,
        "widths": "x".join(str(w) for w in result.widths),
        "target_cells": str(target_cells),
        "actual_cells": str(actual_cells),
        "relative_error_pct": f"{100.0 * (actual_cells - target_cells) / target_cells:.6g}",
        "anisotropy": f"{anisotropy:.6g}",
        "iterations": fmt_int(result.iterations),
        "time_ms": fmt_int(result.time_ms),
        "safe_cells": fmt_int(result.safe_cells),
        "safe_total": fmt_int(result.safe_total),
        "safe_pct": fmt_float(result.safe_pct, 6),
        "precompute_ms": fmt_int(result.precompute_ms),
        "mem_alloc_mb": fmt_float(result.mem_alloc_mb, 3),
        "tt_entries_reported": fmt_int(result.tt_entries_reported),
        "tt_bytes_reported": fmt_int(result.tt_bytes_reported),
        "wall_s": f"{result.wall_s:.3f}",
        "status": result.status,
        "returncode": "" if result.returncode is None else str(result.returncode),
        "cfg": str(result.cfg_path),
        "log": str(result.log_path),
        "command": result.command,
        "error": result.error,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    if not rows:
        path.write_text("")
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def summary_key(row: dict[str, str]) -> tuple[str, str]:
    return (row["example"], row["scale"])


def merge_summary_row(old: dict[str, str] | None, new: dict[str, str]) -> dict[str, str]:
    if old is None:
        merged = dict(new)
    else:
        merged = dict(old)
        for key, value in new.items():
            if value != "" or key not in merged:
                merged[key] = value
    recompute_summary_derived(merged)
    return merged


def recompute_summary_derived(row: dict[str, str]) -> None:
    for key in (
        "bitmap_inline_status",
        "bitmap_inline_R",
        "bitmap_inline_t_ms",
        "bitmap_inline_safe_cells",
        "bitmap_inline_safe_pct",
        "bitmap_inline_mem_mb",
        "bitmap_inline_log",
        "bitmap_inline_vs_threshold_safe_cells",
        "bitmap_inline_time_over_threshold",
    ):
        row.setdefault(key, "")

    threshold_safe = row.get("threshold_safe_cells", "")
    bitmap_safe = row.get("bitmap_safe_cells", "")
    bitmap_inline_safe = row.get("bitmap_inline_safe_cells", "")
    if threshold_safe and bitmap_safe:
        row["bitmap_vs_threshold_safe_cells"] = (
            "pass"
            if int(threshold_safe) == int(bitmap_safe)
            else ("zero" if int(bitmap_safe) == 0 else "mismatch")
        )
    if threshold_safe and bitmap_inline_safe:
        row["bitmap_inline_vs_threshold_safe_cells"] = (
            "pass"
            if int(threshold_safe) == int(bitmap_inline_safe)
            else ("zero" if int(bitmap_inline_safe) == 0 else "mismatch")
        )

    threshold_t = row.get("threshold_t_ms", "")
    lazy_t = row.get("lazy_t_ms", "")
    bitmap_t = row.get("bitmap_t_ms", "")
    bitmap_inline_t = row.get("bitmap_inline_t_ms", "")
    threshold_ms = int(threshold_t) if threshold_t else 0
    if threshold_ms > 0 and lazy_t:
        row["lazy_speedup_vs_threshold"] = f"{int(lazy_t) / threshold_ms:.3g}x"
    if threshold_ms > 0 and bitmap_t:
        row["bitmap_time_over_threshold"] = f"{int(bitmap_t) / threshold_ms:.3g}x"
    if threshold_ms > 0 and bitmap_inline_t:
        row["bitmap_inline_time_over_threshold"] = f"{int(bitmap_inline_t) / threshold_ms:.3g}x"


def select_best_threshold(results: list[RunResult]) -> RunResult | None:
    ok = [r for r in results if r.method == "threshold" and r.status == "ok" and (r.safe_cells or 0) > 0]
    if not ok:
        dry = [r for r in results if r.method == "threshold" and r.status == "dry-run"]
        return dry[0] if dry else None
    return sorted(ok, key=lambda r: (r.time_ms if r.time_ms is not None else 10**18, int(r.d_star)))[0]


def feasible_threshold_dstars(widths: Sequence[int], max_tt_entries: int) -> list[int]:
    return [d for d in range(len(widths)) if table_entries(widths, d) <= max_tt_entries]


def method_timeout(args: argparse.Namespace, method: str) -> int:
    if method == "threshold":
        return args.timeout_threshold
    if method == "bitmap":
        return args.timeout_bitmap
    if method == "bitmap_inline":
        return args.timeout_bitmap_inline
    if method == "lazy":
        return args.timeout_lazy
    raise ValueError(method)


def make_summary_row(
    example: Example,
    scale: str,
    candidate: GridCandidate,
    threshold: RunResult | None,
    threshold_trials: list[RunResult],
    bitmap: RunResult | None,
    bitmap_inline: RunResult | None,
    lazy: RunResult | None,
    candidate_status: str,
) -> dict[str, str]:
    nonempty_trials = [
        r for r in threshold_trials if r.status == "ok" and r.safe_cells is not None and r.safe_cells > 0
    ]
    safe_values = {r.safe_cells for r in nonempty_trials}
    axis_safe_check = ""
    if nonempty_trials:
        axis_safe_check = "pass" if len(safe_values) == 1 else "mismatch"
    elif any(r.status == "dry-run" for r in threshold_trials):
        axis_safe_check = "dry-run"

    row = {
        "example": example.key,
        "scale": scale,
        "widths": "x".join(str(w) for w in candidate.widths),
        "target_cells": str(candidate.target_cells),
        "actual_cells": str(candidate.actual_cells),
        "relative_error_pct": f"{candidate.rel_error_pct:.6g}",
        "anisotropy": f"{candidate.anisotropy:.6g}",
        "status": candidate_status,
        "threshold_axis_safe_check": axis_safe_check,
        "threshold_axis_trials": ";".join(
            f"d{r.d_star}:status={r.status},R={fmt_int(r.iterations)},t_ms={fmt_int(r.time_ms)},safe={fmt_int(r.safe_cells)}"
            for r in threshold_trials
        ),
    }

    if threshold:
        row.update(
            {
                "threshold_d_star": threshold.d_star,
                "threshold_R": fmt_int(threshold.iterations),
                "threshold_t_ms": fmt_int(threshold.time_ms),
                "threshold_safe_cells": fmt_int(threshold.safe_cells),
                "threshold_safe_pct": fmt_float(threshold.safe_pct, 6),
                "threshold_mem_mb": fmt_float(threshold.mem_alloc_mb, 3),
                "threshold_log": str(threshold.log_path),
            }
        )
    else:
        row.update(
            {
                "threshold_d_star": "",
                "threshold_R": "",
                "threshold_t_ms": "",
                "threshold_safe_cells": "",
                "threshold_safe_pct": "",
                "threshold_mem_mb": "",
                "threshold_log": "",
            }
        )

    for name, result in (("bitmap", bitmap), ("bitmap_inline", bitmap_inline), ("lazy", lazy)):
        row[f"{name}_status"] = "" if result is None else result.status
        row[f"{name}_R"] = "" if result is None else fmt_int(result.iterations)
        row[f"{name}_t_ms"] = "" if result is None else fmt_int(result.time_ms)
        row[f"{name}_safe_cells"] = "" if result is None else fmt_int(result.safe_cells)
        row[f"{name}_safe_pct"] = "" if result is None else fmt_float(result.safe_pct, 6)
        row[f"{name}_mem_mb"] = "" if result is None else fmt_float(result.mem_alloc_mb, 3)
        row[f"{name}_log"] = "" if result is None else str(result.log_path)

    equality = ""
    if threshold and bitmap and bitmap.safe_cells is not None and threshold.safe_cells is not None:
        equality = "pass" if bitmap.safe_cells == threshold.safe_cells else ("zero" if bitmap.safe_cells == 0 else "mismatch")
    row["bitmap_vs_threshold_safe_cells"] = equality

    inline_equality = ""
    if threshold and bitmap_inline and bitmap_inline.safe_cells is not None and threshold.safe_cells is not None:
        inline_equality = "pass" if bitmap_inline.safe_cells == threshold.safe_cells else ("zero" if bitmap_inline.safe_cells == 0 else "mismatch")
    row["bitmap_inline_vs_threshold_safe_cells"] = inline_equality

    lazy_speed = ""
    if threshold and lazy and threshold.time_ms and lazy.time_ms:
        lazy_speed = f"{lazy.time_ms / threshold.time_ms:.3g}x"
    bitmap_ratio = ""
    if threshold and bitmap and threshold.time_ms and bitmap.time_ms:
        bitmap_ratio = f"{bitmap.time_ms / threshold.time_ms:.3g}x"
    bitmap_inline_ratio = ""
    if threshold and bitmap_inline and threshold.time_ms and bitmap_inline.time_ms:
        bitmap_inline_ratio = f"{bitmap_inline.time_ms / threshold.time_ms:.3g}x"
    row["lazy_speedup_vs_threshold"] = lazy_speed
    row["bitmap_time_over_threshold"] = bitmap_ratio
    row["bitmap_inline_time_over_threshold"] = bitmap_inline_ratio

    return row


def write_markdown_summary(path: Path, rows: list[dict[str, str]]) -> None:
    def md_ms(value: str) -> str:
        return "" if not value else f"{value} ms"

    lines = [
        "# Paper Experiment Timing Summary",
        "",
        "| Example | Scale | Widths | d* | |X| | Ratio | Threshold R | Threshold t | Bitmap t | Bitmap inline t | Lazy t | Axis check | Bitmap check | Bitmap inline check | Status |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|---|",
    ]
    for row in rows:
        lines.append(
            "| {example} | {scale} | {widths} | {threshold_d_star} | {actual_cells} | "
            "{anisotropy} | {threshold_R} | {threshold_t} | {bitmap_t} | "
            "{bitmap_inline_t} | {lazy_t} | {threshold_axis_safe_check} | "
            "{bitmap_vs_threshold_safe_cells} | {bitmap_inline_vs_threshold_safe_cells} | {status} |".format(
                **row,
                threshold_t=md_ms(row["threshold_t_ms"]),
                bitmap_t=md_ms(row["bitmap_t_ms"]),
                bitmap_inline_t=md_ms(row.get("bitmap_inline_t_ms", "")),
                lazy_t=md_ms(row["lazy_t_ms"]),
            )
        )
    lines.extend(
        [
            "",
            "Notes:",
            "- Every method in a row uses the exact same generated `eta` and grid widths.",
            "- Threshold rows are selected after trying every feasible designated dimension `d*`.",
            "- `Axis check` compares safe-cell counts across the feasible threshold `d*` trials; mismatches are kept visible rather than hidden.",
            "- `zero-safe` threshold grids are rejected and nearby bin counts are tried before bitmap/lazy are run.",
            "- `Bitmap inline` is the bit-packed GFP run with transition dynamics evaluated in the bitmap kernel instead of using a precomputed successor table.",
            "- Bitmap and lazy methods are skipped when full-grid memory or uint32-index limits make them infeasible under the configured caps.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def tex_time(ms: str) -> str:
    if not ms:
        return "--"
    value = int(ms)
    if value >= 10_000:
        return f"{value / 1000.0:.1f}\\,s"
    return f"{value}\\,ms"


def tex_method_time(ms: str, status: str) -> str:
    if status == "timeout":
        return "TO"
    return tex_time(ms)


def tex_cells(cells: str) -> str:
    value = int(cells)
    exponent = int(math.floor(math.log10(value)))
    mantissa = value / (10**exponent)
    if round(mantissa, 2) >= 10.0:
        return f"$10^{{{exponent + 1}}}$"
    if abs(mantissa - 1.0) < 0.005:
        return f"$10^{{{exponent}}}$"
    return f"${mantissa:.2f}\\!\\times\\!10^{{{exponent}}}$"


def tex_grid(widths: str) -> str:
    return "$" + r"\!\times\!".join(widths.split("x")) + "$"


def build_latex_table(rows: list[dict[str, str]]) -> str:
    def check_tex(value: str) -> str:
        if not value:
            return "--"
        if value == "pass":
            return "pass"
        if value == "mismatch":
            return "diff"
        return value

    lines = [
        AUTO_BEGIN,
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{All-method timing comparison on near-isotropic grids generated by \texttt{tools/run\_paper\_experiments\_all\_methods.py}. Bitmap/lazy entries are reported only where the full-grid method is feasible under the configured memory limits. Axis/bitmap checks report whether safe-cell counts agree across threshold axes and with precomputed/inline bitmap GFP; ``zero'' marks a full-grid run whose fixed point collapsed to the empty set.}",
        r"\label{tab:vs-lazy}",
        r"\setlength{\tabcolsep}{2.2pt}",
        r"\renewcommand{\arraystretch}{1.05}",
        r"\scriptsize",
        r"\resizebox{\textwidth}{!}{%",
        r"\begin{tabular}{@{}llccccccccccccc@{}}",
        r"\toprule",
        r"System & $|X|$ & Grid & $d^*$ & Axis & BM & BMI & \multicolumn{2}{c}{Lazy} & \multicolumn{2}{c}{Threshold} & \multicolumn{2}{c}{Bitmap GFP} & \multicolumn{2}{c}{Bitmap inline} \\",
        r"\cmidrule(lr){8-9}\cmidrule(lr){10-11}\cmidrule(lr){12-13}\cmidrule(lr){14-15}",
        r" & & & & & & & $R$ & $t$ & $R$ & $t$ & $R$ & $t$ & $R$ & $t$ \\",
        r"\midrule",
    ]
    previous = None
    order = {key: i for i, key in enumerate(EXAMPLES)}
    sorted_rows = sorted(rows, key=lambda r: (order.get(r["example"], 99), int(r["scale"][2:])))
    for row in sorted_rows:
        if previous is not None and row["example"] != previous:
            lines.append(r"\midrule")
        previous = row["example"]
        lines.append(
            f"{row['example']} & {tex_cells(row['actual_cells'])} & "
            f"{tex_grid(row['widths'])} & "
            f"{row['threshold_d_star'] or '--'} & "
            f"{check_tex(row.get('threshold_axis_safe_check', ''))} & "
            f"{check_tex(row.get('bitmap_vs_threshold_safe_cells', ''))} & "
            f"{check_tex(row.get('bitmap_inline_vs_threshold_safe_cells', ''))} & "
            f"{row['lazy_R'] or '--'} & {tex_method_time(row['lazy_t_ms'], row.get('lazy_status', ''))} & "
            f"{row['threshold_R'] or '--'} & {tex_time(row['threshold_t_ms'])} & "
            f"{row['bitmap_R'] or '--'} & {tex_method_time(row['bitmap_t_ms'], row.get('bitmap_status', ''))} & "
            f"{row.get('bitmap_inline_R', '') or '--'} & "
            f"{tex_method_time(row.get('bitmap_inline_t_ms', ''), row.get('bitmap_inline_status', ''))} \\\\"
        )
    lines.extend(
        [
            r"\bottomrule",
            r"\end{tabular}",
            r"}",
            r"\end{table*}",
            AUTO_END,
            "",
        ]
    )
    return "\n".join(lines)


def update_main_tex(main_tex: Path, rows: list[dict[str, str]]) -> None:
    table = build_latex_table(rows)
    text = main_tex.read_text()
    if AUTO_BEGIN in text and AUTO_END in text:
        pattern = re.compile(re.escape(AUTO_BEGIN) + r".*?" + re.escape(AUTO_END), re.DOTALL)
        text = pattern.sub(lambda _m: table.rstrip(), text)
    else:
        label_pos = text.find(r"\label{tab:vs-lazy}")
        if label_pos < 0:
            raise RuntimeError(f"Could not find \\label{{tab:vs-lazy}} in {main_tex}")
        begin_pos = text.rfind(r"\begin{table", 0, label_pos)
        end_pos = text.find(r"\end{table}", label_pos)
        if begin_pos < 0 or end_pos < 0:
            raise RuntimeError(f"Could not isolate tab:vs-lazy table in {main_tex}")
        end_pos += len(r"\end{table}")
        text = text[:begin_pos] + table.rstrip() + text[end_pos:]
    main_tex.write_text(text)


def parse_scales(values: list[str] | None) -> set[int] | None:
    if not values:
        return None
    out: set[int] = set()
    for value in values:
        for part in value.split(","):
            part = part.strip().lower()
            if not part:
                continue
            if part.startswith("1e"):
                out.add(int(part[2:]))
            else:
                out.add(int(part))
    return out


def parse_examples(values: list[str] | None) -> set[str] | None:
    if not values:
        return None
    aliases = {
        "turn-oncoming": "Turn-Onc",
        "turn-onc": "Turn-Onc",
        "turn-ego": "Turn-Ego",
        "acc": "ACC",
        "acc-5d": "ACC-5D",
    }
    out: set[str] = set()
    for value in values:
        for part in value.split(","):
            item = part.strip()
            if not item:
                continue
            out.add(aliases.get(item.lower(), item))
    return out


def parse_methods(values: list[str] | None) -> set[str]:
    if not values:
        return {"threshold", "bitmap", "lazy"}
    aliases = {
        "bitmap-inline": "bitmap_inline",
        "bitmap_inline": "bitmap_inline",
        "inline-bitmap": "bitmap_inline",
        "inline_bitmap": "bitmap_inline",
        "threshold": "threshold",
        "bitmap": "bitmap",
        "lazy": "lazy",
    }
    allowed = {"threshold", "bitmap", "bitmap_inline", "lazy"}
    out: set[str] = set()
    for value in values:
        for part in value.split(","):
            method = aliases.get(part.strip().lower(), part.strip().lower())
            if method:
                if method not in allowed:
                    raise RuntimeError(f"Unknown method {method}; allowed: {sorted(allowed)}")
                out.add(method)
    return out


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--example", action="append", help="Example filter, comma-separated or repeated")
    parser.add_argument("--scale", action="append", help="Scale filter as 1e8 or 8, comma-separated or repeated")
    parser.add_argument("--methods", action="append", help="Methods: threshold,bitmap,bitmap-inline,lazy")
    parser.add_argument("--out", type=Path, default=None, help="Output directory. Default: timestamped benchmark_results folder")
    parser.add_argument("--device", default=os.environ.get("PFACES_GPU_DEVICE", "1"), help="pFaces GPU device index")
    parser.add_argument("--benchmark-count", type=int, default=1, help="pFaces benchmark_count")
    parser.add_argument("--timeout-threshold", type=int, default=1200, help="Threshold per-run timeout in seconds")
    parser.add_argument("--timeout-bitmap", type=int, default=1200, help="Bitmap per-run timeout in seconds")
    parser.add_argument("--timeout-bitmap-inline", type=int, default=1200, help="Bitmap-inline per-run timeout in seconds")
    parser.add_argument("--timeout-lazy", type=int, default=1200, help="Lazy per-run timeout in seconds")
    parser.add_argument("--max-tt-entries", type=int, default=1_600_000_000, help="Max threshold-table entries for any d*")
    parser.add_argument("--max-bitmap-cells", type=int, default=1_000_000_000, help="Max cells for bitmap GFP")
    parser.add_argument("--max-bitmap-inline-cells", type=int, default=10_500_000_000, help="Max cells for bitmap GFP with inline dynamics")
    parser.add_argument("--max-lazy-cells", type=int, default=1_000_000_000, help="Max cells for lazy/basis runs")
    parser.add_argument("--max-anisotropy", type=float, default=2.0, help="Reject generated grids above this max(width)/min(width)")
    parser.add_argument("--perturb-radius", type=int, default=3, help="Nearby bin-count search radius after zero-safe grids")
    parser.add_argument("--max-grid-candidates", type=int, default=24, help="Max grid candidates per example/scale")
    parser.add_argument("--tt-precompute", action="store_true", help="Use precomputed transitions for threshold mode instead of inline dynamics")
    parser.add_argument("--no-prefix-sweep", action="store_true", help="Disable threshold prefix sweep")
    parser.add_argument("--main-tex", type=Path, default=default_main_tex_path(), help="Paper main.tex path")
    parser.add_argument("--baseline-summary", type=Path, default=None, help="Existing summary.csv whose grids and rows are reused/merged for selected cases")
    parser.add_argument("--ignore-baseline-grids", action="store_true", help="Merge into --baseline-summary but generate fresh grid candidates for selected cases")
    parser.add_argument("--update-main-tex", action="store_true", help="Replace tab:vs-lazy in main.tex with generated LaTeX table")
    parser.add_argument("--dry-run", action="store_true", help="Generate configs and plan without running pFaces")
    parser.add_argument("--strict", action="store_true", help="Exit nonzero if any selected case lacks a nonzero threshold result")
    parser.add_argument("--verbose", type=int, default=2, choices=(1, 2), help="pFaces verbose level")
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    examples_filter = parse_examples(args.example)
    scales_filter = parse_scales(args.scale)
    methods = parse_methods(args.methods)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or (DEFAULT_OUT_PARENT / timestamp)
    out_dir.mkdir(parents=True, exist_ok=True)

    paper_matrix = read_paper_case_matrix(args.main_tex)
    selected_examples = [
        ex
        for key, ex in EXAMPLES.items()
        if examples_filter is None or key in examples_filter
    ]
    if not selected_examples:
        raise RuntimeError("No examples selected")

    run_rows: list[dict[str, str]] = []
    summary_rows: list[dict[str, str]] = []
    plan_rows: list[dict[str, str]] = []
    failures: list[str] = []

    manifest = {
        "root": str(ROOT),
        "kernel_pack": str(KERNEL_PACK),
        "main_tex": str(args.main_tex),
        "paper_matrix_from_main_tex": {k: sorted(v) for k, v in paper_matrix.items()},
        "methods": sorted(methods),
        "args": vars(args) | {"out": str(out_dir), "main_tex": str(args.main_tex)},
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n")

    print(f"[bench] output directory: {out_dir}", flush=True)
    print(f"[bench] methods: {', '.join(sorted(methods))}", flush=True)

    baseline_rows: list[dict[str, str]] = []
    baseline_by_key: dict[tuple[str, str], dict[str, str]] = {}
    summary_by_key: dict[tuple[str, str], dict[str, str]] = {}
    summary_order: list[tuple[str, str]] = []
    if args.baseline_summary is not None:
        baseline_rows = read_csv_rows(args.baseline_summary)
        for row in baseline_rows:
            recompute_summary_derived(row)
            key = summary_key(row)
            baseline_by_key[key] = row
            summary_by_key[key] = dict(row)
            summary_order.append(key)
        summary_rows = [summary_by_key[key] for key in summary_order]
        print(f"[bench] baseline summary: {args.baseline_summary} ({len(baseline_rows)} rows)", flush=True)

    for example in selected_examples:
        if not example.cfg.exists():
            print(f"[skip] {example.key}: missing config {example.cfg}", flush=True)
            continue

        base_text = example.cfg.read_text()
        state = parse_state_block(base_text)
        dim = int(state["dim"])
        lb = state["lb"]
        ub = state["ub"]
        assert isinstance(lb, list) and isinstance(ub, list)
        dynamics_file = example.cfg.parent / re.search(
            r'user_dynamics_file\s*=\s*"([^"]+)"', base_text
        ).group(1)

        scales = example.default_scales
        if scales_filter is not None:
            scales = tuple(sorted(scales_filter))
        if not scales:
            continue

        for exp in scales:
            scale = sci_scale(exp)
            target_cells = 10**exp
            baseline_row = baseline_by_key.get((example.key, scale))
            if baseline_row is not None and not args.ignore_baseline_grids:
                widths = tuple(int(x) for x in baseline_row["widths"].split("x"))
                candidate = make_grid_candidate(widths, target_cells, "baseline_summary")
                if candidate is None:
                    failures.append(f"{example.key} {scale}: invalid baseline grid {baseline_row['widths']}")
                    continue
                candidates = [candidate]
            else:
                seeds = balanced_widths(target_cells, dim, args.max_tt_entries)
                candidates = perturb_widths(
                    seeds,
                    target_cells,
                    max_tt_entries=args.max_tt_entries,
                    max_anisotropy=args.max_anisotropy,
                    perturb_radius=args.perturb_radius,
                    max_candidates=args.max_grid_candidates,
                )
            if (baseline_row is None or args.ignore_baseline_grids) and example.key == "ACC" and exp >= 9:
                # ACC equal-width refinements at and above 1e9 are prone to an
                # empty-safe-set discretization resonance. Prefer the mild
                # stretched seeds first; equal-width candidates remain available
                # later in the deterministic list if the stretched grids fail.
                preferred_widths = ACC_PREFERRED_WIDTHS.get(exp)
                if preferred_widths is not None:
                    preferred = make_grid_candidate(
                        preferred_widths,
                        target_cells,
                        f"acc_preferred_1e{exp}",
                    )
                    if (
                        preferred is not None
                        and preferred.anisotropy <= args.max_anisotropy
                        and min(table_entries(preferred.widths, d) for d in range(dim))
                        <= args.max_tt_entries
                    ):
                        candidates = [preferred] + [
                            c for c in candidates if c.widths != preferred.widths
                        ]
                candidates = sorted(
                    candidates,
                    key=lambda c: (
                        not c.source.startswith("acc_preferred_"),
                        ":" in c.source,
                        c.anisotropy <= 1.000001,
                        abs(c.anisotropy - 1.05),
                        abs(c.rel_error_pct),
                        c.widths,
                    ),
                )
            if not candidates:
                failures.append(f"{example.key} {scale}: no grid candidates under caps")
                continue

            final_summary: dict[str, str] | None = None
            for cand_idx, candidate in enumerate(candidates):
                plan_rows.append(
                    {
                        "example": example.key,
                        "scale": scale,
                        "candidate_idx": str(cand_idx),
                        "widths": "x".join(str(w) for w in candidate.widths),
                        "target_cells": str(target_cells),
                        "actual_cells": str(candidate.actual_cells),
                        "relative_error_pct": f"{candidate.rel_error_pct:.6g}",
                        "anisotropy": f"{candidate.anisotropy:.6g}",
                        "source": candidate.source,
                        "feasible_dstars": ",".join(
                            str(d) for d in feasible_threshold_dstars(candidate.widths, args.max_tt_entries)
                        ),
                    }
                )

                print(
                    f"[grid] {example.key:9s} {scale:>4s} cand={cand_idx:02d} "
                    f"widths={'x'.join(map(str, candidate.widths))} "
                    f"|X|={candidate.actual_cells} ratio={candidate.anisotropy:.3f}",
                    flush=True,
                )

                eta = eta_from_widths(lb, ub, candidate.widths)
                threshold_results: list[RunResult] = []
                for d_star in feasible_threshold_dstars(candidate.widths, args.max_tt_entries):
                    if "threshold" not in methods:
                        continue
                    cfg_text = patch_cfg(
                        base_text,
                        project_name=f"{example.key}_{scale}_threshold_d{d_star}",
                        dynamics_file=dynamics_file,
                        eta=eta,
                        method="threshold",
                        benchmark_count=args.benchmark_count,
                        d_star=d_star,
                        max_basis_elements=example.max_basis_elements,
                        tt_inline=not args.tt_precompute,
                        prefix_sweep=not args.no_prefix_sweep,
                    )
                    print(f"[run]  {example.key:9s} {scale:>4s} threshold d*={d_star}", flush=True)
                    result = run_case(
                        example=example,
                        scale=scale,
                        candidate_idx=cand_idx,
                        widths=candidate.widths,
                        method="threshold",
                        d_star=d_star,
                        cfg_text=cfg_text,
                        out_dir=out_dir,
                        device=args.device,
                        timeout_s=method_timeout(args, "threshold"),
                        dry_run=args.dry_run,
                        verbose=args.verbose,
                    )
                    threshold_results.append(result)
                    run_rows.append(
                        result_to_row(result, target_cells, candidate.actual_cells, candidate.anisotropy)
                    )
                    print(
                        f"[{result.status}] threshold d*={d_star} "
                        f"R={result.iterations or '--'} t={result.time_ms or '--'} ms "
                        f"safe={result.safe_cells if result.safe_cells is not None else '--'}",
                        flush=True,
                    )

                best_threshold = select_best_threshold(threshold_results)
                if "threshold" in methods and best_threshold is None:
                    print(f"[grid] {example.key} {scale} cand={cand_idx:02d}: no nonzero threshold result", flush=True)
                    continue

                bitmap_result: RunResult | None = None
                bitmap_inline_result: RunResult | None = None
                lazy_result: RunResult | None = None
                chosen_d = None if best_threshold is None else int(best_threshold.d_star)
                if chosen_d is None and baseline_row is not None and baseline_row.get("threshold_d_star", ""):
                    chosen_d = int(baseline_row["threshold_d_star"])

                if "bitmap" in methods:
                    if candidate.actual_cells > args.max_bitmap_cells or candidate.actual_cells > UINT32_MAX_CELLS:
                        print(f"[skip] {example.key} {scale} bitmap: full grid exceeds configured bitmap cap", flush=True)
                    else:
                        cfg_text = patch_cfg(
                            base_text,
                            project_name=f"{example.key}_{scale}_bitmap",
                            dynamics_file=dynamics_file,
                            eta=eta,
                            method="bitmap",
                            benchmark_count=args.benchmark_count,
                            d_star=chosen_d,
                            max_basis_elements=example.max_basis_elements,
                            tt_inline=False,
                            prefix_sweep=False,
                        )
                        print(f"[run]  {example.key:9s} {scale:>4s} bitmap", flush=True)
                        bitmap_result = run_case(
                            example=example,
                            scale=scale,
                            candidate_idx=cand_idx,
                            widths=candidate.widths,
                            method="bitmap",
                            d_star=chosen_d,
                            cfg_text=cfg_text,
                            out_dir=out_dir,
                            device=args.device,
                            timeout_s=method_timeout(args, "bitmap"),
                            dry_run=args.dry_run,
                            verbose=args.verbose,
                        )
                        run_rows.append(
                            result_to_row(bitmap_result, target_cells, candidate.actual_cells, candidate.anisotropy)
                        )
                        print(
                            f"[{bitmap_result.status}] bitmap "
                            f"R={bitmap_result.iterations or '--'} t={bitmap_result.time_ms or '--'} ms "
                            f"safe={bitmap_result.safe_cells if bitmap_result.safe_cells is not None else '--'}",
                            flush=True,
                        )

                if "bitmap_inline" in methods:
                    if candidate.actual_cells > args.max_bitmap_inline_cells:
                        print(f"[skip] {example.key} {scale} bitmap-inline: full grid exceeds configured inline bitmap cap", flush=True)
                    else:
                        cfg_text = patch_cfg(
                            base_text,
                            project_name=f"{example.key}_{scale}_bitmap_inline",
                            dynamics_file=dynamics_file,
                            eta=eta,
                            method="bitmap_inline",
                            benchmark_count=args.benchmark_count,
                            d_star=chosen_d,
                            max_basis_elements=example.max_basis_elements,
                            tt_inline=True,
                            prefix_sweep=False,
                        )
                        print(f"[run]  {example.key:9s} {scale:>4s} bitmap-inline", flush=True)
                        bitmap_inline_result = run_case(
                            example=example,
                            scale=scale,
                            candidate_idx=cand_idx,
                            widths=candidate.widths,
                            method="bitmap_inline",
                            d_star=chosen_d,
                            cfg_text=cfg_text,
                            out_dir=out_dir,
                            device=args.device,
                            timeout_s=method_timeout(args, "bitmap_inline"),
                            dry_run=args.dry_run,
                            verbose=args.verbose,
                        )
                        run_rows.append(
                            result_to_row(bitmap_inline_result, target_cells, candidate.actual_cells, candidate.anisotropy)
                        )
                        print(
                            f"[{bitmap_inline_result.status}] bitmap-inline "
                            f"R={bitmap_inline_result.iterations or '--'} t={bitmap_inline_result.time_ms or '--'} ms "
                            f"safe={bitmap_inline_result.safe_cells if bitmap_inline_result.safe_cells is not None else '--'}",
                            flush=True,
                        )

                if "lazy" in methods:
                    if candidate.actual_cells > args.max_lazy_cells or candidate.actual_cells > UINT32_MAX_CELLS:
                        print(f"[skip] {example.key} {scale} lazy: full grid exceeds configured lazy cap", flush=True)
                    else:
                        cfg_text = patch_cfg(
                            base_text,
                            project_name=f"{example.key}_{scale}_lazy",
                            dynamics_file=dynamics_file,
                            eta=eta,
                            method="lazy",
                            benchmark_count=args.benchmark_count,
                            d_star=chosen_d,
                            max_basis_elements=example.max_basis_elements,
                            tt_inline=False,
                            prefix_sweep=False,
                        )
                        print(f"[run]  {example.key:9s} {scale:>4s} lazy", flush=True)
                        lazy_result = run_case(
                            example=example,
                            scale=scale,
                            candidate_idx=cand_idx,
                            widths=candidate.widths,
                            method="lazy",
                            d_star=chosen_d,
                            cfg_text=cfg_text,
                            out_dir=out_dir,
                            device=args.device,
                            timeout_s=method_timeout(args, "lazy"),
                            dry_run=args.dry_run,
                            verbose=args.verbose,
                        )
                        run_rows.append(
                            result_to_row(lazy_result, target_cells, candidate.actual_cells, candidate.anisotropy)
                        )
                        print(
                            f"[{lazy_result.status}] lazy "
                            f"R={lazy_result.iterations or '--'} t={lazy_result.time_ms or '--'} ms "
                            f"safe={lazy_result.safe_cells if lazy_result.safe_cells is not None else '--'}",
                            flush=True,
                        )

                zero_safe_method = next(
                    (
                        name
                        for name, result in (
                            ("bitmap", bitmap_result),
                            ("bitmap-inline", bitmap_inline_result),
                            ("lazy", lazy_result),
                        )
                        if result is not None and result.status == "zero-safe"
                    ),
                    None,
                )
                if zero_safe_method is not None:
                    print(
                        f"[grid] {example.key} {scale} cand={cand_idx:02d}: "
                        f"{zero_safe_method} returned a zero safe set; trying the next grid",
                        flush=True,
                    )
                    continue

                candidate_status = "dry-run" if args.dry_run else "ok"
                final_summary = make_summary_row(
                    example,
                    scale,
                    candidate,
                    best_threshold,
                    threshold_results,
                    bitmap_result,
                    bitmap_inline_result,
                    lazy_result,
                    candidate_status,
                )
                break

            if final_summary is None:
                failures.append(f"{example.key} {scale}: no nonzero threshold grid found")
            else:
                key = summary_key(final_summary)
                if args.baseline_summary is not None:
                    if key not in summary_by_key:
                        summary_order.append(key)
                    summary_by_key[key] = merge_summary_row(summary_by_key.get(key), final_summary)
                    summary_rows = [summary_by_key[item] for item in summary_order]
                else:
                    recompute_summary_derived(final_summary)
                    summary_rows.append(final_summary)

            write_csv(out_dir / "all_runs.csv", run_rows)
            write_csv(out_dir / "grid_plan.csv", plan_rows)
            write_csv(out_dir / "summary.csv", summary_rows)
            write_markdown_summary(out_dir / "summary.md", summary_rows)

    for row in summary_rows:
        recompute_summary_derived(row)

    write_csv(out_dir / "all_runs.csv", run_rows)
    write_csv(out_dir / "grid_plan.csv", plan_rows)
    write_csv(out_dir / "summary.csv", summary_rows)
    write_markdown_summary(out_dir / "summary.md", summary_rows)
    (out_dir / "paper_table.tex").write_text(build_latex_table(summary_rows))

    if args.update_main_tex:
        update_main_tex(args.main_tex, summary_rows)
        print(f"[bench] updated {args.main_tex}", flush=True)

    print(f"[bench] wrote {out_dir / 'summary.csv'}", flush=True)
    print(f"[bench] wrote {out_dir / 'summary.md'}", flush=True)
    print(f"[bench] wrote {out_dir / 'paper_table.tex'}", flush=True)

    if failures:
        print("[bench] unresolved cases:", flush=True)
        for failure in failures:
            print(f"  - {failure}", flush=True)
        if args.strict:
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
