#!/usr/bin/env python3
"""Benchmark ACC 3D TT-only versus bitmap GFP on large uniform grids."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path


CASES = {
    "1e8": 464,    # 464^3 = 99,897,344
    "1e9": 1000,   # 1000^3
}


def patch_cfg(src: str, *, name: str, eta: str, dynamics_file: Path, method: str) -> str:
    text = src
    text = re.sub(r'project_name\s*=\s*"[^"]*";', f'project_name = "{name}";', text)
    text = re.sub(
        r'user_dynamics_file\s*=\s*"[^"]*";',
        f'user_dynamics_file = "{dynamics_file}";',
        text,
    )
    for key in (
        "benchmark_count",
        "record_basis_evolution",
        "save_transitions",
        "use_inline_dynamics",
        "use_prefix_sweep",
        "max_basis_elements",
    ):
        text = re.sub(rf'\n{key}\s*=\s*"[^"]*";[^\n]*', "", text)

    text = re.sub(
        r'(states\s*\{.*?eta\s*=\s*)"[^"]*";',
        rf'\g<1>"{eta}";',
        text,
        count=1,
        flags=re.S,
    )

    for key in ("use_tt_only", "use_bitmap_gfp", "use_threshold_table"):
        text = re.sub(rf'\n{key}\s*=\s*"[^"]*";', "", text)

    text += (
        '\nbenchmark_count = "1";\n'
        'record_basis_evolution = "false";\n'
        'save_transitions = "false";\n'
        'use_inline_dynamics = "false";\n'
        'max_basis_elements = "1000000";\n'
    )
    if method == "tt":
        text += 'use_prefix_sweep = "true";\n'
    else:
        text += 'use_prefix_sweep = "false";\n'

    if method == "tt":
        text += 'use_threshold_table = "true";\nuse_tt_only = "true";\nuse_bitmap_gfp = "false";\n'
    elif method == "bitmap":
        text += 'use_threshold_table = "false";\nuse_tt_only = "false";\nuse_bitmap_gfp = "true";\n'
    else:
        raise ValueError(method)
    return text


def run_case(cfg: Path, kernel_pack: Path, device: str, cwd: Path) -> tuple[str, dict[str, str]]:
    cmd = [
        "pfaces",
        "-G",
        "-k",
        f"mono_synth.gpu@{kernel_pack}",
        "-cfg",
        str(cfg),
        "-d",
        device,
        "-p",
        "-v1",
    ]
    shell_cmd = " ".join(shlex.quote(x) for x in cmd)
    proc = subprocess.run(
        ["bash", "-lc", shell_cmd],
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if proc.returncode != 0:
        print(proc.stdout)
        raise RuntimeError(f"benchmark failed for {cfg}")

    metrics: dict[str, str] = {}
    avg = re.search(r"Average:\s+(\d+)\s+iterations,\s+(\d+)\s+ms", proc.stdout)
    run = re.search(r"Run 1/1:\s+(\d+)\s+iterations.*?,.*?(\d+)\s+ms", proc.stdout)
    safe = re.search(r"Safe cells:\s+(\d+)/(\d+)\s+\(([0-9.]+)%\)", proc.stdout)
    if avg:
        metrics["iterations"] = avg.group(1)
        metrics["time_ms"] = avg.group(2)
    elif run:
        metrics["iterations"] = run.group(1)
        metrics["time_ms"] = run.group(2)
    if safe:
        metrics["safe_cells"] = safe.group(1)
        metrics["total_cells"] = safe.group(2)
        metrics["safe_percent"] = safe.group(3)
    metrics["stdout"] = proc.stdout
    return shell_cmd, metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sizes", nargs="+", choices=CASES.keys(), default=["1e8", "1e9"])
    parser.add_argument("--out", default="tools/benchmark_results/bitmap_gfp_acc")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    acc_cfg = root / "examples" / "acc" / "acc.cfg"
    dynamics = (root / "examples" / "acc" / "acc_dynamics.cl").resolve()
    kernel_pack = (root / "kernel-pack").resolve()
    device = os.environ.get("PFACES_GPU_DEVICE", "1")
    out_dir = (root / args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    base_cfg = acc_cfg.read_text()
    rows: list[dict[str, str]] = []

    for label in args.sizes:
        n = CASES[label]
        total = n ** 3
        eta_vals = [80.0 / (n - 1), 20.0 / (n - 1), 20.0 / (n - 1)]
        eta = ",".join(f"{v:.12g}" for v in eta_vals)
        case_rows: list[dict[str, str]] = []

        for method in ("tt", "bitmap"):
            name = f"acc_{label}_{method}"
            cfg_path = out_dir / f"{name}.cfg"
            cfg_path.write_text(
                patch_cfg(base_cfg, name=name, eta=eta, dynamics_file=dynamics, method=method)
            )
            print(f"[bench] {label} {method}: n={n}, total={total}")
            cmd, metrics = run_case(cfg_path, kernel_pack, device, out_dir)
            (out_dir / f"{name}.log").write_text(metrics.pop("stdout"))
            row = {"case": label, "method": method, "n_per_dim": str(n), "command": cmd}
            row.update(metrics)
            rows.append(row)
            case_rows.append(row)
            print(json.dumps(row, indent=2))

        if len(case_rows) == 2:
            tt_row, bitmap_row = case_rows
            for key in ("safe_cells", "total_cells", "iterations"):
                if tt_row.get(key) != bitmap_row.get(key):
                    raise RuntimeError(
                        f"{label}: TT and bitmap disagree on {key}: "
                        f"{tt_row.get(key)} vs {bitmap_row.get(key)}"
                    )
            print(
                f"[bench] {label}: PASS, bitmap matches threshold table "
                f"({tt_row.get('safe_cells')}/{tt_row.get('total_cells')} safe cells, "
                f"{tt_row.get('iterations')} iterations)"
            )

    report = out_dir / "results.json"
    report.write_text(json.dumps(rows, indent=2) + "\n")
    print(f"[bench] wrote {report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
