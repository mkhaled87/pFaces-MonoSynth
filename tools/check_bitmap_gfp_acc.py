#!/usr/bin/env python3
"""Check ACC 3D bitmap-GFP output against TT-only output exactly."""

from __future__ import annotations

import csv
import os
import shlex
import subprocess
import sys
from pathlib import Path


def run(cmd: list[str], cwd: Path) -> str:
    shell_cmd = " ".join(shlex.quote(x) for x in cmd)
    proc = subprocess.run(
        ["bash", "-lc", shell_cmd],
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    print(proc.stdout)
    if proc.returncode != 0:
        raise RuntimeError(f"command failed ({proc.returncode}): {shell_cmd}")
    return proc.stdout


def parse_tt_threshold(path: Path) -> dict[int, int]:
    final_iter = -1
    rows: list[tuple[int, int, int]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            iteration = int(row["iteration"])
            key = int(row["key_flat"])
            tau = int(row["tau"])
            rows.append((iteration, key, tau))
            final_iter = max(final_iter, iteration)
    return {key: tau for iteration, key, tau in rows if iteration == final_iter}


def parse_bitmap_threshold(path: Path) -> dict[int, int]:
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        return {int(row["key_flat"]): int(row["tau"]) for row in reader}


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    acc_dir = root / "examples" / "acc"
    kernel_pack = root / "kernel-pack"
    device = os.environ.get("PFACES_GPU_DEVICE", "1")

    for name in ("threshold_evolution.csv", "bitmap_threshold_final.csv", "iteration_stats.csv"):
        try:
            (acc_dir / name).unlink()
        except FileNotFoundError:
            pass

    base = [
        "pfaces",
        "-G",
        "-k",
        f"mono_synth.gpu@{kernel_pack}",
        "-cfg",
        "./acc.cfg",
        "-d",
        device,
        "-p",
        "-v1",
    ]

    print("[check] Running TT-only ACC...")
    run(
        base
        + [
            "-co",
            "use_tt_only=true,use_bitmap_gfp=false,benchmark_count=1,"
            "record_basis_evolution=true,use_prefix_sweep=false",
        ],
        cwd=acc_dir,
    )
    tt = parse_tt_threshold(acc_dir / "threshold_evolution.csv")

    print("[check] Running bitmap-GFP ACC...")
    run(
        base
        + [
            "-co",
            "use_tt_only=false,use_bitmap_gfp=true,benchmark_count=1,"
            "record_basis_evolution=true",
        ],
        cwd=acc_dir,
    )
    bitmap = parse_bitmap_threshold(acc_dir / "bitmap_threshold_final.csv")

    keys = set(tt) | set(bitmap)
    mismatches = [
        (k, tt.get(k, 0), bitmap.get(k, 0))
        for k in sorted(keys)
        if tt.get(k, 0) != bitmap.get(k, 0)
    ]

    tt_safe = sum(tt.values())
    bitmap_safe = sum(bitmap.values())
    print(f"[check] TT safe cells:     {tt_safe}")
    print(f"[check] Bitmap safe cells: {bitmap_safe}")
    print(f"[check] Compared columns:  {len(keys)}")
    print(f"[check] Mismatches:        {len(mismatches)}")

    if mismatches:
        print("[check] First mismatches:")
        for key, tt_tau, bm_tau in mismatches[:20]:
            print(f"  key={key}: tt={tt_tau}, bitmap={bm_tau}")
        return 1

    print("[check] PASS: bitmap GFP exactly matches TT-only on ACC 3D.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
