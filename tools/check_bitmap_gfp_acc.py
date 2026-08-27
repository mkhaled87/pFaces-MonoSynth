#!/usr/bin/env python3
"""Check the two explicit reference modes against canonical threshold bytes."""

from __future__ import annotations

import argparse
import hashlib
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--cfg", type=Path,
        default=Path(__file__).resolve().parents[1] / "examples" / "acc" / "acc.cfg",
    )
    parser.add_argument("--device", default=os.environ.get("PFACES_GPU_DEVICE", "1"))
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    cfg = args.cfg.resolve()
    output_dir = (args.output_dir or cfg.parent).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    kernel_pack = root / "kernel-pack"
    device = args.device

    project = "mono_synth_reference_check"
    output = output_dir / f"{project}.threshold.u32.bin"
    for name in (output.name,):
        try:
            (output_dir / name).unlink()
        except FileNotFoundError:
            pass

    base = [
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
    legacy_unset = ",".join(
        f"{key}=__mono_synth_legacy_unset__"
        for key in (
            "use_threshold_table", "use_tt_only", "use_tt_only_gpu",
            "use_bitmap_gfp", "use_inline_dynamics", "use_prefix_sweep",
            "boundary_seeding",
        )
    )

    print("[check] Running CPU threshold reference...")
    run(
        base
        + [
            "-co",
            f"project_name={project},synthesis_method=threshold_cpu_reference,"
            "transition_semantics=extremal_single_successor,"
            "transition_backend=precomputed,boundary_semantics=favorable_saturating,"
            "benchmark_count=1,save_controller=true,"
            f"{legacy_unset}",
        ],
        cwd=output_dir,
    )
    threshold_bytes = output.read_bytes()

    print("[check] Running bitmap-GFP ACC...")
    run(
        base
        + [
            "-co",
            f"project_name={project},synthesis_method=bitmap_reference,"
            "transition_semantics=extremal_single_successor,"
            "transition_backend=precomputed,boundary_semantics=favorable_saturating,"
            "benchmark_count=1,save_controller=true,"
            f"{legacy_unset}",
        ],
        cwd=output_dir,
    )
    bitmap_bytes = output.read_bytes()
    print(f"[check] threshold sha256: {hashlib.sha256(threshold_bytes).hexdigest()}")
    print(f"[check] bitmap sha256:    {hashlib.sha256(bitmap_bytes).hexdigest()}")
    if threshold_bytes != bitmap_bytes:
        print("[check] FAIL: canonical outputs differ")
        return 1
    print("[check] PASS: reference modes have identical canonical outputs.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
