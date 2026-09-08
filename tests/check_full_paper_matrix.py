#!/usr/bin/env python3
"""Structural and dry-run checks for the progressive scale matrix."""

from __future__ import annotations

import csv
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

import run_full_paper_matrix as matrix  # noqa: E402


def main() -> int:
    assert [method["key"] for method in matrix.PAPER_METHODS] == [
        "cdc_host", "cdc_gpu", "cdc_threshold_host", "cdc_threshold_gpu",
        "automatica_scan", "automatica_threshold", "threshold",
        "bitmap_reference",
    ]
    assert {item["key"]: item["scales"] for item in matrix.PAPER_EXPERIMENTS} == {
        "acc": (6, 7, 8, 9, 10, 12, 14),
        "acc_5d": (6, 7, 8, 9, 11),
        "turn_ego": (6, 7, 8, 9, 10, 12, 14),
        "turn_oncoming": (6, 7, 8, 9, 10, 12, 14),
    }
    assert matrix.RUN_POLICY["warmup_runs"] == 0
    assert matrix.RUN_POLICY["measured_runs"] == 1
    assert matrix.RUN_POLICY["max_advance_seconds"] == 60
    assert set(matrix.RUN_POLICY["timeouts_seconds"].values()) == {1200}
    assert matrix.RESOURCE_POLICY["keep_transition_caches"] is True

    acc = matrix.PAPER_EXPERIMENTS[0]
    acc_1e8 = matrix.experiment_widths(acc, 8, 3)
    acc_1e10 = matrix.experiment_widths(acc, 10, 3)
    plans_1e8 = [matrix.plan_method(method, acc_1e8, 0) for method in matrix.PAPER_METHODS]
    plans_1e10 = [matrix.plan_method(method, acc_1e10, 0) for method in matrix.PAPER_METHODS]
    backends_1e8 = {plan["run_method"]: plan["transition_backend"] for plan in plans_1e8}
    assert backends_1e8["threshold"] == "inline"
    assert backends_1e8["bitmap_reference"] == "inline"
    assert all(
        backend == "precomputed"
        for method, backend in backends_1e8.items()
        if method not in ("threshold", "bitmap_reference")
    )
    assert matrix.PAPER_METHODS[6] == {
        "key": "threshold", "label": "Threshold GFP (ours, inline)",
        "solver": "threshold", "transition": "inline",
    }
    assert matrix.PAPER_METHODS[7] == {
        "key": "bitmap_reference", "label": "Bitmap GFP",
        "solver": "bitmap_reference", "transition": "inline",
    }
    assert [plan["run_method"] for plan in plans_1e10 if plan["status"] == "planned"] == [
        "threshold", "bitmap_reference",
    ]
    assert all(plan["transition_backend"] == "inline" for plan in plans_1e10 if plan["status"] == "planned")

    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "matrix"
        subprocess.run(
            [sys.executable, str(TOOLS / "run_full_paper_matrix.py"),
             "--dry-run", "--output", str(output)],
            cwd=ROOT, check=True, stdout=subprocess.DEVNULL,
        )
        with (output / "run_plan.csv").open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 156
        assert len({(row["case"], row["run_method"]) for row in rows}) == 156
        assert sum(row["status"] == "planned" for row in rows) == 109
        assert sum(row["status"] == "skipped_resource" for row in rows) == 47
        for row in rows:
            widths = [int(value) for value in row["widths"].split("x")]
            assert widths
            assert matrix.product(widths) == int(row["actual_cells"])
            assert int(row["actual_cells"]) > 0
            assert row["transition_backend"] in {"precomputed", "inline", "skip"}
        markdown = (output / "full_matrix_table.md").read_text()
        latex = (output / "full_matrix_table.tex").read_text()
        assert markdown.count("| ACC |") == 42
        assert "| Reason |" in markdown
        assert "| Cache kernel/read |" in markdown
        assert "| Cache process wall |" in markdown
        assert "| Wall |" in markdown
        assert "CDC (host)" in markdown
        assert "CDC (GPU)" not in markdown
        assert "CDC + threshold (host)" in markdown
        assert "Automatica scan (GPU)" in markdown
        assert "Automatica + threshold (GPU)" in markdown
        assert "Threshold GFP (ours, inline)" in markdown
        assert "\\begin{longtable}" in latex

        custom_output = Path(directory) / "custom-scale"
        subprocess.run(
            [sys.executable, str(TOOLS / "run_full_paper_matrix.py"),
             "--dry-run", "--examples", "acc", "--N", "7",
             "--output", str(custom_output)],
            cwd=ROOT, check=True, stdout=subprocess.DEVNULL,
        )
        with (custom_output / "run_plan.csv").open(newline="") as stream:
            custom_rows = list(csv.DictReader(stream))
        assert [row["run_method"] for row in custom_rows] == [
            "cdc_host", "cdc_threshold_host", "automatica_scan",
            "automatica_threshold", "threshold", "bitmap_reference",
        ]
        assert {row["case"] for row in custom_rows} == {"acc_1e7"}
        assert {row["scale"] for row in custom_rows} == {"7"}
        assert len(custom_rows) == 6
        assert next(
            row for row in custom_rows if row["run_method"] == "threshold"
        )["transition_backend"] == "inline"

    quick = [{"case": "acc_1e6", "run_method": "threshold", "repetition": 1,
              "status": "ok", "wall_ms": 60000.0}]
    slow = [{"case": "acc_1e6", "run_method": "threshold", "repetition": 1,
             "status": "ok", "wall_ms": 60000.1}]
    timeout = [{"case": "acc_1e6", "run_method": "threshold", "repetition": 1,
                "status": "timeout", "wall_ms": 1200000.0}]
    assert matrix.progression_stop_reason("acc_1e6", "threshold", quick, 1, 60) is None
    assert "exceeded 60s" in matrix.progression_stop_reason(
        "acc_1e6", "threshold", slow, 1, 60
    )
    assert "timeout" in matrix.progression_stop_reason(
        "acc_1e6", "threshold", timeout, 1, 60
    )
    assert "missing measured result" in matrix.progression_stop_reason(
        "acc_1e6", "threshold", [], 1, 60
    )
    skipped = matrix.summarize([{
        "case": "acc_1e7", "run_method": "threshold",
        "status": "skipped_progression", "reason": "not advanced after acc_1e6",
        "transition_backend": "inline",
    }], [], {}, 1)[0]
    assert skipped["status"] == "skipped_progression"
    assert skipped["reason"] == "not advanced after acc_1e6"

    print("progressive paper matrix dry-run passed (156 rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
