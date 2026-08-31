#!/usr/bin/env python3
"""Structural and dry-run checks for the seven-method paper matrix."""

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
        "cdc", "cdc_threshold_host", "cdc_threshold_gpu",
        "automatica_scan", "automatica_threshold", "threshold",
        "bitmap_reference",
    ]
    assert {item["key"]: item["scales"] for item in matrix.PAPER_EXPERIMENTS} == {
        "acc": (8, 9, 10, 12, 14),
        "acc_5d": (8, 9, 11),
        "turn_ego": (8, 9, 10, 12, 14),
        "turn_oncoming": (8, 9, 10, 12, 14),
    }
    assert matrix.RUN_POLICY["warmup_runs"] == 1
    assert matrix.RUN_POLICY["measured_runs"] == 1

    acc = matrix.PAPER_EXPERIMENTS[0]
    acc_1e8 = matrix.experiment_widths(acc, 8, 3)
    acc_1e10 = matrix.experiment_widths(acc, 10, 3)
    plans_1e8 = [matrix.plan_method(method, acc_1e8, 0) for method in matrix.PAPER_METHODS]
    plans_1e10 = [matrix.plan_method(method, acc_1e10, 0) for method in matrix.PAPER_METHODS]
    assert {plan["transition_backend"] for plan in plans_1e8} == {"precomputed"}
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
        assert len(rows) == 126
        assert len({(row["case"], row["run_method"]) for row in rows}) == 126
        for row in rows:
            widths = [int(value) for value in row["widths"].split("x")]
            assert widths
            assert matrix.product(widths) == int(row["actual_cells"])
            assert int(row["actual_cells"]) > 0
            assert row["transition_backend"] in {"precomputed", "inline", "skip"}
        markdown = (output / "full_matrix_table.md").read_text()
        latex = (output / "full_matrix_table.tex").read_text()
        assert markdown.count("| ACC |") == 35
        assert "CDC + threshold (GPU)" in markdown
        assert "\\begin{longtable}" in latex

    print("seven-method paper matrix dry-run passed (126 rows)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
