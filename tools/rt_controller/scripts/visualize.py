#!/usr/bin/env python3
"""
visualize.py — Post-hoc visualization for MonoSafe real-time controller logs.

Reads a CSV log file produced by the rt_controller simulation and generates
a comprehensive set of plots:

  1. State trajectories over time (with safety coloring)
  2. Control inputs over time
  3. Phase-plane / state-space trajectory (2D projections)
  4. Timing performance (controller solve, safety queries)
  5. Oncoming velocity profile & resynthesis events
  6. Top-down intersection animation (optional, for turn scenarios)
  7. Summary statistics table

Usage:
    python visualize.py sim_log.csv                  # All plots
    python visualize.py sim_log.csv --save fig/      # Save to directory
    python visualize.py sim_log.csv --animate         # Generate MP4 animation
    python visualize.py sim_log.csv --state-labels s_ego,v_ego,s_onc
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Optional

import matplotlib
import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="MonoSafe real-time controller log visualization"
    )
    p.add_argument("log", type=str, help="Path to sim_log.csv")
    p.add_argument(
        "--save",
        type=str,
        default=None,
        help="Directory to save figures (PNG). If omitted, show interactively.",
    )
    p.add_argument(
        "--animate",
        action="store_true",
        help="Generate top-down intersection animation (MP4).",
    )
    p.add_argument(
        "--state-labels",
        type=str,
        default=None,
        help="Comma-separated state labels (e.g. s_ego,v_ego,s_onc). "
        "Auto-detected from CSV header if omitted.",
    )
    p.add_argument(
        "--input-labels",
        type=str,
        default=None,
        help="Comma-separated input labels (e.g. torque).",
    )
    p.add_argument(
        "--dpi", type=int, default=150, help="Figure resolution (default: 150)"
    )
    p.add_argument(
        "--style",
        type=str,
        default="seaborn-v0_8-whitegrid",
        help="Matplotlib style (default: seaborn-v0_8-whitegrid)",
    )
    p.add_argument(
        "--dark", action="store_true", help="Use dark background style."
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

# Default state / input label mappings for known scenarios
KNOWN_LABELS: dict[str, dict[str, list[str]]] = {
    "turn_ego_first": {
        "states": ["$s_{ego}$ (m)", "$v_{ego}$ (m/s)", "$s_{onc}$ (m)"],
        "inputs": ["Torque (N·m)"],
    },
    "turn_oncoming_first": {
        "states": ["$s_{ego}$ (m)", "$v_{ego}$ (m/s)", "$s_{onc}$ (m)"],
        "inputs": ["Torque (N·m)"],
    },
    "acc": {
        "states": ["Gap (m)", "$v_{ego}$ (m/s)", "$v_{lead}$ (m/s)"],
        "inputs": ["Accel ($m/s^2$)"],
    },
}


def load_log(path: str) -> pd.DataFrame:
    """Load CSV log and parse columns into typed arrays."""
    df = pd.read_csv(path)
    # Derive convenience columns
    df["is_safe_bool"] = df["is_safe"].astype(bool)
    df["ctrl_us"] = df["ctrl_ms"] * 1000  # µs
    df["query_us"] = df["query_ns"] / 1000  # µs
    # Mark resynthesis events (synth_ms > 0)
    df["resynth"] = df["synth_ms"] > 0
    # Measured parameter (0.0 if scenario has no runtime param)
    if "param_value" not in df.columns:
        df["param_value"] = 0.0
    # Safe fraction (may be absent)
    if "safe_frac" not in df.columns:
        df["safe_frac"] = np.nan
    # Optional safe s_ego slice (legacy/current animation highlight)
    if "safe_s_lo" not in df.columns:
        df["safe_s_lo"] = np.nan
    if "safe_s_hi" not in df.columns:
        df["safe_s_hi"] = np.nan
    return df


def detect_dims(df: pd.DataFrame) -> tuple[list[str], list[str]]:
    """Detect state (x*) and input (u*) column names from CSV header."""
    x_cols = sorted([c for c in df.columns if c.startswith("x")
                     and c[1:].isdigit()], key=lambda c: int(c[1:]))
    u_cols = sorted([c for c in df.columns if c.startswith("u")
                     and c[1:].isdigit()], key=lambda c: int(c[1:]))
    return x_cols, u_cols


def make_labels(x_cols: list[str], u_cols: list[str],
                state_labels: Optional[str], input_labels: Optional[str]
                ) -> tuple[list[str], list[str]]:
    """Build axis labels from user args or defaults."""
    if state_labels:
        s_lab = state_labels.split(",")
    else:
        s_lab = [f"$x_{{{i}}}$" for i in range(len(x_cols))]
    if input_labels:
        u_lab = input_labels.split(",")
    else:
        u_lab = [f"$u_{{{i}}}$" for i in range(len(u_cols))]
    return s_lab, u_lab


# ---------------------------------------------------------------------------
# Color scheme
# ---------------------------------------------------------------------------

COLORS = {
    "safe": "#2ecc71",       # green
    "unsafe": "#e74c3c",     # red
    "state": ["#3498db", "#e67e22", "#9b59b6", "#1abc9c",
              "#e84393", "#fdcb6e", "#00cec9", "#6c5ce7"],
    "control": ["#2d3436", "#d63031", "#00b894", "#0984e3"],
    "resynth": "#f39c12",    # orange
    "grid": "#bdc3c7",
    "accent": "#2980b9",
}


# ---------------------------------------------------------------------------
# Plot 1: State trajectories
# ---------------------------------------------------------------------------

def plot_states(df: pd.DataFrame, x_cols: list[str], s_lab: list[str],
                fig_dir: Optional[str], dpi: int) -> None:
    import matplotlib.pyplot as plt

    n = len(x_cols)
    fig, axes = plt.subplots(n, 1, figsize=(12, 2.5 * n), sharex=True)
    if n == 1:
        axes = [axes]

    t = df["time"].values

    for i, (col, ax) in enumerate(zip(x_cols, axes)):
        x = df[col].values
        c = COLORS["state"][i % len(COLORS["state"])]

        # Draw safe/unsafe background bands
        is_safe = df["is_safe_bool"].values
        _shade_safety(ax, t, is_safe)

        ax.plot(t, x, color=c, linewidth=1.8, zorder=5)
        ax.set_ylabel(s_lab[i] if i < len(s_lab) else col, fontsize=11)
        ax.tick_params(labelsize=9)

        # Mark resynthesis events
        resynth_t = df.loc[df["resynth"], "time"].values
        for rt in resynth_t:
            ax.axvline(rt, color=COLORS["resynth"], linestyle="--",
                       linewidth=0.8, alpha=0.7, zorder=3)

    axes[-1].set_xlabel("Time (s)", fontsize=11)
    axes[0].set_title("State Trajectories", fontsize=13, fontweight="bold")

    # Legend for first axis
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    legend_elems = [
        Patch(facecolor=COLORS["safe"], alpha=0.15, label="Safe"),
        Patch(facecolor=COLORS["unsafe"], alpha=0.15, label="Unsafe"),
        Line2D([0], [0], color=COLORS["resynth"], linestyle="--",
               linewidth=1, label="Resynthesis"),
    ]
    axes[0].legend(handles=legend_elems, loc="upper right", fontsize=8,
                   framealpha=0.8)

    fig.tight_layout()
    _save_or_show(fig, fig_dir, "states.png", dpi)


# ---------------------------------------------------------------------------
# Plot 2: Control inputs
# ---------------------------------------------------------------------------

def plot_controls(df: pd.DataFrame, u_cols: list[str], u_lab: list[str],
                  fig_dir: Optional[str], dpi: int) -> None:
    import matplotlib.pyplot as plt

    n = len(u_cols)
    fig, axes = plt.subplots(n, 1, figsize=(12, 2.5 * n), sharex=True)
    if n == 1:
        axes = [axes]

    t = df["time"].values

    for i, (col, ax) in enumerate(zip(u_cols, axes)):
        u = df[col].values
        c = COLORS["control"][i % len(COLORS["control"])]

        # Step-style plot (ZOH control)
        ax.step(t, u, where="post", color=c, linewidth=1.5, zorder=5)
        ax.set_ylabel(u_lab[i] if i < len(u_lab) else col, fontsize=11)
        ax.tick_params(labelsize=9)
        ax.axhline(0, color=COLORS["grid"], linewidth=0.5)

        _shade_safety(ax, t, df["is_safe_bool"].values)

    axes[-1].set_xlabel("Time (s)", fontsize=11)
    axes[0].set_title("Control Inputs", fontsize=13, fontweight="bold")

    fig.tight_layout()
    _save_or_show(fig, fig_dir, "controls.png", dpi)


# ---------------------------------------------------------------------------
# Plot 3: Phase-plane (2D projections)
# ---------------------------------------------------------------------------

def plot_phase(df: pd.DataFrame, x_cols: list[str], s_lab: list[str],
               fig_dir: Optional[str], dpi: int) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    n = len(x_cols)
    if n < 2:
        return  # Need at least 2 states

    # Generate all 2D projection pairs
    pairs = []
    for i in range(n):
        for j in range(i + 1, n):
            pairs.append((i, j))

    ncols = min(3, len(pairs))
    nrows = (len(pairs) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4.5 * nrows))
    if len(pairs) == 1:
        axes = np.array([axes])
    axes = np.atleast_2d(axes)

    is_safe = df["is_safe_bool"].values

    for idx, (i, j) in enumerate(pairs):
        ax = axes.flat[idx]
        xi = df[x_cols[i]].values
        xj = df[x_cols[j]].values

        # Create colored line segments based on safety
        points = np.column_stack([xi, xj]).reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        colors = [COLORS["safe"] if s else COLORS["unsafe"] for s in is_safe[:-1]]

        lc = LineCollection(segments, colors=colors, linewidths=1.5, zorder=5)
        ax.add_collection(lc)

        # Start/end markers
        ax.plot(xi[0], xj[0], "o", color=COLORS["accent"], markersize=8,
                zorder=10, label="Start")
        ax.plot(xi[-1], xj[-1], "s", color="#2d3436", markersize=8,
                zorder=10, label="End")

        ax.set_xlabel(s_lab[i] if i < len(s_lab) else x_cols[i], fontsize=10)
        ax.set_ylabel(s_lab[j] if j < len(s_lab) else x_cols[j], fontsize=10)
        ax.autoscale_view()
        ax.legend(fontsize=8)
        ax.set_title(f"Phase: {x_cols[i]} vs {x_cols[j]}", fontsize=10)

    # Hide unused axes
    for idx in range(len(pairs), axes.size):
        axes.flat[idx].set_visible(False)

    fig.suptitle("State-Space Trajectories", fontsize=13, fontweight="bold", y=1.01)
    fig.tight_layout()
    _save_or_show(fig, fig_dir, "phase.png", dpi)


# ---------------------------------------------------------------------------
# Plot 4: Timing performance
# ---------------------------------------------------------------------------

def plot_timing(df: pd.DataFrame, fig_dir: Optional[str], dpi: int) -> None:
    """
    Stacked-bar computation-time breakdown per timestep.
    Blue = controller solve time; orange = synthesis (basis) time.
    Background shading indicates safety status at each step.
    """
    import matplotlib.pyplot as plt

    t        = df["time"].values
    ctrl_ms  = df["ctrl_ms"].values
    synth_ms = df["synth_ms"].values
    is_safe  = df["is_safe_bool"].values
    n        = len(t)
    dt       = (t[1] - t[0]) if n > 1 else 0.5
    bar_w    = dt * 0.75

    fig, ax = plt.subplots(figsize=(14, 5), facecolor="white")

    # Safety background shading
    for k in range(n):
        ax.axvspan(t[k] - dt * 0.5, t[k] + dt * 0.5,
                   color=COLORS["safe"] if is_safe[k] else COLORS["unsafe"],
                   alpha=0.07, zorder=0)

    # Stacked bars: controller (bottom) + synthesis (top)
    ax.bar(t, ctrl_ms,  width=bar_w, color="#2980b9", alpha=0.88,
           label="Controller", zorder=5)
    ax.bar(t, synth_ms, width=bar_w, bottom=ctrl_ms,
           color="#e67e22", alpha=0.88, label="Synthesis (basis)", zorder=5)

    # Mean ctrl time line
    mean_ctrl = ctrl_ms.mean()
    ax.axhline(mean_ctrl, color="#2980b9", linestyle="--", linewidth=1.2,
               alpha=0.7, label=f"Ctrl mean: {mean_ctrl:.1f} ms", zorder=6)

    # Resynthesis step markers
    resynth_t = df.loc[df["resynth"], "time"].values
    for rt in resynth_t:
        ax.axvline(rt, color=COLORS["resynth"], linestyle=":",
                   linewidth=1.2, alpha=0.8, zorder=4)

    ax.set_xlabel("Time (s)", fontsize=11)
    ax.set_ylabel("Computation time (ms)", fontsize=11)
    ax.set_title("Per-Step Computation Time Breakdown",
                 fontsize=13, fontweight="bold")
    ax.set_xlim(t[0] - dt, t[-1] + dt)
    ax.set_ylim(0, (ctrl_ms + synth_ms).max() * 1.18)
    ax.legend(fontsize=10, framealpha=0.9)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", alpha=0.2)

    # Annotate safety legend patches
    from matplotlib.patches import Patch
    handles, labels = ax.get_legend_handles_labels()
    handles += [Patch(fc=COLORS["safe"],   alpha=0.25, label="Safe step"),
                Patch(fc=COLORS["unsafe"], alpha=0.25, label="Unsafe step")]
    ax.legend(handles=handles, fontsize=9, framealpha=0.9)

    fig.tight_layout()
    _save_or_show(fig, fig_dir, "timing.png", dpi)


# ---------------------------------------------------------------------------
# Plot 5: Oncoming velocity & safety
# ---------------------------------------------------------------------------

def plot_param_safety(df: pd.DataFrame, fig_dir: Optional[str], dpi: int) -> None:
    """
    Three-subplot panel:
      (a) Measured parameter + ego velocity.
      (b) Safety certification status per step.
      (c) Safe-set basis size over time.
    """
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True,
                             facecolor="white")
    t = df["time"].values
    dt = (t[1] - t[0]) if len(t) > 1 else 0.5
    is_safe = df["is_safe_bool"].values
    resynth_t = df.loc[df["resynth"], "time"].values

    def _shade(ax):
        for k in range(len(t)):
            ax.axvspan(t[k] - dt * 0.5, t[k] + dt * 0.5,
                       color=COLORS["safe"] if is_safe[k] else COLORS["unsafe"],
                       alpha=0.06, zorder=0)

    # (a) Measured parameter + ego velocity
    ax = axes[0]
    param_vals = df["param_value"].values
    has_param = not np.allclose(param_vals, 0.0)
    if has_param:
        ax.plot(t, param_vals, color="#e67e22", linewidth=2.0,
                label="Param", zorder=5)
    if "x1" in df.columns:
        ax.plot(t, df["x1"].values, color="#2c3e50", linewidth=2.0,
                linestyle="--", label="$v_{ego}$ / $x_1$", zorder=5)
    for rt in resynth_t:
        ax.axvline(rt, color=COLORS["resynth"], linestyle=":",
                   linewidth=1.0, alpha=0.7, zorder=3)
    _shade(ax)
    ax.set_ylabel("Value", fontsize=11)
    ax.set_title("Parameter & State", fontsize=12, fontweight="bold")
    ax.legend(fontsize=10, framealpha=0.9)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # (b) Safety status
    ax = axes[1]
    is_f = is_safe.astype(float)
    ax.fill_between(t, 0, 1, where=is_safe,  color=COLORS["safe"],
                    alpha=0.30, step="post")
    ax.fill_between(t, 0, 1, where=~is_safe, color=COLORS["unsafe"],
                    alpha=0.30, step="post")
    ax.step(t, is_f, where="post", color="#2d3436", linewidth=1.5, zorder=5)
    ax.set_ylabel("Certified", fontsize=11)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["No", "Yes"])
    ax.set_ylim(-0.1, 1.1)
    ax.set_title("Safety Certification", fontsize=12, fontweight="bold")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    # (c) Safe set: basis size (left axis) + safe fraction % (right axis)
    ax = axes[2]
    ax.fill_between(t, 0, df["basis_size"].values, step="post",
                    color=COLORS["safe"], alpha=0.12)
    ln1 = ax.step(t, df["basis_size"].values, where="post",
                  color=COLORS["safe"], linewidth=1.8, zorder=5,
                  label="Basis |B|")
    _shade(ax)
    ax.set_ylabel("Basis Size", fontsize=11)
    ax.set_xlabel("Time (s)", fontsize=11)
    ax.spines["top"].set_visible(False)

    # Right axis: safe fraction (%)
    if "safe_frac" in df.columns and not df["safe_frac"].isna().all():
        ax2 = ax.twinx()
        safe_pct = df["safe_frac"].values * 100.0
        ln2 = ax2.step(t, safe_pct, where="post",
                       color="#8E44AD", linewidth=1.8, linestyle="--",
                       zorder=5, label="Safe fraction %")
        ax2.set_ylabel("Safe Fraction (%)", fontsize=11, color="#8E44AD")
        ax2.tick_params(axis="y", labelcolor="#8E44AD")
        ax2.spines["top"].set_visible(False)
        # Combined legend
        lines = ln1 + ln2
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, fontsize=9, loc="upper right", framealpha=0.9)
    ax.set_title("Safe Set Size", fontsize=12, fontweight="bold")

    fig.tight_layout()
    _save_or_show(fig, fig_dir, "param_safety.png", dpi)


# ---------------------------------------------------------------------------
# Plot 6: Summary statistics
# ---------------------------------------------------------------------------

def plot_summary(df: pd.DataFrame, fig_dir: Optional[str], dpi: int) -> None:
    import matplotlib.pyplot as plt

    total = len(df)
    safe_count = df["is_safe_bool"].sum()
    resynth_count = df["resynth"].sum()

    stats = [
        ("Total steps", f"{total}"),
        ("Safe steps", f"{safe_count}/{total} ({100*safe_count/max(1,total):.1f}%)"),
        ("Resynthesis events", f"{resynth_count}"),
        ("Duration", f"{df['time'].iloc[-1]:.1f} s"),
        ("", ""),
        ("Ctrl time (mean)", f"{df['ctrl_ms'].mean():.2f} ms"),
        ("Ctrl time (max)", f"{df['ctrl_ms'].max():.2f} ms"),
        ("Ctrl time (P95)", f"{np.percentile(df['ctrl_ms'], 95):.2f} ms"),
        ("Safety query (mean)", f"{df['query_ns'].mean()/1000:.1f} µs"),
        ("Safety query (max)", f"{df['query_ns'].max()/1000:.1f} µs"),
        ("", ""),
        ("Basis size (final)", f"{df['basis_size'].iloc[-1]}"),
    ]

    if resynth_count > 0:
        synth_times = df.loc[df["resynth"], "synth_ms"]
        stats.append(("Synth time (mean)", f"{synth_times.mean():.1f} ms"))
        stats.append(("Synth time (max)", f"{synth_times.max():.1f} ms"))

    fig, ax = plt.subplots(figsize=(6, 0.4 * len(stats) + 1))
    ax.axis("off")

    table = ax.table(
        cellText=[[k, v] for k, v in stats],
        colLabels=["Metric", "Value"],
        colWidths=[0.45, 0.45],
        loc="center",
        cellLoc="left",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 1.5)

    # Style header
    for j in range(2):
        cell = table[0, j]
        cell.set_facecolor(COLORS["accent"])
        cell.set_text_props(color="white", fontweight="bold")

    # Alternate row colors
    for i in range(1, len(stats) + 1):
        for j in range(2):
            cell = table[i, j]
            if stats[i - 1][0] == "":
                cell.set_height(0.02)
                cell.set_facecolor("white")
                cell.set_edgecolor("white")
            elif i % 2 == 0:
                cell.set_facecolor("#f0f3f4")

    fig.suptitle("Simulation Summary", fontsize=14, fontweight="bold")
    fig.tight_layout()
    _save_or_show(fig, fig_dir, "summary.png", dpi)


# ---------------------------------------------------------------------------
# Plot 7: Top-down intersection animation (optional)
# ---------------------------------------------------------------------------

def animate_intersection(df: pd.DataFrame, x_cols: list[str],
                         fig_dir: Optional[str], dpi: int) -> None:
    """
    Publication-quality top-down left-turn intersection animation.

    Road scene (left panel)
    -----------------------
    Schematic dual-carriageway (E–W) with a northbound exit:
      • Ego vehicle  : lower lane (y = −lane_off), approaching from the
                       WEST (left); turns left (north) at the intersection.
      • Oncoming veh.: upper lane (y = +lane_off), approaching from the
                       EAST (right) — i.e. head-on toward the ego.
    Position mapping in screen coords:
      x_ego  = s_ego      (−38 → 0 → +10 in metres)
      x_onc  = −s_onc     (s_onc goes −65 → −10, so x_onc goes +65 → +10)
    Both vehicles converge on the intersection at x ≈ 0.

    Time-series panel (right)
    -------------------------
    Three stacked subplots with safety-shaded backgrounds:
      ① Both velocities (v_ego, v_oncoming) on the same axis
      ② Safe-set basis size
      ③ Control input

    Assumes 3D state vector: [s_ego, v_ego, s_onc].
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, FFMpegWriter
    from matplotlib.patches import Rectangle
    from matplotlib.gridspec import GridSpec
    import matplotlib.transforms as _mtf

    if len(x_cols) < 3:
        print("[animate] Skipping: need >= 3 state dims.")
        return

    # ── data ────────────────────────────────────────────────────────
    s_ego    = df[x_cols[0]].values
    v_ego    = df[x_cols[1]].values
    s_onc    = df[x_cols[2]].values
    u_ctrl   = df["u0"].values
    v_onc    = df["param_value"].values
    is_safe  = df["is_safe_bool"].values
    times    = df["time"].values
    basis    = df["basis_size"].values
    ctrl_ms  = df["ctrl_ms"].values
    synth_ms = df["synth_ms"].values
    N        = len(times)
    dt       = times[1] - times[0] if N > 1 else 0.1
    safe_frac = df["safe_frac"].values if "safe_frac" in df.columns else np.full(N, np.nan)
    safe_s_lo = df["safe_s_lo"].values if "safe_s_lo" in df.columns else np.full(N, np.nan)
    safe_s_hi = df["safe_s_hi"].values if "safe_s_hi" in df.columns else np.full(N, np.nan)
    finite_lo = safe_s_lo[np.isfinite(safe_s_lo)]
    finite_hi = safe_s_hi[np.isfinite(safe_s_hi)]

    # ── road geometry (physical metres) ───────────────────────────
    hw         = 4.0       # half-width: 8 m total road (2×2-lane)
    lane_off   = hw / 2    # lane-centre offset = 2 m
    R          = hw + lane_off   # = 2 m — tight turn into near northbound lane
    arc_sa0    = -hw             # s where arc begins  (= −4)
    arc_sa1    = arc_sa0 + R * np.pi / 2   # arc end   (≈ −0.86)
    arc_cx     = -hw             # arc centre x        (= −4)
    arc_cy     = -lane_off + R   # arc centre y        (= 0)
    arc_x_exit = arc_cx + R      # = −lane_off = −2
    arc_y_exit = arc_cy          # = 0

    def _ego_xy(s):
        """s (1-D) → (x, y) physical coords."""
        if s < arc_sa0:
            return np.array([s, -lane_off])
        if s > arc_sa1:
            return np.array([arc_x_exit, arc_y_exit + (s - arc_sa1)])
        frac  = (s - arc_sa0) / (R * np.pi / 2)
        theta = -np.pi / 2 + frac * np.pi / 2
        return np.array([arc_cx + R * np.cos(theta),
                         arc_cy + R * np.sin(theta)])

    def _ego_heading(s):
        if s < arc_sa0:   return 0.0
        if s > arc_sa1:   return 90.0
        return (s - arc_sa0) / (R * np.pi / 2) * 90.0

    def _onc_xy(s):
        """Oncoming: upper lane (y = +lane_off), approaches from +x."""
        return np.array([-s, lane_off])

    def _ego_segment(s_lo, s_hi, n=160):
        if np.isnan(s_lo) or np.isnan(s_hi):
            return np.empty((0, 2))
        if s_hi < s_lo:
            s_lo, s_hi = s_hi, s_lo
        ss = np.linspace(s_lo, s_hi, n)
        return np.array([_ego_xy(s) for s in ss])

    def _draw_boundary_tick(ax_, s, color, lw=2.0, tick_len=1.6):
        eps = 0.15
        p0 = _ego_xy(s)
        pm = _ego_xy(s - eps)
        pp = _ego_xy(s + eps)
        tangent = pp - pm
        norm = np.linalg.norm(tangent)
        if norm < 1e-9:
            tangent = np.array([1.0, 0.0])
            norm = 1.0
        tangent = tangent / norm
        normal = np.array([-tangent[1], tangent[0]])
        a = p0 - 0.5 * tick_len * normal
        b = p0 + 0.5 * tick_len * normal
        ax_.plot([a[0], b[0]], [a[1], b[1]],
                 color=color, lw=lw, solid_capstyle="round", zorder=8.2)
        ax_.plot(p0[0], p0[1], marker="o", ms=4.5,
                 mec="white", mew=0.9, mfc=color, zorder=8.3)

    ego_pts  = np.array([_ego_xy(s)      for s in s_ego])
    onc_pts  = np.array([_onc_xy(s)      for s in s_onc])
    ego_hdgs = np.array([_ego_heading(s) for s in s_ego])
    guide    = np.array([_ego_xy(s)      for s in np.linspace(-42, 14, 600)])
    path_s_min = min(float(np.nanmin(finite_lo)) if finite_lo.size else float(np.min(s_ego)),
                     float(np.min(s_ego)), -50.0)
    path_s_max = max(float(np.nanmax(finite_hi)) if finite_hi.size else float(np.max(s_ego)),
                     float(np.max(s_ego)), 10.0)

    # ── fixed limits for time-series subplots ─────────────────────
    tp   = dt * 0.5
    TL   = (times[0] - tp, times[-1] + tp)
    VVL  = (min(v_ego.min(), v_onc.min()) - 0.5,
            max(v_ego.max(), v_onc.max()) + 0.5)
    BL   = (basis.min() * 0.82,  basis.max() * 1.18)
    safe_pct = safe_frac * 100.0
    safe_pct_valid = safe_pct[~np.isnan(safe_pct)]
    SFL  = (0, safe_pct_valid.max() * 1.3) if len(safe_pct_valid) > 0 else (0, 100)
    CL   = (u_ctrl.min() - 200,  u_ctrl.max() + 200)
    # Timing: stacked bars ctrl_ms + synth_ms
    time_max = (ctrl_ms + synth_ms).max() * 1.20
    TML  = (0, time_max)

    # ── colour palette ─────────────────────────────────────────────
    C = {
        "road":  "#D5D8DC",
        "edge":  "#717D7E",
        "isect": "#E8EAEB",
        "dng":   "#E74C3C",
        "ln":    "white",
        "safe":  "#1E8449",
        "unsafe":"#C0392B",
        "onc":   "#E67E22",
        "ctrl":  "#2980B9",
        "synth": "#E67E22",
        "vego":  "#2C3E50",
        "guide": "#AEB6BF",
        "txt":   "#1C2833",
    }

    # ── rectangular car helper ─────────────────────────────────────
    # Physical size: 4.5 m long × 2.0 m wide (fits in 4 m half-lane)
    CAR_L, CAR_W = 4.5, 2.0

    def _draw_car(ax_, pos, hdg_deg, fc, label):
        """Draw a rectangular car centred at pos, rotated by hdg_deg."""
        rect = Rectangle((-CAR_L / 2, -CAR_W / 2), CAR_L, CAR_W,
                         fc=fc, ec="white", lw=1.6, zorder=10)
        tf = (_mtf.Affine2D()
                  .rotate_deg(hdg_deg)
                  .translate(pos[0], pos[1])
              + ax_.transData)
        rect.set_transform(tf)
        ax_.add_patch(rect)
        # windscreen stripe
        fa  = np.radians(hdg_deg)
        wx  = pos[0] + (CAR_L / 2 - 0.5) * np.cos(fa)
        wy  = pos[1] + (CAR_L / 2 - 0.5) * np.sin(fa)
        px, py = -np.sin(fa), np.cos(fa)
        ax_.plot([wx - px * CAR_W * 0.38, wx + px * CAR_W * 0.38],
                 [wy - py * CAR_W * 0.38, wy + py * CAR_W * 0.38],
                 c="white", lw=1.8, zorder=11)
        ax_.text(pos[0], pos[1], label, ha="center", va="center",
                 fontsize=6, fontweight="bold", color="white", zorder=12)

    # ── figure + gridspec (road left, 2×2 right) ──────────────────
    # Right panel: 2 columns × 2 rows → 4 subplots
    # Overall: 3-column gridspec: road | sp_col_a | sp_col_b
    fig = plt.figure(figsize=(18, 9), facecolor="white")
    gs  = GridSpec(2, 3, figure=fig, width_ratios=[2.0, 1, 1],
                   hspace=0.42, wspace=0.30,
                   left=0.03, right=0.97, top=0.92, bottom=0.08)
    a_rd = fig.add_subplot(gs[:, 0])          # road (spans both rows)
    a_vv = fig.add_subplot(gs[0, 1])          # velocities  (top-left)
    a_ct = fig.add_subplot(gs[0, 2])          # control     (top-right)
    a_bs = fig.add_subplot(gs[1, 1])          # basis size  (bottom-left)
    a_tm = fig.add_subplot(gs[1, 2])          # timing      (bottom-right)

    # ── helper: safety background spans ───────────────────────────
    def _spans(ax_, idx_):
        for k in range(idx_ + 1):
            t0 = times[k]
            t1 = times[k + 1] if k + 1 < N else t0 + dt
            ax_.axvspan(t0, t1,
                        color=C["safe"] if is_safe[k] else C["unsafe"],
                        alpha=0.07, zorder=0)

    def _style(ax_, ylabel, title, xlabel=None):
        ax_.tick_params(labelsize=8)
        ax_.grid(axis="y", alpha=0.15)
        ax_.set_ylabel(ylabel, fontsize=9)
        ax_.set_title(title, fontsize=10, fontweight="bold", pad=3)
        for sp in ("top", "right"):
            ax_.spines[sp].set_visible(False)
        if xlabel:
            ax_.set_xlabel(xlabel, fontsize=9)
        else:
            ax_.tick_params(labelbottom=False)

    # ── per-frame draw ─────────────────────────────────────────────
    def _draw(idx):
        # ══ LEFT: ROAD SCENE (equal-aspect tracking camera) ══════
        a_rd.clear()
        a_rd.set_facecolor("white")
        a_rd.axis("off")

        # Tracking viewport: show both vehicles with margin, equal aspect
        ex, ey = ego_pts[idx]
        ox, oy = onc_pts[idx]
        cx  = (ex + ox) / 2
        # Vertical: centre on road midpoint, include north exit lane
        cy  = (max(ey, oy) + min(-hw, -hw)) / 2 + 2.0
        # Span to encompass both vehicles + margin
        span_x = abs(ex - ox) + 8.0
        span_y = max(span_x, 16.0)   # at least 24 m tall
        # Enforce panel aspect ratio (≈ 2/3 of 18"=12" wide, full 9" tall)
        panel_w, panel_h = 12.0, 7.5
        target_ar = panel_w / panel_h   # ~1.6
        if span_x / span_y < target_ar:
            span_x = span_y * target_ar
        else:
            span_y = span_x / target_ar
        a_rd.set_xlim(cx - span_x / 2, cx + span_x / 2)
        a_rd.set_ylim(cy - span_y / 2, cy + span_y / 2)
        a_rd.set_aspect("equal", adjustable="box")

        xl = a_rd.get_xlim()
        yl = a_rd.get_ylim()

        # E–W road surface
        a_rd.fill_between([xl[0], xl[1]], -hw, hw,
                          color=C["road"], zorder=1)
        # North exit road surface
        a_rd.fill_betweenx([hw, yl[1]], -hw, hw,
                           color=C["road"], zorder=1)
        # Intersection box
        a_rd.add_patch(Rectangle((-hw, -hw), hw * 2, hw * 2,
                                  fc=C["isect"], zorder=2))
        # Danger-zone tint
        a_rd.add_patch(Rectangle((-hw, -hw), hw * 2, hw * 2,
                                  fc=C["dng"], alpha=0.10, lw=0, zorder=3))

        # Kerb lines
        for ye in (-hw, hw):
            a_rd.plot([xl[0], -hw], [ye, ye], c=C["edge"], lw=1.2, zorder=4)
            a_rd.plot([hw, xl[1]], [ye, ye],  c=C["edge"], lw=1.2, zorder=4)
        for xe in (-hw, hw):
            a_rd.plot([xe, xe], [hw, yl[1]], c=C["edge"], lw=1.2, zorder=4)

        # Centre-line dashes
        dash_step = max(2, int(span_x / 30)) * 2
        for xs in np.arange(xl[0], xl[1], dash_step):
            a_rd.plot([xs, xs + dash_step * 0.45], [0, 0],
                      c=C["ln"], lw=0.8, alpha=0.45, zorder=4)
        for ys in np.arange(hw, yl[1], dash_step):
            a_rd.plot([0, 0], [ys, ys + dash_step * 0.45],
                      c=C["ln"], lw=0.8, alpha=0.45, zorder=4)

        # Lane separator (between ego and oncoming lanes)
        a_rd.plot([xl[0], -hw], [0, 0],
                  c=C["edge"], lw=0.7, alpha=0.4, ls="--", zorder=4)
        a_rd.plot([hw, xl[1]], [0, 0],
                  c=C["edge"], lw=0.7, alpha=0.4, ls="--", zorder=4)

        # Conflict-zone label
        a_rd.text(0, 0, "CONFLICT\nZONE", ha="center", va="center",
                  fontsize=6.5, color=C["dng"], alpha=0.5,
                  fontweight="bold", zorder=4)

        # Ego path guide
        g_vis = guide[(guide[:, 0] >= xl[0]) & (guide[:, 0] <= xl[1]) &
                      (guide[:, 1] >= yl[0]) & (guide[:, 1] <= yl[1])]
        if len(g_vis) > 1:
            a_rd.plot(g_vis[:, 0], g_vis[:, 1],
                      c=C["guide"], lw=1.3, ls="--", alpha=0.50, zorder=5)

        # Certified 1-D slice along s_ego for the current (v_ego, s_onc)
        # Unsafe region is shown in red, safe region in green, with explicit
        # boundary markers. This makes slice collapses / jumps visually clear.
        lo_i = safe_s_lo[idx]
        hi_i = safe_s_hi[idx]
        unsafe_left = _ego_segment(path_s_min, lo_i, n=180) if np.isfinite(lo_i) and lo_i > path_s_min else np.empty((0, 2))
        unsafe_right = _ego_segment(hi_i, path_s_max, n=180) if np.isfinite(hi_i) and hi_i < path_s_max else np.empty((0, 2))
        safe_seg = _ego_segment(lo_i, hi_i, n=220)

        def _clip_vis(seg):
            if len(seg) == 0:
                return seg
            return seg[(seg[:, 0] >= xl[0] - 2) & (seg[:, 0] <= xl[1] + 2) &
                       (seg[:, 1] >= yl[0] - 2) & (seg[:, 1] <= yl[1] + 2)]

        for unsafe_seg in (_clip_vis(unsafe_left), _clip_vis(unsafe_right)):
            if len(unsafe_seg) > 1:
                a_rd.plot(unsafe_seg[:, 0], unsafe_seg[:, 1],
                          c=C["unsafe"], lw=9.0, alpha=0.14,
                          solid_capstyle="round", zorder=5.15)
                a_rd.plot(unsafe_seg[:, 0], unsafe_seg[:, 1],
                          c=C["unsafe"], lw=3.2, alpha=0.62,
                          solid_capstyle="round", zorder=5.2)

        safe_vis = _clip_vis(safe_seg)
        if len(safe_vis) > 1:
            a_rd.plot(safe_vis[:, 0], safe_vis[:, 1],
                      c=C["safe"], lw=10.0, alpha=0.16,
                      solid_capstyle="round", zorder=5.25)
            a_rd.plot(safe_vis[:, 0], safe_vis[:, 1],
                      c=C["safe"], lw=4.0, alpha=0.65,
                      solid_capstyle="round", zorder=5.3)

        if np.isfinite(lo_i):
            _draw_boundary_tick(a_rd, lo_i, C["safe"] if np.isfinite(hi_i) else C["unsafe"])
        if np.isfinite(hi_i) and (not np.isfinite(lo_i) or abs(hi_i - lo_i) > 1e-9):
            _draw_boundary_tick(a_rd, hi_i, C["safe"])

        # if np.isfinite(lo_i) and np.isfinite(hi_i):
        #     if abs(hi_i - lo_i) < 1e-9:
        #         slice_txt = f"safe $s_{{ego}}$: singleton at {lo_i:.1f} m"
        #     else:
        #         slice_txt = f"safe $s_{{ego}}$: [{lo_i:.1f}, {hi_i:.1f}] m"
        #     a_rd.text(0.02, 0.82, slice_txt,
        #               transform=a_rd.transAxes, fontsize=8, va="top",
        #               color=C["txt"], zorder=20,
        #               bbox=dict(boxstyle="round,pad=0.25", fc="white",
        #                         ec=C["safe"], alpha=0.92, lw=1.2))
        #     a_rd.text(0.02, 0.765, "green = certified safe slice, red = unsafe slice",
        #               transform=a_rd.transAxes, fontsize=7.5, va="top",
        #               color=C["txt"], zorder=20,
        #               bbox=dict(boxstyle="round,pad=0.22", fc="white",
        #                         ec=C["unsafe"], alpha=0.88, lw=1.0))

        # Ego trail
        for k in range(idx):
            tc = C["safe"] if is_safe[k] else C["unsafe"]
            α  = 0.25 + 0.65 * ((k + 1) / max(idx, 1))
            a_rd.plot(ego_pts[k:k+2, 0], ego_pts[k:k+2, 1],
                      c=tc, lw=3.0, alpha=α,
                      solid_capstyle="round", zorder=6)
        # Oncoming trail
        for k in range(idx):
            α = 0.20 + 0.50 * ((k + 1) / max(idx, 1))
            a_rd.plot(onc_pts[k:k+2, 0], onc_pts[k:k+2, 1],
                      c=C["onc"], lw=2.5, alpha=α,
                      solid_capstyle="round", zorder=6)

        # Rectangular cars
        ego_col = C["safe"] if is_safe[idx] else C["unsafe"]
        _draw_car(a_rd, ego_pts[idx], ego_hdgs[idx], ego_col, "EGO")
        _draw_car(a_rd, onc_pts[idx], 180.0,         C["onc"],  "ONC")

        # Road direction arrows (relative to current viewport)
        for ax_x in np.arange(xl[0] + 8, ex - 10, 18):
            a_rd.annotate("", xy=(ax_x + 5, -lane_off),
                          xytext=(ax_x, -lane_off),
                          arrowprops=dict(arrowstyle="-|>", color="white",
                                         lw=0.7, alpha=0.3), zorder=4)
        for ax_x in np.arange(ox + 10, xl[1] - 8, 18):
            a_rd.annotate("", xy=(ax_x - 5, lane_off),
                          xytext=(ax_x, lane_off),
                          arrowprops=dict(arrowstyle="-|>", color="white",
                                         lw=0.7, alpha=0.3), zorder=4)

        # Timestamp + safety badge
        a_rd.text(0.02, 0.98, f"$t = {times[idx]:.1f}\\,$s",
                  transform=a_rd.transAxes, fontsize=13, va="top",
                  fontweight="bold", color=C["txt"], zorder=20)
        slab = "CERTIFIED SAFE" if is_safe[idx] else "UNCERTIFIED"
        scol = C["safe"] if is_safe[idx] else C["unsafe"]
        a_rd.text(0.02, 0.89, slab, transform=a_rd.transAxes, fontsize=9,
                  va="top", fontweight="bold", color=scol,
                  bbox=dict(boxstyle="round,pad=0.3", fc="white",
                            ec=scol, alpha=0.9, lw=1.5), zorder=20)
        # State readout
        a_rd.text(0.98, 0.98,
                  f"$s_{{ego}}={s_ego[idx]:.1f}$ m\n"
                  f"$v_{{ego}}={v_ego[idx]:.2f}$ m/s\n"
                  f"$s_{{onc}}={s_onc[idx]:.1f}$ m",
                  transform=a_rd.transAxes, fontsize=8, va="top", ha="right",
                  family="monospace", color=C["txt"],
                  bbox=dict(boxstyle="round,pad=0.3", fc="white",
                            alpha=0.85, ec=C["guide"]), zorder=20)

        # ══ RIGHT 2×2: TIME-SERIES PANELS ═════════════════════════
        sl = slice(0, idx + 1)

        # ① Velocities (top-left)
        a_vv.clear()
        a_vv.set_xlim(TL); a_vv.set_ylim(VVL)
        a_vv.plot(times[sl], v_onc[sl], "-o", c=C["onc"],
                  lw=2.0, ms=3.0, label="$v_{onc}$", zorder=5)
        a_vv.plot(times[sl], v_ego[sl], "--s", c=C["vego"],
                  lw=2.0, ms=3.0, label="$v_{ego}$", zorder=5)
        a_vv.legend(fontsize=8, loc="lower right", framealpha=0.85)
        _style(a_vv, "Velocity (m/s)", "Vehicle Velocities")
        _spans(a_vv, idx)

        # ② Control input (top-right)
        a_ct.clear()
        a_ct.set_xlim(TL); a_ct.set_ylim(CL)
        a_ct.axhline(0, c=C["guide"], lw=0.8, zorder=0)
        a_ct.step(times[sl], u_ctrl[sl], where="post",
                  c=C["ctrl"], lw=2.0, zorder=5)
        a_ct.plot(times[idx], u_ctrl[idx], "o", c=C["ctrl"], ms=5, zorder=6)
        _style(a_ct, "$u$ (N)", "Control Input")
        _spans(a_ct, idx)

        # ③ Safe-set fraction (bottom-left)
        a_bs.clear()
        a_bs.set_xlim(TL); a_bs.set_ylim(SFL)
        a_bs.fill_between(times[sl], 0, safe_pct[sl],
                          color=C["safe"], alpha=0.12, step="post")
        a_bs.step(times[sl], safe_pct[sl], where="post",
                  c=C["safe"], lw=2.0, zorder=5)
        a_bs.plot(times[idx], safe_pct[idx], "o", c=C["safe"], ms=5, zorder=6)
        _style(a_bs, "Safe (%)", "Safe Fraction of State Space", xlabel="Time (s)")
        _spans(a_bs, idx)

        # ④ Computation time stacked barh (bottom-right)
        a_tm.clear()
        a_tm.set_xlim(TL); a_tm.set_ylim(TML)
        bw = dt * 0.72
        a_tm.bar(times[sl], ctrl_ms[sl], width=bw,
                 color=C["ctrl"], alpha=0.85, label="Controller", zorder=5)
        a_tm.bar(times[sl], synth_ms[sl], width=bw, bottom=ctrl_ms[sl],
                 color=C["synth"], alpha=0.85, label="Synthesis", zorder=5)
        a_tm.axhline(ctrl_ms.mean(), c=C["ctrl"], lw=1.0, ls="--",
                     alpha=0.6, zorder=4)
        a_tm.legend(fontsize=7, loc="upper right", framealpha=0.85)
        _style(a_tm, "Time (ms)", "Computation Time", xlabel="Time (s)")
        _spans(a_tm, idx)

        fig.suptitle("MonoSafe  ·  Left-Turn Intersection  (dt = 0.1 s)",
                     fontsize=13, fontweight="bold",
                     color=C["txt"], y=0.97)
        return []

    anim = FuncAnimation(fig, _draw, frames=N, interval=100, blit=False)

    if fig_dir:
        mp4_path = os.path.join(fig_dir, "intersection.mp4")
        try:
            writer = FFMpegWriter(fps=10, metadata={"title": "MonoSafe Sim"})
            anim.save(mp4_path, writer=writer, dpi=dpi)
            print(f"[animate] Saved: {mp4_path}")
        except Exception as e:
            gif_path = os.path.join(fig_dir, "intersection.gif")
            try:
                anim.save(gif_path, writer="pillow", fps=10, dpi=dpi // 2)
                print(f"[animate] Saved: {gif_path} (ffmpeg unavailable)")
            except Exception as e2:
                print(f"[animate] Could not save animation: {e2}")
    else:
        plt.show()

    plt.close(fig)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _shade_safety(ax, t: np.ndarray, is_safe: np.ndarray) -> None:
    """Add safe/unsafe background shading to an axis."""
    ylims = ax.get_ylim()
    # We need to re-apply after autoscale; use callback instead
    ax.fill_between(t,1,-1,
                    where=is_safe, color=COLORS["safe"],
                    alpha=0.08, step="post", zorder=0)
    ax.fill_between(t,1,-1,
                    where=~is_safe, color=COLORS["unsafe"],
                    alpha=0.08, step="post", zorder=0)


def _save_or_show(fig, fig_dir: Optional[str], name: str, dpi: int) -> None:
    import matplotlib.pyplot as plt

    if fig_dir:
        os.makedirs(fig_dir, exist_ok=True)
        path = os.path.join(fig_dir, name)
        fig.savefig(path, dpi=dpi, bbox_inches="tight", facecolor=fig.get_facecolor())
        print(f"  Saved: {path}")
    else:
        plt.show()
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = parse_args()

    # Apply style
    import matplotlib.pyplot as plt
    try:
        if args.dark:
            plt.style.use("dark_background")
        else:
            plt.style.use(args.style)
    except OSError:
        plt.style.use("ggplot")  # Fallback

    # Load data
    print(f"Loading: {args.log}")
    df = load_log(args.log)
    x_cols, u_cols = detect_dims(df)
    s_lab, u_lab = make_labels(x_cols, u_cols, args.state_labels, args.input_labels)

    print(f"  States: {len(x_cols)} dims  |  Inputs: {len(u_cols)} dims  |  "
          f"Steps: {len(df)}  |  Duration: {df['time'].iloc[-1]:.1f}s")

    # Generate plots
    print("Generating plots...")
    plot_states(df, x_cols, s_lab, args.save, args.dpi)
    plot_controls(df, u_cols, u_lab, args.save, args.dpi)
    plot_phase(df, x_cols, s_lab, args.save, args.dpi)
    plot_timing(df, args.save, args.dpi)
    plot_param_safety(df, args.save, args.dpi)
    plot_summary(df, args.save, args.dpi)

    # Optional animation
    if args.animate:
        print("Generating animation...")
        animate_intersection(df, x_cols, args.save, args.dpi)

    print("Done.")


if __name__ == "__main__":
    main()
