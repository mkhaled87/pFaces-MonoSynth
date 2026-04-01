#!/usr/bin/env python3
"""
Visualize trajectory colored by dual sub-SafeSet status
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
from pathlib import Path

def main():
    if len(sys.argv) < 2:
        print("Usage: visualize_dual_safety.py <sim_log.csv>")
        sys.exit(1)
    
    csv_path = Path(sys.argv[1])
    output_dir = csv_path.parent
    
    df = pd.read_csv(csv_path)
    
    # Create safety categories
    # 0: neither safe (red)
    # 1: wait only (blue)
    # 2: go only (yellow/orange)
    # 3: both safe (green)
    df['safety_cat'] = df['wait_safe'] + 2 * df['go_safe']
    
    colors = ['red', 'blue', 'orange', 'green']
    labels = ['Neither', 'Wait only', 'Go only', 'Both']
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Trajectory Colored by Sub-SafeSet Status', fontsize=16)
    
    # Plot 1: s_ego vs time
    ax = axes[0, 0]
    for cat in range(4):
        mask = df['safety_cat'] == cat
        if mask.any():
            ax.scatter(df[mask]['time'], df[mask]['x0'], c=colors[cat], 
                      label=labels[cat], s=20, alpha=0.7)
    ax.axhline(10, color='purple', linestyle='--', label='Goal', alpha=0.5)
    ax.axhline(0, color='gray', linestyle=':', alpha=0.3)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('s_ego (m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 2: v_ego vs time
    ax = axes[0, 1]
    for cat in range(4):
        mask = df['safety_cat'] == cat
        if mask.any():
            ax.scatter(df[mask]['time'], df[mask]['x1'], c=colors[cat], 
                      label=labels[cat], s=20, alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('v_ego (m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 3: s_ego vs v_ego phase plot
    ax = axes[1, 0]
    for cat in range(4):
        mask = df['safety_cat'] == cat
        if mask.any():
            ax.scatter(df[mask]['x0'], df[mask]['x1'], c=colors[cat], 
                      label=labels[cat], s=30, alpha=0.7, marker='o')
    ax.axvline(10, color='purple', linestyle='--', alpha=0.5)
    ax.axvline(0, color='gray', linestyle=':', alpha=0.3)
    ax.set_xlabel('s_ego (m)')
    ax.set_ylabel('v_ego (m/s)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: oncoming positions vs time
    ax = axes[1, 1]
    ax.plot(df['time'], df['x2'], label='s_onc1', color='blue', alpha=0.6)
    ax.plot(df['time'], df['x3'], label='s_onc2', color='orange', alpha=0.6)
    ax.axhline(10, color='purple', linestyle='--', label='CZ boundary', alpha=0.5)
    ax.axhline(-10, color='purple', linestyle='--', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Oncoming Position (m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = output_dir / 'dual_safety.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Saved {output_path}")
    
    # Print statistics
    print("\n=== Safety Statistics ===")
    for cat, label in enumerate(labels):
        count = (df['safety_cat'] == cat).sum()
        pct = 100 * count / len(df)
        print(f"{label:12s}: {count:3d}/{len(df)} ({pct:5.1f}%)")
    
    # Find first step where each sub-SafeSet becomes safe
    wait_first = df[df['wait_safe'] == 1].iloc[0] if df['wait_safe'].any() else None
    go_first = df[df['go_safe'] == 1].iloc[0] if df['go_safe'].any() else None
    
    if wait_first is not None:
        print(f"\nFirst wait-safe at t={wait_first['time']:.1f}s: "
              f"s_ego={wait_first['x0']:.1f}, s_onc1={wait_first['x2']:.1f}")
    if go_first is not None:
        print(f"First go-safe at t={go_first['time']:.1f}s: "
              f"s_ego={go_first['x0']:.1f}, s_onc2={go_first['x3']:.1f}")

if __name__ == '__main__':
    main()
