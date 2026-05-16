#!/usr/bin/env python3
"""
pFaces-MonoSynth: Generalized 3D Safe Set Evolution Video Generator
Automatically extracts state space parameters from pFaces .cfg files
and generates an evolution video from basis_coordinates.csv.
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv
import sys
import re
import argparse
import os
from matplotlib.animation import FuncAnimation, FFMpegWriter, PillowWriter
from matplotlib import cm
from matplotlib.colors import Normalize

def parse_config_file(cfg_file):
    """
    Parse pFaces .cfg file to extract state space parameters.
    Returns dict with lb, ub, eta, and priorities arrays.
    """
    try:
        if not os.path.exists(cfg_file):
            print(f"Error: Config file {cfg_file} not found.")
            return None

        with open(cfg_file, 'r') as f:
            content = f.read()
        
        # Extract states block
        states_match = re.search(r'states\s*\{([^}]+)\}', content, re.DOTALL)
        if not states_match:
            print(f"Warning: Could not find states block in {cfg_file}")
            return None
        
        states_block = states_match.group(1)
        
        # Parse arrays (eta, lb, ub, priorities)
        def parse_array(text, key):
            pattern = rf'{key}\s*=\s*"([^"]+)"'
            match = re.search(pattern, text)
            if match:
                values_str = match.group(1)
                return [float(x.strip()) for x in values_str.split(',')]
            return None
        
        eta = parse_array(states_block, 'eta')
        lb = parse_array(states_block, 'lb')
        ub = parse_array(states_block, 'ub')
        priorities = parse_array(states_block, 'priorities')
        
        # Convert priorities to int
        if priorities:
            priorities = [int(p) for p in priorities]
        
        # Try to extract labels from comments
        labels = ["State 1", "State 2", "State 3"]
        label_match = re.search(r'# State Space:\s*\[(.*?)\]', content)
        if label_match:
            extracted_labels = [l.strip() for l in label_match.group(1).split(',')]
            for i, l in enumerate(extracted_labels):
                if i < len(labels):
                    labels[i] = l

        if not all([eta, lb, ub, priorities]):
            print(f"Warning: Could not parse all required parameters from {cfg_file}")
            return None
        
        return {
            'lb': lb,
            'ub': ub,
            'eta': eta,
            'priorities': priorities,
            'labels': labels
        }
    
    except Exception as e:
        print(f"Warning: Error parsing {cfg_file}: {e}")
        return None

def grid_to_physical(idx, dim_idx, config):
    """
    Convert grid index (1-based) to physical coordinate
    """
    lb = config['lb'][dim_idx]
    ub = config['ub'][dim_idx]
    eta = config['eta'][dim_idx]
    priority = config['priorities'][dim_idx]
    
    if priority == 1:
        return lb + (idx - 1) * eta
    else:
        return ub - (idx - 1) * eta

def load_data(csv_file, config):
    """Load basis coordinates and track earliest iteration each point appeared"""
    try:
        if not os.path.exists(csv_file):
            print(f"Error: CSV file {csv_file} not found.")
            sys.exit(1)

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            
            point_birth = {}  # tuple of indices -> earliest_iteration
            iterations_data = {}  # iteration -> list of points
            seen_in_iteration = {} # iteration -> set of point_keys
            
            for row in reader:
                iteration = int(row['iteration'])
                
                # Dynamically get indices based on dimensionality found in config
                dims = len(config['lb'])
                indices = []
                valid = True
                for d in range(dims):
                    idx_name = f'idx{d}'
                    if idx_name not in row:
                        valid = False
                        break
                    idx_val = int(row[idx_name])
                    if idx_val <= 0:
                        valid = False
                        break
                    indices.append(idx_val)
                
                if not valid:
                    continue
                
                # skip if already seen in THIS iteration (e.g. from multiple benchmark runs)
                if iteration not in seen_in_iteration:
                    seen_in_iteration[iteration] = set()
                
                point_key = tuple(indices)
                if point_key in seen_in_iteration[iteration]:
                    continue
                seen_in_iteration[iteration].add(point_key)
                
                # Record EARLIEST appearance only
                if point_key not in point_birth:
                    point_birth[point_key] = iteration
                
                # Convert only first 3 dims to physical coordinates for 3D plotting
                phys_coords = []
                for d in range(min(dims, 3)):
                    phys_coords.append(grid_to_physical(indices[d], d, config))
                
                # Fill missing dims for 3D if system is 2D
                if len(phys_coords) == 2:
                    phys_coords.append(0.0)
                
                if iteration not in iterations_data:
                    iterations_data[iteration] = []
                
                iterations_data[iteration].append(phys_coords + [point_birth[point_key]])
        
        iterations = sorted(iterations_data.keys())
        basis_coords_with_birth = [np.array(iterations_data[i]) for i in iterations]
        
        return iterations, basis_coords_with_birth
        
    except Exception as e:
        print(f"Error loading data: {e}")
        sys.exit(1)


def load_threshold_data(csv_file, config):
    """Load threshold evolution CSV. Each row is a column-top point per iteration.
    Returns (iterations, data_per_iteration) where data[:,0:3] = physical coords,
    data[:,3] = tau value (column height, used for coloring)."""
    try:
        if not os.path.exists(csv_file):
            print(f"Error: CSV file {csv_file} not found.")
            sys.exit(1)

        dims = len(config['lb'])
        # Compute max tau (N_d* = grid size of d_star) for color normalization
        # d_star is the dim with largest grid extent
        grid_sizes = []
        for d in range(dims):
            N = round((config['ub'][d] - config['lb'][d]) / config['eta'][d]) + 1
            grid_sizes.append(N)

        iterations_data = {}
        seen_in_iteration = {}

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                iteration = int(row['iteration'])
                tau = int(row['tau'])

                indices = []
                valid = True
                for d in range(dims):
                    idx_name = f'idx{d}'
                    if idx_name not in row:
                        valid = False
                        break
                    idx_val = int(row[idx_name])
                    if idx_val <= 0:
                        valid = False
                        break
                    indices.append(idx_val)

                if not valid:
                    continue

                if iteration not in seen_in_iteration:
                    seen_in_iteration[iteration] = set()
                point_key = tuple(indices)
                if point_key in seen_in_iteration[iteration]:
                    continue
                seen_in_iteration[iteration].add(point_key)

                phys_coords = []
                for d in range(min(dims, 3)):
                    phys_coords.append(grid_to_physical(indices[d], d, config))
                if len(phys_coords) == 2:
                    phys_coords.append(0.0)

                if iteration not in iterations_data:
                    iterations_data[iteration] = []
                iterations_data[iteration].append(phys_coords + [float(tau)])

        iterations = sorted(iterations_data.keys())
        data = [np.array(iterations_data[i]) for i in iterations]
        return iterations, data

    except Exception as e:
        print(f"Error loading threshold data: {e}")
        sys.exit(1)

def auto_label_from_cfg(cfg_path):
    """Derive a human-readable animation title from the cfg filename."""
    name = os.path.splitext(os.path.basename(cfg_path))[0]
    # Strip suffixes like _basis, _tt, _fine, _coarse
    for suffix in ('_basis', '_tt', '_fine', '_coarse'):
        name = name.replace(suffix, '')
    # Title-case words
    return ' '.join(w.capitalize() for w in name.replace('_', ' ').split())


def method_label(mode):
    return 'TT-only' if mode == 'threshold' else 'Basis'


def main():
    parser = argparse.ArgumentParser(description="Generate 3D Safe Set Evolution Video")
    parser.add_argument("--cfg", required=True, help="Path to pFaces .cfg file")
    parser.add_argument("--csv", help="Path to CSV data file (auto-detected from mode if omitted)")
    parser.add_argument("--out", help="Output file name (defaults to cfg folder)")
    parser.add_argument("--gif", action="store_true", help="Generate ONLY a GIF")
    parser.add_argument("--fps", type=int, default=25, help="Frames per second")
    parser.add_argument("--dpi", type=int, default=150, help="DPI resolution (quality)")
    parser.add_argument("--sample", type=int, default=1, help="Sample every Nth iteration")
    parser.add_argument("--hold", type=int, default=2, help="Hold each iteration for N frames (slowness)")
    parser.add_argument("--rotations", type=float, default=0.4, help="Number of rotations")
    parser.add_argument("--cmap", default="plasma", help="Matplotlib colormap")
    parser.add_argument("--theme", choices=["dark", "light"], default="light", help="UI theme")
    parser.add_argument("--mode", choices=["basis", "threshold"], default="basis",
                        help="Data mode: 'basis' for basis_coordinates.csv, 'threshold' for threshold_evolution.csv")
    parser.add_argument("--label", default="", help="Custom title prefix (auto-derived from cfg if omitted)")
    parser.add_argument("--marker-size", type=float, default=0,
                        help="Scatter marker size; 0 = auto-adaptive based on point count")

    args = parser.parse_args()

    # Determine default paths relative to cfg
    cfg_dir = os.path.dirname(os.path.abspath(args.cfg))

    if not args.csv:
        if args.mode == "threshold":
            args.csv = os.path.join(cfg_dir, "threshold_evolution.csv")
        else:
            args.csv = os.path.join(cfg_dir, "basis_coordinates.csv")

    if not args.out:
        ext = ".gif" if args.gif else ".mp4"
        prefix = "threshold_evolution" if args.mode == "threshold" else "safe_set_evolution"
        args.out = os.path.join(cfg_dir, prefix + ext)

    # Build title prefix
    title_prefix = args.label if args.label else auto_label_from_cfg(args.cfg)
    title_prefix = f"{title_prefix}  [{method_label(args.mode)} Method]"

    # Load config
    config = parse_config_file(args.cfg)
    if not config:
        sys.exit(1)

    # Compute grid sizes for info display
    dims = len(config['lb'])
    grid_sizes = []
    for d in range(dims):
        N = round((config['ub'][d] - config['lb'][d]) / config['eta'][d]) + 1
        grid_sizes.append(N)
    total_cells = 1
    for N in grid_sizes:
        total_cells *= N
    grid_str = 'x'.join(str(N) for N in grid_sizes)

    # Load data
    print(f"Loading data from {args.csv}...")
    if args.mode == "threshold":
        iterations, data = load_threshold_data(args.csv, config)
    else:
        iterations, data = load_data(args.csv, config)
    element_label = "threshold columns" if args.mode == "threshold" else "basis elements"
    print(f"  Loaded {len(iterations)} iterations, {len(data[-1])} final {element_label}.")

    # Compute adaptive marker size from maximum number of points in any frame
    if args.marker_size > 0:
        marker_size = args.marker_size
    else:
        max_pts = max((len(d) for d in data if len(d) > 0), default=1)
        # Scale: ~60px for 100 pts, ~8px for 10k pts, minimum 4
        marker_size = max(4.0, min(80.0, 6000.0 / max(1, max_pts ** 0.7)))
    print(f"  Grid: {grid_str} ({total_cells:,} cells)  Marker size: {marker_size:.1f}")

    # Setup plotting
    if args.theme == "dark":
        plt.style.use('dark_background')
        edge_color = 'none'
        text_color = 'white'
    else:
        plt.style.use('default')
        edge_color = 'none'
        text_color = 'black'

    fig = plt.figure(figsize=(16, 10), dpi=args.dpi)
    ax = fig.add_subplot(111, projection='3d', position=[0.05, 0.05, 0.82, 0.88])

    # Tick label size
    ax.tick_params(axis='both', labelsize=10)

    # Pre-calculated rotation settings
    total_rot_deg = 360 * args.rotations
    initial_azim = 45
    elevation = 28

    # Bounds
    x_bounds = (config['lb'][0], config['ub'][0])
    y_bounds = (config['lb'][1], config['ub'][1])
    z_bounds = (config['lb'][2], config['ub'][2]) if len(config['lb']) >= 3 else (-1.0, 1.0)

    # Axis labels
    ax.set_xlabel(config['labels'][0], fontweight='bold', fontsize=12, labelpad=12)
    ax.set_ylabel(config['labels'][1], fontweight='bold', fontsize=12, labelpad=12)
    ax.set_zlabel(config['labels'][2], fontweight='bold', fontsize=12, labelpad=12)

    if config['priorities'][0] == 0:
        ax.set_xlim(x_bounds[1], x_bounds[0])
    else:
        ax.set_xlim(x_bounds[0], x_bounds[1])

    if config['priorities'][1] == 0:
        ax.set_ylim(y_bounds[1], y_bounds[0])
    else:
        ax.set_ylim(y_bounds[0], y_bounds[1])

    if len(config['priorities']) >= 3:
        if config['priorities'][2] == 0:
            ax.set_zlim(z_bounds[1], z_bounds[0])
        else:
            ax.set_zlim(z_bounds[0], z_bounds[1])
    else:
        ax.set_zlim(-1, 1)

    # Corner markers (static, persist through animation)
    # "Start corner" = all dims at their highest grid index (where basis iteration begins)
    # "End corner"   = all dims at their lowest grid index (opposite extreme)
    high_corner = []
    low_corner  = []
    for d in range(min(dims, 3)):
        if config['priorities'][d] == 1:   # ascending: idx_max → ub
            high_corner.append(config['ub'][d])
            low_corner.append(config['lb'][d])
        else:                              # descending: idx_max → lb
            high_corner.append(config['lb'][d])
            low_corner.append(config['ub'][d])

    ax.scatter(*high_corner, color='gold', s=260, marker='*', zorder=10,
               edgecolors='darkorange', linewidths=1.5)
    ax.text(high_corner[0], high_corner[1], high_corner[2],
            '  Start', color='darkorange', fontsize=9, fontweight='bold', zorder=11)
    ax.scatter(*low_corner, color='deepskyblue', s=140, marker='D', zorder=10,
               edgecolors='royalblue', linewidths=1.0)
    ax.text(low_corner[0], low_corner[1], low_corner[2],
            '  End', color='royalblue', fontsize=9, fontweight='bold', zorder=11)

    # Color normalization
    if args.mode == "threshold":
        all_taus = np.concatenate([d[:, 3] for d in data if len(d) > 0])
        norm = Normalize(vmin=max(1, all_taus.min()), vmax=all_taus.max())
        cbar_label = 'Threshold \u03c4 (column height)'
    else:
        norm = Normalize(vmin=1, vmax=iterations[-1])
        cbar_label = 'Birth Iteration'
    sm = cm.ScalarMappable(cmap=args.cmap, norm=norm)
    sm.set_array([])

    cbar_ax = fig.add_axes([0.89, 0.15, 0.02, 0.65])
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label(cbar_label, fontweight='bold', fontsize=10, rotation=270, labelpad=22)
    cbar.ax.tick_params(labelsize=9)

    # Grid info text (static, bottom-left)
    fig.text(0.05, 0.01, f'Grid: {grid_str}  ({total_cells:,} cells)',
             fontsize=9, color=text_color, alpha=0.7)

    # Sample iterations for animation frames
    sampled_indices = list(range(0, len(iterations), args.sample))
    if not sampled_indices or sampled_indices[-1] != len(iterations) - 1:
        sampled_indices.append(len(iterations) - 1)

    total_frames = len(sampled_indices) * args.hold
    sc = None

    def update(frame):
        nonlocal sc

        idx = (frame // args.hold) % len(sampled_indices)
        array_idx = sampled_indices[idx]
        iteration = iterations[array_idx]
        current_data = data[array_idx]

        # Smooth rotation
        azimuth = initial_azim + (frame / total_frames) * total_rot_deg
        ax.view_init(elev=elevation, azim=azimuth)

        if frame % args.hold == 0 or sc is None:
            if sc:
                sc.remove()

            if len(current_data) > 0:
                # Adaptive alpha: reduce for very dense point clouds
                n_pts = len(current_data)
                alpha = max(0.35, min(0.92, 1.0 - n_pts / (n_pts + 2000)))
                sc = ax.scatter(
                    current_data[:, 0], current_data[:, 1], current_data[:, 2],
                    c=current_data[:, 3], cmap=args.cmap, norm=norm,
                    alpha=alpha, s=marker_size,
                    edgecolors=edge_color, linewidths=0.0,
                    depthshade=True
                )
            else:
                sc = ax.scatter([], [], [])

            pct_done = 100.0 * iteration / iterations[-1]
            ax.set_title(
                f'{title_prefix}\n'
                f'Iteration {iteration}/{iterations[-1]}  ({pct_done:.0f}%)   '
                f'{len(current_data):,} {element_label}',
                fontweight='bold', fontsize=13, pad=12, color=text_color
            )

        return []

    print(f"Generating {'GIF' if args.gif else 'MP4'}: {args.out}")
    print(f"  Frames: {total_frames}, FPS: {args.fps}, DPI: {args.dpi}")

    anim = FuncAnimation(fig, update, frames=total_frames, interval=1000 / args.fps)

    if args.gif:
        anim.save(args.out, writer=PillowWriter(fps=args.fps))
    else:
        writer = FFMpegWriter(fps=args.fps, bitrate=14000, codec='libx264',
                              extra_args=['-pix_fmt', 'yuv420p', '-preset', 'medium'])
        anim.save(args.out, writer=writer)

    print(f"  Saved to {args.out}")


if __name__ == "__main__":
    main()
