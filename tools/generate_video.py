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

def main():
    parser = argparse.ArgumentParser(description="Generate 3D Safe Set Evolution Video")
    parser.add_argument("--cfg", required=True, help="Path to pFaces .cfg file")
    parser.add_argument("--csv", help="Path to basis_coordinates.csv (defaults to cfg folder)")
    parser.add_argument("--out", help="Output file name (defaults to cfg folder)")
    parser.add_argument("--gif", action="store_true", help="Generate ONLY a GIF")
    parser.add_argument("--fps", type=int, default=25, help="Frames per second")
    parser.add_argument("--dpi", type=int, default=150, help="DPI resolution (quality)")
    parser.add_argument("--sample", type=int, default=1, help="Sample every Nth iteration")
    parser.add_argument("--hold", type=int, default=2, help="Hold each iteration for N frames (slowness)")
    parser.add_argument("--rotations", type=float, default=1, help="Number of rotations")
    parser.add_argument("--cmap", default="plasma", help="Matplotlib colormap")
    parser.add_argument("--theme", choices=["dark", "light"], default="light", help="UI theme")
    
    args = parser.parse_args()

    # Determine default paths relative to cfg
    cfg_dir = os.path.dirname(os.path.abspath(args.cfg))
    
    if not args.csv:
        args.csv = os.path.join(cfg_dir, "basis_coordinates.csv")
    
    if not args.out:
        ext = ".gif" if args.gif else ".mp4"
        args.out = os.path.join(cfg_dir, "safe_set_evolution" + ext)

    # Load config
    config = parse_config_file(args.cfg)
    if not config:
        sys.exit(1)

    # Load data
    print(f"Loading data from {args.csv}...")
    iterations, data = load_data(args.csv, config)
    print(f"✓ Loaded {len(iterations)} iterations, {len(data[-1])} final basis elements.")

    # Setup plotting
    if args.theme == "dark":
        plt.style.use('dark_background')
        edge_color = 'none'
    else:
        plt.style.use('default')
        edge_color = 'white'

    fig = plt.figure(figsize=(18, 12), dpi=args.dpi)
    ax = fig.add_subplot(111, projection='3d', position=[0.05, 0.05, 0.85, 0.9])
    
    # Pre-calculated rotation settings
    total_rot_deg = 360 * args.rotations
    initial_azim = 45
    elevation = 25

    # Bounds
    x_bounds = (config['lb'][0], config['ub'][0])
    y_bounds = (config['lb'][1], config['ub'][1])
    if len(config['lb']) >= 3:
        z_bounds = (config['lb'][2], config['ub'][2])
    else:
        z_bounds = (-1.0, 1.0)

    # Initialize axis labels and limits ONCE (optimization)
    ax.set_xlabel(config['labels'][0], fontweight='bold', labelpad=10)
    ax.set_ylabel(config['labels'][1], fontweight='bold', labelpad=10)
    ax.set_zlabel(config['labels'][2], fontweight='bold', labelpad=10)
    
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

    # Norm for colors
    norm = Normalize(vmin=1, vmax=iterations[-1])
    sm = cm.ScalarMappable(cmap=args.cmap, norm=norm)
    sm.set_array([])
    
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label('Birth Iteration', fontweight='bold', rotation=270, labelpad=25)

    # Sample iterations
    sampled_indices = list(range(0, len(iterations), args.sample))
    if len(sampled_indices) == 0 or sampled_indices[-1] != len(iterations) - 1:
        sampled_indices.append(len(iterations) - 1)

    # Total frames = sampled_iterations * hold_factor
    total_frames = len(sampled_indices) * args.hold
    
    # Global to keep track of the scatter object
    sc = None

    def update(frame):
        nonlocal sc
        
        # Calculate iteration index
        idx = (frame // args.hold) % len(sampled_indices)
        array_idx = sampled_indices[idx]
        iteration = iterations[array_idx]
        current_data = data[array_idx]
        
        # Smooth Rotation
        azimuth = initial_azim + (frame / total_frames) * total_rot_deg
        ax.view_init(elev=elevation, azim=azimuth)
        
        # Update data ONLY when it changes (every 'hold' frames)
        if frame % args.hold == 0 or sc is None:
            if sc:
                sc.remove()
            
            if len(current_data) > 0:
                sc = ax.scatter(current_data[:, 0], current_data[:, 1], current_data[:, 2],
                          c=current_data[:, 3], cmap=args.cmap, 
                          vmin=1, vmax=iterations[-1],
                          alpha=0.8, s=60, edgecolors=edge_color, linewidths=0.4)
            else:
                # Placeholder to avoid sc being None
                sc = ax.scatter([], [], [])
            
            ax.set_title(f'Iteration {iteration}/{iterations[-1]} - {len(current_data)} basis elements', 
                        fontweight='bold', fontsize=16, pad=20)
        
        return []

    print(f"Generating {'GIF' if args.gif else 'video'}: {args.out}...")
    print(f"Frames: {total_frames}, FPS: {args.fps}, Quality: {args.dpi} DPI")
    
    anim = FuncAnimation(fig, update, frames=total_frames, interval=1000/args.fps)
    
    if args.gif:
        # Optimized GIF saving
        anim.save(args.out, writer=PillowWriter(fps=args.fps))
    else:
        # High quality MP4 with x264
        writer = FFMpegWriter(fps=args.fps, bitrate=12000, codec='libx264',
                             extra_args=['-pix_fmt', 'yuv420p', '-preset', 'medium'])
        anim.save(args.out, writer=writer)

    print(f"✓ Saved to {args.out}")

if __name__ == "__main__":
    main()
