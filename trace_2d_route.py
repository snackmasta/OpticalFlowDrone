#!/usr/bin/env python3
"""
Python Script to Trace and Plot the 2D Ground Trajectory & Route from Optical Flow Session Logs.
"""

import os
import sys
import argparse
import numpy as np
import pandas as pd

import matplotlib
# Use Agg non-interactive backend unless explicitly requested
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


DEFAULT_CSV = os.path.join("recordings", "optical_flow_session_20260724_171327.csv")


def plot_2d_route(csv_path, output_png=None, show_gui=False, compare_raw=True):
    if not os.path.exists(csv_path):
        print(f"Error: Target CSV file '{csv_path}' not found.")
        return

    print(f"Reading session log: {csv_path}")
    df = pd.read_csv(csv_path)

    # Required columns check
    required_cols = ["Timestamp (s)", "X Position (cm)", "Y Position (cm)"]
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        print(f"Error: Missing required column(s) {missing} in {csv_path}")
        return

    # Convert coordinates from cm to meters
    x_m = df["X Position (cm)"].values / 100.0
    y_m = df["Y Position (cm)"].values / 100.0

    raw_x_m = (df["Raw X Position (cm)"].values / 100.0) if "Raw X Position (cm)" in df.columns else None
    raw_y_m = (df["Raw Y Position (cm)"].values / 100.0) if "Raw Y Position (cm)" in df.columns else None

    # Calculate timestamps and velocities
    timestamps = df["Timestamp (s)"].values
    t_elapsed = timestamps - timestamps[0]
    total_duration = t_elapsed[-1]

    # Calculate 2D incremental path distance (arc length)
    dx = np.diff(x_m)
    dy = np.diff(y_m)
    step_dists = np.sqrt(dx**2 + dy**2)
    total_distance_m = np.sum(step_dists)

    # Calculate displacement from origin
    dist_from_origin = np.sqrt(x_m**2 + y_m**2)
    max_disp_m = np.max(dist_from_origin)
    final_disp_m = dist_from_origin[-1]

    # Speed metrics
    if "Speed (m/s)" in df.columns:
        speed = df["Speed (m/s)"].values
    elif "VX (m/s)" in df.columns and "VY (m/s)" in df.columns:
        speed = np.sqrt(df["VX (m/s)"].values**2 + df["VY (m/s)"].values**2)
    else:
        speed = np.zeros_like(x_m)

    max_speed = np.max(speed)
    avg_speed = np.mean(speed)

    # Printing Route Statistics
    print("\n" + "=" * 65)
    print("               2D TRAJECTORY & ROUTE ANALYSIS              ")
    print("=" * 65)
    print(f"Total Duration        : {total_duration:.2f} seconds ({len(df)} points)")
    print(f"Total Path Distance   : {total_distance_m:.3f} meters")
    print(f"Max Displacement      : {max_disp_m:.3f} meters from origin")
    print(f"Final Position        : X = {x_m[-1]:+.3f}m, Y = {y_m[-1]:+.3f}m (dist = {final_disp_m:.3f}m)")
    print(f"Bounding Box Range    : X: [{np.min(x_m):+.2f}m to {np.max(x_m):+.2f}m], Y: [{np.min(y_m):+.2f}m to {np.max(y_m):+.2f}m]")
    print(f"Speed Statistics      : Avg = {avg_speed:.3f} m/s, Max = {max_speed:.3f} m/s")
    print("=" * 65 + "\n")

    # Set up matplotlib figure
    fig, ax = plt.subplots(figsize=(10, 8.5))

    # Optional: Plot raw uncompensated trajectory if available
    if compare_raw and raw_x_m is not None and raw_y_m is not None:
        ax.plot(raw_x_m, raw_y_m, color='#94a3b8', linestyle='--', linewidth=1.2, alpha=0.6, label='Raw Uncompensated Route')

    # Create colored multi-segment line collection by time progress
    points = np.array([x_m, y_m]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    lc = LineCollection(segments, cmap='turbo', norm=plt.Normalize(0, total_duration))
    lc.set_array(t_elapsed[:-1])
    lc.set_linewidth(2.5)
    line = ax.add_collection(lc)

    # Add colorbar for time progress
    cbar = fig.colorbar(line, ax=ax, pad=0.02)
    cbar.set_label('Time Elapsed (seconds)', fontsize=11, fontweight='bold')

    # Draw direction quiver arrows periodically along the route
    sub = max(1, len(dx) // 25)
    ax.quiver(
        x_m[:-1:sub], y_m[:-1:sub],
        dx[::sub], dy[::sub],
        color='#0284c7', scale=1, scale_units='xy', angles='xy',
        width=0.005, headwidth=4, headlength=5, alpha=0.8, label='Direction Vector'
    )

    # Highlight Start and End points
    ax.plot(x_m[0], y_m[0], marker='o', markersize=10, color='#10b981', markeredgecolor='black', markeredgewidth=1.5, zorder=5, label=f'START (0.00m, 0.00m)')
    ax.plot(x_m[-1], y_m[-1], marker='s', markersize=10, color='#ef4444', markeredgecolor='black', markeredgewidth=1.5, zorder=5, label=f'END ({x_m[-1]:+.2f}m, {y_m[-1]:+.2f}m)')

    # Add Origin crosshairs
    ax.axhline(0, color='#cbd5e1', linestyle='-', linewidth=0.8, alpha=0.7)
    ax.axvline(0, color='#cbd5e1', linestyle='-', linewidth=0.8, alpha=0.7)

    # Axes titles and formatting
    ax.set_title(f'Optical Flow 2D Trajectory Route — ({os.path.basename(csv_path)})', fontsize=13, fontweight='bold', pad=12)
    ax.set_xlabel('X Position (meters) [East / Right]', fontsize=11, fontweight='bold')
    ax.set_ylabel('Y Position (meters) [North / Forward]', fontsize=11, fontweight='bold')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.95, fontsize=10)

    # Annotate Route Metrics Summary Box
    metrics_text = (
        f"Route Metrics:\n"
        f" • Duration    : {total_duration:.1f} s\n"
        f" • Arc Length  : {total_distance_m:.2f} m\n"
        f" • Max Dist    : {max_disp_m:.2f} m\n"
        f" • Avg Speed   : {avg_speed:.2f} m/s\n"
        f" • Max Speed   : {max_speed:.2f} m/s"
    )
    ax.text(0.02, 0.98, metrics_text, transform=ax.transAxes, fontsize=9.5, fontfamily='monospace',
            verticalalignment='top', bbox=dict(boxstyle='round,pad=0.6', facecolor='#f8fafc', edgecolor='#94a3b8', alpha=0.9))

    plt.tight_layout()

    # Save figure
    if not output_png:
        base_name = os.path.splitext(os.path.basename(csv_path))[0]
        output_png = os.path.join(os.path.dirname(csv_path) or ".", f"{base_name}_2d_route.png")

    plt.savefig(output_png, dpi=300, bbox_inches='tight')
    print(f"2D Trajectory Route plot saved to: {output_png}")

    if show_gui:
        matplotlib.use('TkAgg')
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Trace 2D position trajectory and route from optical flow CSV log.")
    parser.add_argument("csv_file", nargs="?", default=DEFAULT_CSV, help="Path to optical flow session CSV file.")
    parser.add_argument("-o", "--output", help="Path to save output PNG plot.")
    parser.add_argument("--no-raw", action="store_true", help="Disable plotting raw uncompensated position comparison.")
    parser.add_argument("--show", action="store_true", help="Display interactive GUI plot window.")
    args = parser.parse_args()

    plot_2d_route(args.csv_file, output_png=args.output, show_gui=args.show, compare_raw=not args.no_raw)


if __name__ == "__main__":
    main()
