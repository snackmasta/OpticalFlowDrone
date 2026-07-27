#!/usr/bin/env python3
"""
Python Script to Graph Roll and Pitch Angles Over Time from Optical Flow CSV Logs.
"""

import os
import sys
import argparse

import pandas as pd

import matplotlib
# Use Agg non-interactive backend unless explicitly asked to show GUI window
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot_roll_pitch(csv_path, output_png=None, show_gui=False):
    if not os.path.exists(csv_path):
        print(f"Error: Target CSV file '{csv_path}' not found.")
        return

    print(f"Reading session log: {csv_path}")
    df = pd.read_csv(csv_path)

    # Check required columns
    required_cols = ["Timestamp (s)", "Roll (deg)", "Pitch (deg)"]
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        print(f"Error: Missing required column(s) {missing} in {csv_path}")
        return

    # Calculate elapsed time in seconds from initial timestamp
    start_time = df["Timestamp (s)"].iloc[0]
    elapsed_time = df["Timestamp (s)"] - start_time
    roll = df["Roll (deg)"]
    pitch = df["Pitch (deg)"]

    # Compute statistics
    roll_min, roll_max, roll_mean, roll_std = roll.min(), roll.max(), roll.mean(), roll.std()
    pitch_min, pitch_max, pitch_mean, pitch_std = pitch.min(), pitch.max(), pitch.mean(), pitch.std()
    duration = elapsed_time.iloc[-1]

    print("\n" + "=" * 60)
    print("          ROLL & PITCH ATTITUDE TRACKING SUMMARY          ")
    print("=" * 60)
    print(f"Session Duration : {duration:.2f} seconds ({len(df)} samples)")
    print(f"Roll  (deg)      : Min = {roll_min:+.2f}°, Max = {roll_max:+.2f}°, Mean = {roll_mean:+.2f}°, Std = {roll_std:.2f}°")
    print(f"Pitch (deg)      : Min = {pitch_min:+.2f}°, Max = {pitch_max:+.2f}°, Mean = {pitch_mean:+.2f}°, Std = {pitch_std:.2f}°")
    print("=" * 60 + "\n")

    # Set up plot styling
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    # Subplot 1: Roll Angle over Time
    ax1.plot(elapsed_time, roll, color='#ec4899', linewidth=1.5, label='Roll (°)')
    ax1.axhline(0, color='#94a3b8', linestyle='--', linewidth=0.8, alpha=0.7)
    ax1.set_ylabel('Roll (°)', fontsize=11, fontweight='bold', color='#ec4899')
    ax1.set_title(f'Drone Attitude Tracking — Roll & Pitch ({os.path.basename(csv_path)})', fontsize=13, fontweight='bold', pad=12)
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

    # Subplot 2: Pitch Angle over Time
    ax2.plot(elapsed_time, pitch, color='#10b981', linewidth=1.5, label='Pitch (°)')
    ax2.axhline(0, color='#94a3b8', linestyle='--', linewidth=0.8, alpha=0.7)
    ax2.set_xlabel('Elapsed Time (seconds)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('Pitch (°)', fontsize=11, fontweight='bold', color='#10b981')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', frameon=True, facecolor='white', framealpha=0.9)

    # Annotate summary stats box
    stats_text = (
        f"Roll : Mean = {roll_mean:+.2f}° | Std = {roll_std:.2f}° | Range = [{roll_min:+.1f}°, {roll_max:+.1f}°]\n"
        f"Pitch: Mean = {pitch_mean:+.2f}° | Std = {pitch_std:.2f}° | Range = [{pitch_min:+.1f}°, {pitch_max:+.1f}°]"
    )
    fig.text(0.14, 0.02, stats_text, fontsize=9, fontfamily='monospace',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8fafc', edgecolor='#cbd5e1', alpha=0.9))

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.12)

    # Save figure
    if not output_png:
        base_name = os.path.splitext(os.path.basename(csv_path))[0]
        output_png = os.path.join(os.path.dirname(csv_path) or ".", f"{base_name}_roll_pitch.png")

    plt.savefig(output_png, dpi=300, bbox_inches='tight')
    print(f"High-resolution plot saved to: {output_png}")

    if show_gui:
        matplotlib.use('TkAgg')
        plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Graph Roll and Pitch angles over time from optical flow CSV log.")
    parser.add_argument("csv_file", nargs="?", default=os.path.join("recordings", "optical_flow_session_20260724_165535.csv"),
                        help="Path to optical flow session CSV file.")
    parser.add_argument("-o", "--output", help="Path to save output PNG plot.")
    parser.add_argument("--show", action="store_true", help="Display interactive GUI plot window.")
    args = parser.parse_args()

    plot_roll_pitch(args.csv_file, output_png=args.output, show_gui=args.show)
