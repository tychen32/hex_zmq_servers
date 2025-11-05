#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-11-05
################################################################

import os
import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = f"{SCRIPT_DIR}/csv/traj_sim.csv"
PLOTS_DIR = f"{SCRIPT_DIR}/plots"


def load_data(csv_path, ignore_s=2):
    """Load trajectory data from CSV file."""
    # Load CSV manually using numpy
    data = np.genfromtxt(csv_path,
                         delimiter=',',
                         names=True,
                         dtype=None,
                         encoding='utf-8')
    print(f"Loaded {len(data)} data points from {csv_path}")
    
    # Filter data: only keep ts > ignore_s * 1e9
    original_count = len(data)
    data = data[data['ts'] > ignore_s * 1e9]
    filtered_count = len(data)
    
    if filtered_count < original_count:
        print(f"Filtered: {original_count - filtered_count} points removed (ts <= {ignore_s}e9)")
        print(f"Analyzing: {filtered_count} data points (ts > {ignore_s}e9)")
    
    print(f"Columns: {list(data.dtype.names)}")
    return data


def plot_joint_positions(data, joint_indices=None):
    """Plot joint positions over time."""
    if joint_indices is None:
        joint_indices = range(7)

    fig, axes = plt.subplots(len(joint_indices),
                             1,
                             figsize=(12, 2.5 * len(joint_indices)))
    if len(joint_indices) == 1:
        axes = [axes]

    # Convert timestamp to relative time in seconds
    time = (data['ts'] - data['ts'][0]) / 1e9

    for idx, joint_idx in enumerate(joint_indices):
        ax = axes[idx]
        ax.plot(time,
                data[f'q{joint_idx}'],
                label=f'q{joint_idx} (actual)',
                linewidth=1.5)
        ax.plot(time,
                data[f'mid_q{joint_idx}'],
                label=f'mid_q{joint_idx} (interpolated)',
                linewidth=1.5,
                linestyle='--',
                alpha=0.7)
        ax.plot(time,
                data[f'ik_q{joint_idx}'],
                label=f'ik_q{joint_idx} (IK target)',
                linewidth=1.0,
                linestyle='--',
                alpha=0.7)
        ax.set_ylabel(f'Joint {joint_idx} (rad)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Joint Positions', fontsize=14, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_joint_velocities(data, joint_indices=None):
    """Plot joint velocities over time."""
    if joint_indices is None:
        joint_indices = range(7)

    fig, axes = plt.subplots(len(joint_indices),
                             1,
                             figsize=(12, 2.5 * len(joint_indices)))
    if len(joint_indices) == 1:
        axes = [axes]

    # Convert timestamp to relative time in seconds
    time = (data['ts'] - data['ts'][0]) / 1e9

    for idx, joint_idx in enumerate(joint_indices):
        ax = axes[idx]
        ax.plot(time,
                data[f'dq{joint_idx}'],
                label=f'dq{joint_idx}',
                linewidth=1.5)
        ax.set_ylabel(f'Joint {joint_idx} (rad/s)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Joint Velocities', fontsize=14, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_joint_efforts(data, joint_indices=None):
    """Plot joint efforts/torques over time."""
    if joint_indices is None:
        joint_indices = range(7)

    fig, axes = plt.subplots(len(joint_indices),
                             1,
                             figsize=(12, 2.5 * len(joint_indices)))
    if len(joint_indices) == 1:
        axes = [axes]

    # Convert timestamp to relative time in seconds
    time = (data['ts'] - data['ts'][0]) / 1e9

    for idx, joint_idx in enumerate(joint_indices):
        ax = axes[idx]
        ax.plot(time,
                data[f'eff{joint_idx}'],
                label=f'eff{joint_idx}',
                linewidth=1.5)
        ax.set_ylabel(f'Joint {joint_idx} (Nm)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Joint Efforts/Torques', fontsize=14, fontweight='bold')
    plt.tight_layout()
    return fig


def plot_tracking_errors(data, joint_indices=None):
    """Plot tracking errors (target - actual) for joints."""
    if joint_indices is None:
        joint_indices = range(7)

    fig, axes = plt.subplots(len(joint_indices),
                             1,
                             figsize=(12, 2.5 * len(joint_indices)))
    if len(joint_indices) == 1:
        axes = [axes]

    # Convert timestamp to relative time in seconds
    time = (data['ts'] - data['ts'][0]) / 1e9

    for idx, joint_idx in enumerate(joint_indices):
        ax = axes[idx]
        error_mid_q = data[f'mid_q{joint_idx}'] - data[f'q{joint_idx}']
        error_ik_q = data[f'ik_q{joint_idx}'] - data[f'q{joint_idx}']
        
        ax.plot(time, error_mid_q, label=f'mid_q{joint_idx} error', linewidth=1.5)
        ax.plot(time, error_ik_q, label=f'ik_q{joint_idx} error', linewidth=1.0, linestyle='--', alpha=0.7)
        
        ax.set_ylabel(f'Joint {joint_idx} error (rad)')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

        # Add statistics text
        mean_err_mid = np.mean(np.abs(error_mid_q))
        max_err_mid = np.max(np.abs(error_mid_q))
        mean_err_ik = np.mean(np.abs(error_ik_q))
        max_err_ik = np.max(np.abs(error_ik_q))
        
        ax.text(0.02,
                0.98,
                f'mid_q: Mean={mean_err_mid:.5f}, Max={max_err_mid:.5f}\n' +
                f'ik_q: Mean={mean_err_ik:.5f}, Max={max_err_ik:.5f}',
                transform=ax.transAxes,
                verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Joint Tracking Errors (Target - Actual)',
                 fontsize=14,
                 fontweight='bold')
    plt.tight_layout()
    return fig


def plot_summary_statistics(data):
    """Plot summary statistics for all joints."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    joints = range(7)

    # Position ranges
    ax = axes[0, 0]
    pos_ranges = [
        np.max(data[f'q{j}']) - np.min(data[f'q{j}']) for j in joints
    ]
    ax.bar(joints, pos_ranges, color='steelblue', alpha=0.7)
    ax.set_xlabel('Joint Index')
    ax.set_ylabel('Position Range (rad)')
    ax.set_title('Joint Position Ranges')
    ax.grid(True, alpha=0.3, axis='y')

    # Velocity ranges
    ax = axes[0, 1]
    vel_max = [np.max(np.abs(data[f'dq{j}'])) for j in joints]
    ax.bar(joints, vel_max, color='darkorange', alpha=0.7)
    ax.set_xlabel('Joint Index')
    ax.set_ylabel('Max |Velocity| (rad/s)')
    ax.set_title('Maximum Absolute Velocities')
    ax.grid(True, alpha=0.3, axis='y')

    # Effort/torque ranges
    ax = axes[1, 0]
    eff_max = [np.max(np.abs(data[f'eff{j}'])) for j in joints]
    ax.bar(joints, eff_max, color='darkgreen', alpha=0.7)
    ax.set_xlabel('Joint Index')
    ax.set_ylabel('Max |Effort| (Nm)')
    ax.set_title('Maximum Absolute Efforts/Torques')
    ax.grid(True, alpha=0.3, axis='y')

    # Tracking errors
    ax = axes[1, 1]
    errors_mean_mid_q = [
        np.mean(np.abs(data[f'mid_q{j}'] - data[f'q{j}'])) for j in joints
    ]
    errors_max_mid_q = [
        np.max(np.abs(data[f'mid_q{j}'] - data[f'q{j}'])) for j in joints
    ]
    errors_mean_ik_q = [
        np.mean(np.abs(data[f'ik_q{j}'] - data[f'q{j}'])) for j in joints
    ]
    errors_max_ik_q = [
        np.max(np.abs(data[f'ik_q{j}'] - data[f'q{j}'])) for j in joints
    ]
    
    x = np.arange(len(joints))
    width = 0.2
    ax.bar(x - width * 1.5,
           errors_mean_mid_q,
           width,
           label='mid_q Mean',
           color='crimson',
           alpha=0.7)
    ax.bar(x - width / 2,
           errors_max_mid_q,
           width,
           label='mid_q Max',
           color='darkred',
           alpha=0.7)
    ax.bar(x + width / 2,
           errors_mean_ik_q,
           width,
           label='ik_q Mean',
           color='orange',
           alpha=0.7)
    ax.bar(x + width * 1.5,
           errors_max_ik_q,
           width,
           label='ik_q Max',
           color='darkorange',
           alpha=0.7)
    
    ax.set_xlabel('Joint Index')
    ax.set_ylabel('Tracking Error (rad)')
    ax.set_title('Tracking Errors (mid_q vs ik_q)')
    ax.set_xticks(x)
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    fig.suptitle('Summary Statistics', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig


def print_statistics(data):
    """Print statistical summary of the data."""
    print("\n" + "=" * 60)
    print("TRAJECTORY DATA STATISTICS")
    print("=" * 60)

    time = (data['ts'] - data['ts'][0]) / 1e9
    duration = time[-1]
    n_samples = len(data)
    avg_hz = n_samples / duration if duration > 0 else 0

    print(f"\nTime Information:")
    print(f"  Duration: {duration:.2f} seconds")
    print(f"  Samples: {n_samples}")
    print(f"  Average frequency: {avg_hz:.2f} Hz")

    print(f"\nJoint Statistics:")
    for j in range(7):
        print(f"\n  Joint {j}:")
        print(
            f"    Position range: [{np.min(data[f'q{j}']):.4f}, {np.max(data[f'q{j}']):.4f}] rad"
        )
        print(f"    Max velocity: {np.max(np.abs(data[f'dq{j}'])):.4f} rad/s")
        print(f"    Max effort: {np.max(np.abs(data[f'eff{j}'])):.4f} Nm")

        error_mid_q = data[f'mid_q{j}'] - data[f'q{j}']
        error_ik_q = data[f'ik_q{j}'] - data[f'q{j}']
        print(f"    Tracking error (mid_q) - Mean: {np.mean(np.abs(error_mid_q)):.6f} rad, "
              f"Max: {np.max(np.abs(error_mid_q)):.6f} rad")
        print(f"    Tracking error (ik_q)  - Mean: {np.mean(np.abs(error_ik_q)):.6f} rad, "
              f"Max: {np.max(np.abs(error_ik_q)):.6f} rad")

    print("\n" + "=" * 60 + "\n")


def main():
    # Load data
    data = load_data(CSV_FILE)

    # Print statistics
    print_statistics(data)

    # Create output directory if saving
    os.makedirs(PLOTS_DIR, exist_ok=True)
    print(f"Saving plots to: {PLOTS_DIR}")

    # Generate plots
    print("\nGenerating plots...")

    # Plot 1: Joint positions
    fig1 = plot_joint_positions(data)
    fig1.savefig(f"{PLOTS_DIR}/joint_positions.png",
                 dpi=150,
                 bbox_inches='tight')
    print("  Saved: joint_positions.png")

    # Plot 2: Joint velocities
    fig2 = plot_joint_velocities(data)
    fig2.savefig(f"{PLOTS_DIR}/joint_velocities.png",
                 dpi=150,
                 bbox_inches='tight')
    print("  Saved: joint_velocities.png")

    # Plot 3: Joint efforts
    fig3 = plot_joint_efforts(data)
    fig3.savefig(f"{PLOTS_DIR}/joint_efforts.png",
                 dpi=150,
                 bbox_inches='tight')
    print("  Saved: joint_efforts.png")

    # Plot 4: Tracking errors
    fig4 = plot_tracking_errors(data)
    fig4.savefig(f"{PLOTS_DIR}/tracking_errors.png",
                 dpi=150,
                 bbox_inches='tight')
    print("  Saved: tracking_errors.png")

    # Plot 5: Summary statistics (always for all joints)
    fig5 = plot_summary_statistics(data)
    fig5.savefig(f"{PLOTS_DIR}/summary_statistics.png",
                 dpi=150,
                 bbox_inches='tight')
    print("  Saved: summary_statistics.png")


if __name__ == '__main__':
    main()
