#!/usr/bin/env python3
"""
Trajectory analysis script to compare vibration/smoothness across different configurations.
Computes:
- Position jitter (high-frequency variations in x, y, z)
- Orientation jitter (high-frequency variations in quaternion)
- Velocity smoothness (acceleration spikes)
"""

import pandas as pd
import numpy as np
from pathlib import Path

def quaternion_angle_diff(q1, q2):
    """Compute angle difference between two quaternions in radians."""
    dot = np.abs(np.sum(q1 * q2))
    dot = np.clip(dot, -1.0, 1.0)
    return 2.0 * np.arccos(dot)

def compute_jitter_metrics(df):
    """Compute jitter metrics from trajectory dataframe."""
    # Remove duplicate timestamps
    df = df.drop_duplicates(subset='timestamp', keep='first').copy()
    df = df.sort_values('timestamp').reset_index(drop=True)

    # Position
    pos = df[['x', 'y', 'z']].values
    timestamps = df['timestamp'].values

    # Compute velocities
    dt = np.diff(timestamps)

    # Filter out very small dt (< 1ms)
    valid_mask = dt > 0.001

    pos_diff = np.diff(pos, axis=0)
    velocities = pos_diff[valid_mask] / dt[valid_mask, np.newaxis]

    # Compute accelerations (jitter indicator)
    dt2 = dt[:-1][valid_mask[:-1]]
    valid_mask2 = dt2 > 0.001

    acc_diff = np.diff(velocities, axis=0)
    accelerations = acc_diff[valid_mask2] / dt2[valid_mask2, np.newaxis]

    # Position jitter: std of second derivative
    pos_jitter = np.std(accelerations, axis=0)
    pos_jitter_total = np.linalg.norm(pos_jitter)

    # Orientation jitter
    quats = df[['qx', 'qy', 'qz', 'qw']].values
    angle_diffs = []
    for i in range(1, len(quats)):
        angle_diffs.append(quaternion_angle_diff(quats[i-1], quats[i]))
    angle_diffs = np.array(angle_diffs)

    # Angular velocity
    angular_velocities = angle_diffs[valid_mask] / dt[valid_mask]

    # Angular acceleration
    ang_diff = np.diff(angular_velocities)
    angular_accel = ang_diff[valid_mask2] / dt2[valid_mask2]

    orient_jitter = np.std(angular_accel)

    # High-frequency content analysis
    acc_variance = np.var(accelerations, axis=0)

    return {
        'pos_jitter_x': pos_jitter[0],
        'pos_jitter_y': pos_jitter[1],
        'pos_jitter_z': pos_jitter[2],
        'pos_jitter_total': pos_jitter_total,
        'orient_jitter_rad': orient_jitter,
        'orient_jitter_deg': np.degrees(orient_jitter),
        'acc_variance_x': acc_variance[0],
        'acc_variance_y': acc_variance[1],
        'acc_variance_z': acc_variance[2],
        'max_linear_accel': np.max(np.linalg.norm(accelerations, axis=1)),
        'max_angular_accel': np.max(np.abs(angular_accel)),
        'mean_linear_accel': np.mean(np.linalg.norm(accelerations, axis=1)),
        'mean_angular_accel': np.mean(np.abs(angular_accel)),
        'num_valid_samples': len(accelerations),
    }

def main():
    test_dir = Path('/home/q/colcon_ws/src/glim_ros2/test_results')

    files = {
        'CT Mode (LiDAR-only)': 'path_ct_mode.csv',
        'GPU+IMU (0.1/0.08)': 'path_gpu_imu_0.1_0.08.csv',
        'Z-Height Align': 'path_z_height_align.csv',
    }

    results = {}

    print("=" * 80)
    print("TRAJECTORY VIBRATION/JITTER ANALYSIS")
    print("=" * 80)
    print()

    for name, filename in files.items():
        filepath = test_dir / filename
        if not filepath.exists():
            print(f"[SKIP] {name}: File not found")
            continue

        df = pd.read_csv(filepath)
        print(f"[ANALYZING] {name}: {len(df)} poses")

        metrics = compute_jitter_metrics(df)
        results[name] = metrics
        print(f"  Valid acceleration samples: {metrics['num_valid_samples']}")

    print()
    print("=" * 80)
    print("RESULTS COMPARISON")
    print("=" * 80)
    print()

    # Print comparison table
    metric_names = [
        ('pos_jitter_total', 'Position Jitter (m/s²)', 'Lower is better'),
        ('orient_jitter_deg', 'Orientation Jitter (°/s²)', 'Lower is better'),
        ('max_linear_accel', 'Max Linear Accel (m/s²)', 'Lower is smoother'),
        ('max_angular_accel', 'Max Angular Accel (rad/s²)', 'Lower is smoother'),
        ('mean_linear_accel', 'Mean Linear Accel (m/s²)', 'Lower is smoother'),
        ('mean_angular_accel', 'Mean Angular Accel (rad/s²)', 'Lower is smoother'),
    ]

    print(f"{'Metric':<35} | ", end="")
    for name in results.keys():
        print(f"{name:<22} | ", end="")
    print()
    print("-" * 120)

    for metric_key, metric_name, note in metric_names:
        print(f"{metric_name:<35} | ", end="")
        values = []
        for name in results.keys():
            val = results[name][metric_key]
            values.append(val)
            print(f"{val:<22.4f} | ", end="")
        print()

        # Mark the best
        best_idx = np.argmin(values)
        best_name = list(results.keys())[best_idx]
        print(f"  → Best: {best_name} ({note})")
        print()

    print()
    print("=" * 80)
    print("DETAILED POSITION JITTER BREAKDOWN")
    print("=" * 80)
    print()

    for name, metrics in results.items():
        print(f"{name}:")
        print(f"  X-axis jitter: {metrics['pos_jitter_x']:.4f} m/s²")
        print(f"  Y-axis jitter: {metrics['pos_jitter_y']:.4f} m/s²")
        print(f"  Z-axis jitter: {metrics['pos_jitter_z']:.4f} m/s²")
        print()

    print()
    print("=" * 80)
    print("RECOMMENDATION")
    print("=" * 80)
    print()

    # Determine best config based on total jitter
    best_config = min(results.keys(), key=lambda k: results[k]['pos_jitter_total'])
    worst_config = max(results.keys(), key=lambda k: results[k]['pos_jitter_total'])

    print(f"Best configuration for minimal vibration: {best_config}")
    print(f"Worst configuration: {worst_config}")
    print()

    # Calculate improvement
    if len(results) >= 2:
        configs = list(results.keys())
        for i, config in enumerate(configs):
            for j, other in enumerate(configs):
                if i >= j:
                    continue
                val1 = results[config]['pos_jitter_total']
                val2 = results[other]['pos_jitter_total']
                diff = (val2 - val1) / val1 * 100
                if diff > 0:
                    print(f"{config} is {diff:.1f}% smoother than {other}")
                else:
                    print(f"{other} is {-diff:.1f}% smoother than {config}")

if __name__ == '__main__':
    main()
