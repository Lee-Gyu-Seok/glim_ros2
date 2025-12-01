#!/usr/bin/env python3
"""
Analyze trajectory loop closure error.
Compares first and last poses to evaluate loop closure accuracy.
"""

import sys
import numpy as np
from pathlib import Path
import glob

def quaternion_to_euler(qx, qy, qz, qw):
    """Convert quaternion to euler angles (roll, pitch, yaw) in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def quaternion_angle_diff(q1, q2):
    """Calculate angle difference between two quaternions in degrees."""
    # q1, q2: [qx, qy, qz, qw]
    dot = abs(np.dot(q1, q2))
    dot = min(1.0, dot)  # Clamp for numerical stability
    angle_rad = 2 * np.arccos(dot)
    return np.degrees(angle_rad)

def load_trajectory(filepath):
    """Load TUM format trajectory file."""
    data = np.loadtxt(filepath)
    return data

def analyze_loop_closure(traj_file):
    """Analyze loop closure error from trajectory file."""
    traj = load_trajectory(traj_file)

    if len(traj) < 2:
        print("Error: Trajectory too short")
        return None

    # First and last poses
    first = traj[0]
    last = traj[-1]

    # Position: x, y, z (columns 1, 2, 3)
    pos_first = first[1:4]
    pos_last = last[1:4]

    # Quaternion: qx, qy, qz, qw (columns 4, 5, 6, 7)
    quat_first = first[4:8]
    quat_last = last[4:8]

    # Calculate errors
    pos_error = np.linalg.norm(pos_last - pos_first)
    pos_error_xyz = pos_last - pos_first

    rot_error = quaternion_angle_diff(quat_first, quat_last)

    # Euler angles for reference
    euler_first = quaternion_to_euler(*quat_first)
    euler_last = quaternion_to_euler(*quat_last)
    euler_diff = np.array(euler_last) - np.array(euler_first)

    # Wrap yaw difference to [-180, 180]
    for i in range(3):
        while euler_diff[i] > 180:
            euler_diff[i] -= 360
        while euler_diff[i] < -180:
            euler_diff[i] += 360

    return {
        'total_frames': len(traj),
        'duration': last[0] - first[0],
        'pos_error': pos_error,
        'pos_error_xyz': pos_error_xyz,
        'rot_error': rot_error,
        'euler_first': euler_first,
        'euler_last': euler_last,
        'euler_diff': euler_diff,
        'pos_first': pos_first,
        'pos_last': pos_last,
    }

def find_latest_log_dir():
    """Find the latest glim log directory."""
    log_base = Path.home() / "colcon_ws/src/glim_ros2/src/glim_ros/Log"
    if not log_base.exists():
        # Try alternative path
        log_base = Path("/home/soslab/colcon_ws/src/glim_ros2/src/glim_ros/Log")

    if not log_base.exists():
        return None

    # Find directories starting with 'map_'
    map_dirs = sorted(log_base.glob("map_*"), key=lambda x: x.stat().st_mtime, reverse=True)

    if not map_dirs:
        return None

    return map_dirs[0]

def main():
    if len(sys.argv) > 1:
        traj_file = sys.argv[1]
    else:
        # Find latest log directory
        log_dir = find_latest_log_dir()
        if log_dir is None:
            print("Error: No log directory found")
            print("Usage: python analyze_loop_closure.py [traj_lidar.txt]")
            sys.exit(1)

        traj_file = log_dir / "traj_lidar.txt"
        if not traj_file.exists():
            traj_file = log_dir / "traj_imu.txt"

    if not Path(traj_file).exists():
        print(f"Error: File not found: {traj_file}")
        sys.exit(1)

    print(f"Analyzing: {traj_file}")
    print("=" * 60)

    result = analyze_loop_closure(traj_file)

    if result is None:
        sys.exit(1)

    print(f"Total frames: {result['total_frames']}")
    print(f"Duration: {result['duration']:.2f} sec")
    print()
    print("=== Loop Closure Error ===")
    print(f"Position Error: {result['pos_error']*100:.2f} cm ({result['pos_error']:.4f} m)")
    print(f"  X: {result['pos_error_xyz'][0]*100:+.2f} cm")
    print(f"  Y: {result['pos_error_xyz'][1]*100:+.2f} cm")
    print(f"  Z: {result['pos_error_xyz'][2]*100:+.2f} cm")
    print()
    print(f"Rotation Error: {result['rot_error']:.4f} deg")
    print(f"  Roll diff:  {result['euler_diff'][0]:+.4f} deg")
    print(f"  Pitch diff: {result['euler_diff'][1]:+.4f} deg")
    print(f"  Yaw diff:   {result['euler_diff'][2]:+.4f} deg")
    print()

    # Check against targets
    pos_target = 0.02  # 2cm
    rot_target = 1.0   # 1 degree

    pos_ok = result['pos_error'] <= pos_target
    rot_ok = result['rot_error'] <= rot_target

    print("=== Target Check ===")
    print(f"Position: {'PASS' if pos_ok else 'FAIL'} (target: {pos_target*100:.0f} cm)")
    print(f"Rotation: {'PASS' if rot_ok else 'FAIL'} (target: {rot_target:.1f} deg)")

    if pos_ok and rot_ok:
        print("\n*** ALL TARGETS MET! ***")

    return result

if __name__ == "__main__":
    main()
