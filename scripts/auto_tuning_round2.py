#!/usr/bin/env python3
"""
Automated parameter tuning script for GLIM SLAM - Round 2
Based on Round 1 findings: denser keyframes work best
Target: Position error < 2cm, Rotation error < 1 degree
"""

import os
import sys
import json
import time
import subprocess
import shutil
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime
from pathlib import Path

# Unbuffered output
sys.stdout = sys.stderr = open(sys.stdout.fileno(), mode='w', buffering=1)

# Paths
COLCON_WS = "/home/soslab/colcon_ws"
CONFIG_PATH = f"{COLCON_WS}/src/glim_ros2/src/glim/config/presets/mlx"
INSTALL_CONFIG_PATH = f"{COLCON_WS}/install/glim/share/glim/config/presets/mlx"
BAG_PATH = "/home/soslab/Documents/bags/Pangyo_indoor_parking_lot_slam_2025_11_19-19_31_37"
LOG_PATH = f"{COLCON_WS}/src/glim_ros2/src/glim_ros/Log"
RESULTS_FILE = "/tmp/tuning_results_round2.csv"

def remove_json_comments(json_str):
    """Remove C-style comments from JSON string"""
    import re
    json_str = re.sub(r'//.*$', '', json_str, flags=re.MULTILINE)
    json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
    return json_str

def read_json_with_comments(filepath):
    """Read JSON file that may contain comments"""
    with open(filepath, 'r') as f:
        content = f.read()
    clean_content = remove_json_comments(content)
    return json.loads(clean_content)

def calculate_return_error(traj_file):
    """Calculate return-to-origin error from trajectory file"""
    try:
        with open(traj_file, 'r') as f:
            lines = f.readlines()

        if len(lines) < 10:
            return None, None

        first = lines[0].strip().split()
        last = lines[-1].strip().split()

        t0 = np.array([float(first[1]), float(first[2]), float(first[3])])
        q0 = np.array([float(first[4]), float(first[5]), float(first[6]), float(first[7])])

        t1 = np.array([float(last[1]), float(last[2]), float(last[3])])
        q1 = np.array([float(last[4]), float(last[5]), float(last[6]), float(last[7])])

        pos_error = np.linalg.norm(t1 - t0) * 100  # cm

        r0 = R.from_quat(q0)
        r1 = R.from_quat(q1)
        rot_diff = r0.inv() * r1
        rot_error = np.degrees(rot_diff.magnitude())

        return pos_error, rot_error
    except Exception as e:
        print(f"Error calculating return error: {e}")
        return None, None

def update_config(config_file, updates):
    """Update specific parameters in config file"""
    data = read_json_with_comments(config_file)
    main_key = list(data.keys())[0]

    for key, value in updates.items():
        data[main_key][key] = value

    with open(config_file, 'w') as f:
        json.dump(data, f, indent=2)

def copy_configs_to_install():
    """Copy config files from src to install directory"""
    files = [
        "config_global_mapping_cpu.json",
        "config_sub_mapping_cpu.json",
        "config_odometry_cpu.json",
        "config_scan_context.json"
    ]
    for f in files:
        src = os.path.join(CONFIG_PATH, f)
        dst = os.path.join(INSTALL_CONFIG_PATH, f)
        if os.path.exists(src):
            try:
                if os.path.samefile(src, dst):
                    continue
            except:
                pass
            if os.path.exists(dst):
                os.remove(dst)
            shutil.copy2(src, dst)

def run_glim_test(timeout=420):
    """Run GLIM and return the trajectory file path"""
    subprocess.run("pkill -9 -f glim_rosbag", shell=True, capture_output=True)
    time.sleep(2)

    existing_logs = set(os.listdir(LOG_PATH)) if os.path.exists(LOG_PATH) else set()

    cmd = f"cd {COLCON_WS} && source install/setup.bash && timeout {timeout} ros2 run glim_ros glim_rosbag {BAG_PATH}"

    start_time = time.time()
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, executable='/bin/bash')
    elapsed = time.time() - start_time

    time.sleep(3)

    if os.path.exists(LOG_PATH):
        new_logs = set(os.listdir(LOG_PATH)) - existing_logs
        if new_logs:
            latest_log = sorted(new_logs)[-1]
            traj_file = os.path.join(LOG_PATH, latest_log, "traj_lidar.txt")
            if os.path.exists(traj_file) and os.path.getsize(traj_file) > 100000:
                return traj_file, elapsed

    return None, elapsed

def log_result(params, pos_error, rot_error, elapsed, success):
    """Log result to CSV file"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if not os.path.exists(RESULTS_FILE):
        with open(RESULTS_FILE, 'w') as f:
            f.write("timestamp,pos_error_cm,rot_error_deg,elapsed_sec,success,params\n")

    with open(RESULTS_FILE, 'a') as f:
        params_str = str(params).replace(',', ';')
        f.write(f"{timestamp},{pos_error:.2f},{rot_error:.2f},{elapsed:.1f},{success},{params_str}\n")

def test_parameters(params_dict, test_name=""):
    """Test a specific parameter configuration"""
    print(f"\n{'='*60}")
    print(f"Testing: {test_name}")
    print(f"Parameters: {params_dict}")
    print(f"{'='*60}")

    if 'global' in params_dict:
        update_config(f"{CONFIG_PATH}/config_global_mapping_cpu.json", params_dict['global'])

    if 'sub' in params_dict:
        update_config(f"{CONFIG_PATH}/config_sub_mapping_cpu.json", params_dict['sub'])

    if 'odom' in params_dict:
        update_config(f"{CONFIG_PATH}/config_odometry_cpu.json", params_dict['odom'])

    if 'scan_context' in params_dict:
        update_config(f"{CONFIG_PATH}/config_scan_context.json", params_dict['scan_context'])

    copy_configs_to_install()

    traj_file, elapsed = run_glim_test()

    if traj_file:
        pos_error, rot_error = calculate_return_error(traj_file)
        if pos_error is not None:
            success = pos_error < 2.0 and rot_error < 1.0
            log_result(params_dict, pos_error, rot_error, elapsed, success)
            print(f"Result: Position Error = {pos_error:.2f} cm, Rotation Error = {rot_error:.2f} deg")
            print(f"Elapsed: {elapsed:.1f} sec")
            return pos_error, rot_error, elapsed

    log_result(params_dict, 999, 999, elapsed, False)
    print("Test failed - no valid trajectory")
    return 999, 999, elapsed

def main():
    print("="*60)
    print("GLIM Parameter Auto-Tuning - Round 2")
    print(f"Target: Position Error < 2cm, Rotation Error < 1 degree")
    print(f"Start time: {datetime.now()}")
    print("="*60)

    best_pos_error = 999
    best_rot_error = 999
    best_params = None

    # Round 2 parameter sets - based on Round 1 findings
    # Key insight: denser_keyframes (interval_trans=0.5, max_keyframes=15) was best at 4.75cm
    # Strategy: Try even denser keyframes and fine-tune loop closure

    parameter_sets = [
        # Ultra-dense keyframes
        {
            'name': 'ultra_dense_kf_0.3',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 20.0,
                'min_implicit_loop_overlap': 0.08,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.3,
                'max_num_keyframes': 20,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
        },

        # Very dense keyframes with fine voxel
        {
            'name': 'very_dense_kf_0.4',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 20.0,
                'min_implicit_loop_overlap': 0.08,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.4,
                'max_num_keyframes': 18,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
        },

        # Dense keyframes + more loop candidates
        {
            'name': 'dense_kf_more_loops',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 25.0,
                'min_implicit_loop_overlap': 0.06,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
        },

        # Dense keyframes + lower overlap threshold
        {
            'name': 'dense_kf_low_overlap',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 20.0,
                'min_implicit_loop_overlap': 0.05,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
        },

        # Dense keyframes with max overlap check
        {
            'name': 'dense_kf_max_overlap_0.6',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
                'max_keyframe_overlap': 0.6,
            },
        },

        # Denser keyframes + higher levels
        {
            'name': 'dense_kf_5_levels',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 5,
                'max_implicit_loop_distance': 20.0,
                'min_implicit_loop_overlap': 0.08,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 5,
            },
        },

        # Extreme dense keyframes
        {
            'name': 'extreme_dense_kf_0.25',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.25,
                'max_num_keyframes': 25,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
        },

        # Dense keyframes + scan context tuning
        {
            'name': 'dense_kf_sc_tuned',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 20.0,
                'min_implicit_loop_overlap': 0.08,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
            'scan_context': {
                'sc_dist_threshold': 0.35,
                'inlier_fraction_threshold': 0.65,
                'num_candidates_from_tree': 12,
            },
        },

        # Best combo: dense + fine voxel (0.08) + moderate loop
        {
            'name': 'optimal_combo_v1',
            'global': {
                'submap_voxel_resolution': 0.08,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 18.0,
                'min_implicit_loop_overlap': 0.10,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.08,
                'keyframe_voxelmap_levels': 4,
                'submap_voxel_resolution': 0.08,
                'submap_downsample_resolution': 0.08,
            },
        },

        # Optimal combo v2 with more keyframes
        {
            'name': 'optimal_combo_v2',
            'global': {
                'submap_voxel_resolution': 0.08,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 18.0,
                'min_implicit_loop_overlap': 0.10,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.4,
                'max_num_keyframes': 20,
                'keyframe_voxel_resolution': 0.08,
                'keyframe_voxelmap_levels': 4,
                'submap_voxel_resolution': 0.08,
                'submap_downsample_resolution': 0.08,
            },
        },

        # Dense keyframes with tighter keyframe update rotation
        {
            'name': 'dense_kf_rot_0.5',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'keyframe_update_interval_rot': 0.5,  # Radians (~28 degrees)
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
        },

        # Dense + more aggressive loop detection
        {
            'name': 'dense_aggressive_loop',
            'global': {
                'submap_voxel_resolution': 0.10,
                'submap_voxelmap_levels': 4,
                'max_implicit_loop_distance': 30.0,
                'min_implicit_loop_overlap': 0.05,
            },
            'sub': {
                'keyframe_update_interval_trans': 0.5,
                'max_num_keyframes': 15,
                'keyframe_voxel_resolution': 0.10,
                'keyframe_voxelmap_levels': 4,
            },
            'scan_context': {
                'sc_dist_threshold': 0.45,
                'inlier_fraction_threshold': 0.55,
                'num_candidates_from_tree': 18,
            },
        },
    ]

    for params in parameter_sets:
        name = params.pop('name')
        pos_err, rot_err, elapsed = test_parameters(params, name)

        if pos_err < best_pos_error or (pos_err == best_pos_error and rot_err < best_rot_error):
            best_pos_error = pos_err
            best_rot_error = rot_err
            best_params = {'name': name, **params}

            print(f"\n*** NEW BEST: pos={pos_err:.2f}cm, rot={rot_err:.2f}deg ***\n")

        if pos_err < 2.0 and rot_err < 1.0:
            print(f"\n{'='*60}")
            print("TARGET ACHIEVED!")
            print(f"Position Error: {pos_err:.2f} cm (target: <2cm)")
            print(f"Rotation Error: {rot_err:.2f} deg (target: <1deg)")
            print(f"Parameters: {params}")
            print(f"{'='*60}")

    print(f"\n{'='*60}")
    print("Round 2 Tuning Complete")
    print(f"Best Result: pos={best_pos_error:.2f}cm, rot={best_rot_error:.2f}deg")
    print(f"Best Parameters: {best_params}")
    print(f"Results saved to: {RESULTS_FILE}")
    print(f"End time: {datetime.now()}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
