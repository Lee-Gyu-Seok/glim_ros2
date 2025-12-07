#!/usr/bin/env python3
"""
Stability Test Script - 10 consecutive runs
Tests glim_rosbag for:
- No crashes (exit code 0)
- Position error < 3cm
- Orientation error < 1 degree

Note: IndeterminantLinearSystemException is a recoverable warning from ISAM2
optimization during early submaps. The system recovers automatically, so these
are logged but not counted as failures.
"""

import subprocess
import os
import time
import glob
import numpy as np
from scipy.spatial.transform import Rotation

# Configuration
BAG_PATH = "/home/soslab/Documents/bags/Pangyo_indoor_parking_lot_slam_2025_11_19-19_31_37"
COLCON_WS = "/home/soslab/colcon_ws"
GLIM_LOG_DIR = "/home/soslab/colcon_ws/src/glim_ros2/src/glim_ros/Log"
NUM_RUNS = 10
TIMEOUT_SECONDS = 600  # 10 minutes per run

# Criteria
MAX_POSITION_ERROR_M = 0.03  # 3cm
MAX_ORIENTATION_ERROR_DEG = 1.0  # 1 degree


def calculate_origin_return_error(traj_file):
    """Calculate position and orientation error between first and last pose."""
    try:
        with open(traj_file, 'r') as f:
            lines = f.readlines()

        if len(lines) < 2:
            return None, None, "Too few poses"

        # Parse first and last poses (TUM format: timestamp tx ty tz qx qy qz qw)
        first_parts = lines[0].strip().split()
        last_parts = lines[-1].strip().split()

        if len(first_parts) < 8 or len(last_parts) < 8:
            return None, None, "Invalid pose format"

        # Extract positions
        first_pos = np.array([float(first_parts[1]), float(first_parts[2]), float(first_parts[3])])
        last_pos = np.array([float(last_parts[1]), float(last_parts[2]), float(last_parts[3])])

        # Extract quaternions (qx, qy, qz, qw)
        first_quat = [float(first_parts[4]), float(first_parts[5]), float(first_parts[6]), float(first_parts[7])]
        last_quat = [float(last_parts[4]), float(last_parts[5]), float(last_parts[6]), float(last_parts[7])]

        # Calculate position error
        position_error = np.linalg.norm(last_pos - first_pos)

        # Calculate orientation error
        R1 = Rotation.from_quat(first_quat)
        R2 = Rotation.from_quat(last_quat)
        R_diff = R1.inv() * R2
        angle_error = np.degrees(np.abs(R_diff.magnitude()))

        return position_error, angle_error, None
    except Exception as e:
        return None, None, str(e)


def run_single_test(run_number):
    """Run a single test and return results."""
    print(f"\n{'='*60}")
    print(f"RUN {run_number}/{NUM_RUNS}")
    print(f"{'='*60}")

    # Clean up previous map directories in GLIM_LOG_DIR
    for map_dir in glob.glob(os.path.join(GLIM_LOG_DIR, "map_*")):
        subprocess.run(["rm", "-rf", map_dir], capture_output=True)

    log_file = f"/tmp/glim_stability_test_run{run_number}.log"

    # Run glim_rosbag
    cmd = f"cd {COLCON_WS} && source install/setup.bash && ros2 run glim_ros glim_rosbag {BAG_PATH} 2>&1 | tee {log_file}"

    print(f"Starting test run {run_number}...")
    start_time = time.time()

    try:
        result = subprocess.run(
            ["bash", "-c", cmd],
            timeout=TIMEOUT_SECONDS,
            capture_output=False
        )
        elapsed_time = time.time() - start_time
        exit_code = result.returncode
    except subprocess.TimeoutExpired:
        elapsed_time = TIMEOUT_SECONDS
        print(f"ERROR: Test timed out after {TIMEOUT_SECONDS} seconds")
        return {
            'run': run_number,
            'success': False,
            'exit_code': -1,
            'elapsed_time': elapsed_time,
            'error': 'Timeout',
            'indeterminant_errors': -1,
            'position_error': None,
            'orientation_error': None
        }

    # Check for IndeterminantLinearSystemException
    try:
        with open(log_file, 'r') as f:
            log_content = f.read()
        indeterminant_count = log_content.count('IndeterminantLinearSystemException')
    except:
        indeterminant_count = -1

    # Find trajectory file in GLIM_LOG_DIR
    time.sleep(2)  # Wait for file to be written
    traj_files = glob.glob(os.path.join(GLIM_LOG_DIR, "map_*/traj_lidar.txt"))

    position_error = None
    orientation_error = None
    traj_error = None

    if traj_files:
        traj_file = sorted(traj_files)[-1]  # Get the latest
        position_error, orientation_error, traj_error = calculate_origin_return_error(traj_file)
    else:
        traj_error = "No trajectory file found"

    # Determine success
    # Note: IndeterminantLinearSystemException is a recoverable warning from ISAM2
    # optimization. The system recovers automatically, so we only check for actual failures.
    success = (
        exit_code == 0 and
        position_error is not None and
        orientation_error is not None and
        position_error <= MAX_POSITION_ERROR_M and
        orientation_error <= MAX_ORIENTATION_ERROR_DEG
    )

    result = {
        'run': run_number,
        'success': success,
        'exit_code': exit_code,
        'elapsed_time': elapsed_time,
        'error': traj_error,
        'indeterminant_errors': indeterminant_count,
        'position_error': position_error,
        'orientation_error': orientation_error
    }

    # Print results
    print(f"\nRun {run_number} Results:")
    print(f"  Exit code: {exit_code}")
    print(f"  Elapsed time: {elapsed_time:.1f}s")
    print(f"  IndeterminantLinearSystemException count: {indeterminant_count} (info only, not failure criteria)")
    if position_error is not None:
        print(f"  Position error: {position_error*100:.2f} cm (target: <{MAX_POSITION_ERROR_M*100} cm)")
    else:
        print(f"  Position error: N/A ({traj_error})")
    if orientation_error is not None:
        print(f"  Orientation error: {orientation_error:.3f} deg (target: <{MAX_ORIENTATION_ERROR_DEG} deg)")
    else:
        print(f"  Orientation error: N/A")
    print(f"  SUCCESS: {success}")

    return result


def main():
    print("="*60)
    print("GLIM STABILITY TEST - 10 CONSECUTIVE RUNS")
    print("="*60)
    print(f"Bag: {BAG_PATH}")
    print(f"Target position error: < {MAX_POSITION_ERROR_M*100} cm")
    print(f"Target orientation error: < {MAX_ORIENTATION_ERROR_DEG} deg")
    print(f"Timeout per run: {TIMEOUT_SECONDS}s")
    print("="*60)

    results = []

    for i in range(1, NUM_RUNS + 1):
        result = run_single_test(i)
        results.append(result)

        # Print running summary
        successful = sum(1 for r in results if r['success'])
        print(f"\n[Progress: {i}/{NUM_RUNS}] Successful: {successful}/{i}")

        if not result['success']:
            print(f"WARNING: Run {i} failed!")

    # Final summary
    print("\n" + "="*60)
    print("FINAL SUMMARY")
    print("="*60)

    successful = sum(1 for r in results if r['success'])

    print(f"\nTotal runs: {NUM_RUNS}")
    print(f"Successful: {successful}")
    print(f"Failed: {NUM_RUNS - successful}")

    print(f"\nDetailed results:")
    print("-"*60)
    for r in results:
        status = "PASS" if r['success'] else "FAIL"
        pos_str = f"{r['position_error']*100:.2f}cm" if r['position_error'] else "N/A"
        ori_str = f"{r['orientation_error']:.3f}deg" if r['orientation_error'] else "N/A"
        print(f"  Run {r['run']}: {status} | Pos: {pos_str} | Ori: {ori_str} | IndeterminantErrors: {r['indeterminant_errors']}")

    # Calculate statistics
    pos_errors = [r['position_error'] for r in results if r['position_error'] is not None]
    ori_errors = [r['orientation_error'] for r in results if r['orientation_error'] is not None]

    if pos_errors:
        print(f"\nPosition error statistics:")
        print(f"  Mean: {np.mean(pos_errors)*100:.2f} cm")
        print(f"  Std:  {np.std(pos_errors)*100:.2f} cm")
        print(f"  Max:  {np.max(pos_errors)*100:.2f} cm")
        print(f"  Min:  {np.min(pos_errors)*100:.2f} cm")

    if ori_errors:
        print(f"\nOrientation error statistics:")
        print(f"  Mean: {np.mean(ori_errors):.3f} deg")
        print(f"  Std:  {np.std(ori_errors):.3f} deg")
        print(f"  Max:  {np.max(ori_errors):.3f} deg")
        print(f"  Min:  {np.min(ori_errors):.3f} deg")

    if successful == NUM_RUNS:
        print(f"\n*** ALL {NUM_RUNS} TESTS PASSED! ***")
        return 0
    else:
        print(f"\n*** FAILED: Only {successful}/{NUM_RUNS} tests passed ***")
        return 1


if __name__ == "__main__":
    exit(main())
