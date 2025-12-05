#!/usr/bin/env python3
"""
GLIM Stability Test Script
Run GLIM rosbag repeatedly for 5 hours and log all results
"""

import subprocess
import time
import os
import signal
import sys
from datetime import datetime, timedelta
import json

# Configuration
BAG_PATH = os.path.expanduser("~/Documents/bags/Pangyo_indoor_parking_lot_slam_2025_11_19-19_31_37")
TEST_DURATION_HOURS = 5
LOG_DIR = "/home/soslab/colcon_ws/src/glim_ros2/stability_test_logs"
COLCON_WS = "/home/soslab/colcon_ws"

# Results storage
results = {
    "start_time": None,
    "end_time": None,
    "total_runs": 0,
    "successful_runs": 0,
    "failed_runs": 0,
    "errors": [],
    "run_details": []
}

def get_timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")

def log_message(msg):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] {msg}")
    with open(os.path.join(LOG_DIR, "stability_test_main.log"), "a") as f:
        f.write(f"[{timestamp}] {msg}\n")

def run_glim_test(run_number):
    """Run a single GLIM test iteration"""
    run_start = datetime.now()
    log_file = os.path.join(LOG_DIR, f"run_{run_number:03d}_{get_timestamp()}.log")

    log_message(f"=== Starting Run #{run_number} ===")

    # Prepare command
    cmd = f"""
    cd {COLCON_WS} && \
    source install/setup.bash && \
    ros2 run glim_ros glim_rosbag {BAG_PATH}
    """

    try:
        # Run the command and capture output
        process = subprocess.Popen(
            cmd,
            shell=True,
            executable='/bin/bash',
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid
        )

        # Collect output
        output_lines = []
        error_detected = False
        error_messages = []

        while True:
            line = process.stdout.readline()
            if not line and process.poll() is not None:
                break
            if line:
                output_lines.append(line)
                # Print to console for real-time monitoring
                print(line, end='')

                # Check for errors
                lower_line = line.lower()
                if any(err in lower_line for err in ['error', 'exception', 'fault', 'segmentation', 'abort', 'crash', 'failed']):
                    if 'error' not in lower_line or ('error' in lower_line and 'warning' not in lower_line):
                        error_detected = True
                        error_messages.append(line.strip())

        return_code = process.wait()
        run_end = datetime.now()
        duration = (run_end - run_start).total_seconds()

        # Write log file
        with open(log_file, 'w') as f:
            f.write(f"Run #{run_number}\n")
            f.write(f"Start: {run_start}\n")
            f.write(f"End: {run_end}\n")
            f.write(f"Duration: {duration:.2f} seconds\n")
            f.write(f"Return Code: {return_code}\n")
            f.write(f"Error Detected: {error_detected}\n")
            f.write("="*80 + "\n")
            f.writelines(output_lines)

        # Determine success
        success = (return_code == 0 and not error_detected)

        run_result = {
            "run_number": run_number,
            "start_time": run_start.isoformat(),
            "end_time": run_end.isoformat(),
            "duration_seconds": duration,
            "return_code": return_code,
            "success": success,
            "error_detected": error_detected,
            "error_messages": error_messages if error_messages else None,
            "log_file": log_file
        }

        if success:
            log_message(f"Run #{run_number} completed successfully in {duration:.1f}s")
            results["successful_runs"] += 1
        else:
            log_message(f"Run #{run_number} FAILED (code: {return_code}, errors: {error_detected})")
            results["failed_runs"] += 1
            if error_messages:
                results["errors"].append({
                    "run_number": run_number,
                    "messages": error_messages
                })

        results["run_details"].append(run_result)
        return success, run_result

    except Exception as e:
        log_message(f"Exception during Run #{run_number}: {str(e)}")
        results["failed_runs"] += 1
        results["errors"].append({
            "run_number": run_number,
            "messages": [str(e)]
        })
        return False, None

def cleanup_processes():
    """Kill any remaining GLIM processes"""
    try:
        subprocess.run("pkill -9 -f 'glim_rosbag'", shell=True, stderr=subprocess.DEVNULL)
        subprocess.run("pkill -9 -f 'glim_ros'", shell=True, stderr=subprocess.DEVNULL)
        time.sleep(2)
    except:
        pass

def save_results():
    """Save results to JSON file"""
    results_file = os.path.join(LOG_DIR, f"stability_test_results_{get_timestamp()}.json")
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    log_message(f"Results saved to {results_file}")
    return results_file

def generate_report():
    """Generate final test report"""
    report_file = os.path.join(LOG_DIR, f"stability_test_report_{get_timestamp()}.txt")

    total_test_time = (datetime.fromisoformat(results["end_time"]) -
                       datetime.fromisoformat(results["start_time"])).total_seconds() / 3600

    with open(report_file, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("GLIM STABILITY TEST REPORT\n")
        f.write("=" * 80 + "\n\n")

        f.write(f"Test Start:     {results['start_time']}\n")
        f.write(f"Test End:       {results['end_time']}\n")
        f.write(f"Total Duration: {total_test_time:.2f} hours\n\n")

        f.write("-" * 40 + "\n")
        f.write("SUMMARY\n")
        f.write("-" * 40 + "\n")
        f.write(f"Total Runs:      {results['total_runs']}\n")
        f.write(f"Successful Runs: {results['successful_runs']}\n")
        f.write(f"Failed Runs:     {results['failed_runs']}\n")

        if results['total_runs'] > 0:
            success_rate = (results['successful_runs'] / results['total_runs']) * 100
            f.write(f"Success Rate:    {success_rate:.1f}%\n")
        f.write("\n")

        if results['errors']:
            f.write("-" * 40 + "\n")
            f.write("ERRORS ENCOUNTERED\n")
            f.write("-" * 40 + "\n")
            for error in results['errors']:
                f.write(f"\nRun #{error['run_number']}:\n")
                for msg in error['messages']:
                    f.write(f"  - {msg}\n")

        f.write("\n" + "-" * 40 + "\n")
        f.write("RUN DETAILS\n")
        f.write("-" * 40 + "\n")
        for run in results['run_details']:
            status = "OK" if run['success'] else "FAIL"
            f.write(f"Run #{run['run_number']:03d}: [{status}] {run['duration_seconds']:.1f}s\n")

        f.write("\n" + "=" * 80 + "\n")
        f.write("END OF REPORT\n")
        f.write("=" * 80 + "\n")

    log_message(f"Report saved to {report_file}")
    return report_file

def main():
    global results

    # Setup signal handler for graceful shutdown
    def signal_handler(sig, frame):
        log_message("Received interrupt signal, stopping tests...")
        results["end_time"] = datetime.now().isoformat()
        save_results()
        generate_report()
        cleanup_processes()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize
    os.makedirs(LOG_DIR, exist_ok=True)
    results["start_time"] = datetime.now().isoformat()
    end_time = datetime.now() + timedelta(hours=TEST_DURATION_HOURS)

    log_message("=" * 60)
    log_message("GLIM STABILITY TEST STARTED")
    log_message(f"Test will run for {TEST_DURATION_HOURS} hours")
    log_message(f"Bag file: {BAG_PATH}")
    log_message(f"Log directory: {LOG_DIR}")
    log_message("=" * 60)

    run_number = 0

    # Main test loop
    while datetime.now() < end_time:
        run_number += 1
        results["total_runs"] = run_number

        # Clean up before each run
        cleanup_processes()
        time.sleep(3)

        # Run test
        success, run_result = run_glim_test(run_number)

        # Brief pause between runs
        remaining = (end_time - datetime.now()).total_seconds()
        if remaining > 0:
            log_message(f"Time remaining: {remaining/3600:.2f} hours")
            time.sleep(5)

    # Finalize
    results["end_time"] = datetime.now().isoformat()
    cleanup_processes()

    log_message("=" * 60)
    log_message("STABILITY TEST COMPLETED")
    log_message("=" * 60)

    # Save and report
    save_results()
    report_file = generate_report()

    # Print summary
    print("\n" + "=" * 60)
    print("FINAL SUMMARY")
    print("=" * 60)
    print(f"Total Runs: {results['total_runs']}")
    print(f"Successful: {results['successful_runs']}")
    print(f"Failed: {results['failed_runs']}")
    if results['total_runs'] > 0:
        print(f"Success Rate: {(results['successful_runs']/results['total_runs'])*100:.1f}%")
    print(f"\nFull report: {report_file}")
    print("=" * 60)

if __name__ == "__main__":
    main()
