#!/usr/bin/env python3
"""Save path from /glim_ros/path topic to CSV file - auto-saves periodically"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import csv
import sys
import signal

class PathSaver(Node):
    def __init__(self, output_file):
        super().__init__('path_saver')
        self.output_file = output_file
        self.poses = []
        self.last_save_count = 0
        self.subscription = self.create_subscription(
            Path,
            '/glim_ros/path',
            self.path_callback,
            10)
        self.get_logger().info(f'Saving path to {output_file}')

        # Auto-save timer every 3 seconds
        self.timer = self.create_timer(3.0, self.auto_save)

    def path_callback(self, msg):
        # Store all poses from the path
        self.poses = []
        for pose_stamped in msg.poses:
            t = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
            p = pose_stamped.pose.position
            q = pose_stamped.pose.orientation
            self.poses.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def auto_save(self):
        if len(self.poses) > self.last_save_count:
            self.save()
            self.last_save_count = len(self.poses)

    def save(self):
        if self.poses:
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
                writer.writerows(self.poses)
            self.get_logger().info(f'Saved {len(self.poses)} poses to {self.output_file}')

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 save_path.py <output_file.csv>")
        sys.exit(1)

    rclpy.init()
    node = PathSaver(sys.argv[1])

    def signal_handler(sig, frame):
        node.save()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.save()

if __name__ == '__main__':
    main()
