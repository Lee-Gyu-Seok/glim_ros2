#pragma once

#include <any>
#include <atomic>
#include <thread>
#include <chrono>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;
struct ColorizedPointCloud;
struct ColorizedSubmap;

/**
 * @brief Rviz-based viewer
 *
 * This module publishes point clouds and poses to ROS topics for RViz visualization.
 * RGB colorization is handled by librgb_colorizer_ros.so module, which stores colors
 * as "rgb_colors" aux_attribute on point clouds. This viewer reads those colors
 * and includes them in the published PointCloud2 messages.
 */
class RvizViewer : public ExtensionModuleROS2 {
public:
  RvizViewer();
  ~RvizViewer();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;
  virtual void at_exit(const std::string& dump_path) override;

private:
  void set_callbacks();
  void odometry_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected);
  void publish_colored_points(const ColorizedPointCloud& colorized);
  void publish_non_colorized_points(const EstimationFrame::ConstPtr& new_frame);
  void publish_rgb_map();
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void on_submap_colorized(const ColorizedSubmap& submap);
  void invoke(const std::function<void()>& task);
  void save_map_pcd(const std::string& dump_path);

  void spin_once();

  // Convert point cloud with colors to PointCloud2 message
  sensor_msgs::msg::PointCloud2::UniquePtr frame_to_colored_pointcloud2(
    const std::string& frame_id, double stamp,
    const gtsam_points::PointCloud& frame, const std::vector<uint32_t>& colors);

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Time last_globalmap_pub_time;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  bool publish_imu2lidar;
  double tf_time_offset;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_pub;
  nav_msgs::msg::Path path_msg;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_points::PointCloud::ConstPtr> submaps;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // RGB colorizer integration
  bool rgb_colorizer_enabled;  // Whether RGB colorizer is active

  // FOV-only RGB submaps - updated via on_submap_colorized callback
  struct FovSubmap {
    double stamp;                         // Representative timestamp for trajectory lookup
    std::vector<Eigen::Vector4d> points;  // FOV-only points in submap origin frame
    std::vector<uint32_t> colors;         // RGB colors
    Eigen::Isometry3d T_world_origin;     // Submap pose (updated on globalmap update)
  };
  std::mutex fov_submaps_mutex;
  std::vector<FovSubmap> fov_submaps;  // FOV-only submap list
  bool fov_submaps_updated;

  // Raw point cloud accumulation (full resolution, no submap downsampling)
  // Points stored in sensor frame with timestamp for later reprojection using optimized trajectory
  struct RawFrame {
    double stamp;                         // Timestamp for trajectory lookup
    std::vector<Eigen::Vector4d> points;  // Points in sensor frame (NOT world frame)
    std::vector<uint32_t> colors;         // RGB colors (if available)
  };
  std::mutex raw_frames_mutex;
  std::vector<RawFrame> raw_frames;  // Accumulated raw frames

  // Trajectory loading for optimized reprojection
  std::map<double, Eigen::Isometry3d> load_trajectory(const std::string& traj_file);
  Eigen::Isometry3d interpolate_pose(const std::map<double, Eigen::Isometry3d>& traj, double stamp);

  // Map saving
  std::string map_save_path;
  double map_downsample_resolution;  // Voxel resolution for downsampled_map.pcd (0 = disable)

  // Logging
  std::shared_ptr<spdlog::logger> logger;
};
}  // namespace glim