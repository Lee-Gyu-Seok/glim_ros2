#pragma once

#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/core.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/util/callback_slot.hpp>

namespace spdlog {
class logger;
}

namespace glim {

/**
 * @brief Colorized point cloud data (FOV-only points with RGB colors)
 */
struct ColorizedPointCloud {
  double stamp;
  std::string frame_id;
  std::vector<Eigen::Vector4d> points;
  std::vector<uint32_t> colors;  // packed RGB (0x00RRGGBB)
};

/**
 * @brief Callbacks for RGB colorizer events
 */
struct RGBColorizerCallbacks {
  /// @brief Called when a frame has been colorized (FOV-only points)
  static CallbackSlot<void(const ColorizedPointCloud&)> on_frame_colorized;
};

/**
 * @brief RGB colorizer module for ROS2
 *
 * This module subscribes to camera image topic and colorizes point clouds
 * based on LiDAR-Camera calibration. Colors are stored as frame attributes
 * that can be used by other modules (e.g., rviz_viewer).
 */
class RGBColorizerROS : public ExtensionModuleROS2 {
public:
  RGBColorizerROS();
  ~RGBColorizerROS();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  void set_callbacks();

  // Callback handlers
  void on_new_frame(const EstimationFrame::ConstPtr& frame);
  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  // Calibration
  bool load_calibration(const std::string& filepath);

  // Colorization
  struct ColorizedPoints {
    std::vector<Eigen::Vector4d> points;
    std::vector<uint32_t> colors;  // packed RGB (0x00RRGGBB)
  };

  // Colorize only FOV-visible points
  ColorizedPoints colorize_points_fov_only(
    const Eigen::Vector4d* points,
    int num_points,
    const cv::Mat& image,
    const Eigen::Isometry3d& T_camera_points);

  // Colorize all points (FOV outside = gray)
  std::vector<uint32_t> colorize_points_all(
    const Eigen::Vector4d* points,
    int num_points,
    const cv::Mat& image,
    const Eigen::Isometry3d& T_camera_points);

private:
  std::shared_ptr<spdlog::logger> logger;

  // Configuration
  bool enabled;
  std::string image_topic;
  double sync_tolerance;
  size_t max_buffer_size;

  // LiDAR-Camera calibration
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  Eigen::Isometry3d T_camera_lidar;

  // Image buffer
  std::mutex image_buffer_mutex;
  std::deque<std::pair<double, cv::Mat>> image_buffer;

  // ROS subscription
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;
};

}  // namespace glim
