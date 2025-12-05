#pragma once

#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>

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

  // Async processing threads
  void processing_thread_func();
  void image_decode_thread_func();

  // Calibration
  bool load_calibration(const std::string& filepath);

  // Colorization
  struct ColorizedPoints {
    std::vector<Eigen::Vector4d> points;
    std::vector<uint32_t> colors;  // packed RGB (0x00RRGGBB)
  };

  // Task for async processing
  struct ColorizeTask {
    double stamp;
    double image_stamp;  // Timestamp of matched image (for motion compensation)
    PreprocessedFrame::ConstPtr raw_frame;  // Keep reference to avoid copying
    cv::Mat image;
    Eigen::Isometry3d T_camera_points;
    Eigen::Isometry3d T_lidar_imu;  // LiDAR-IMU transformation
    Eigen::Matrix<double, 8, Eigen::Dynamic> imu_rate_trajectory;  // IMU-rate trajectory [t, x, y, z, qx, qy, qz, qw]
  };

  // Colorize only FOV-visible points (with optional motion compensation)
  ColorizedPoints colorize_points_fov_only(
    const Eigen::Vector4d* points,
    int num_points,
    const cv::Mat& image,
    const Eigen::Isometry3d& T_camera_points,
    const ColorizeTask* task = nullptr);  // Optional task for motion compensation

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

  // Z-buffer occlusion handling configuration
  bool enable_z_buffer;
  std::string splat_size_mode;  // "fixed", "adaptive", or "physical"
  int fixed_splat_size;
  double lidar_angular_resolution;  // degrees, for "physical" mode

  // Image undistortion configuration
  bool enable_undistort;  // undistort image before colorization (better quality)
  cv::Mat undist_map1;    // pre-computed undistortion map X
  cv::Mat undist_map2;    // pre-computed undistortion map Y

  // Motion compensation configuration
  bool enable_motion_compensation;  // compensate for LiDAR-camera time difference

  // LiDAR-Camera calibration
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  Eigen::Isometry3d T_camera_lidar;

  // Decoded image buffer (ready for colorization)
  std::mutex image_buffer_mutex;
  std::deque<std::pair<double, cv::Mat>> image_buffer;

  // Raw compressed image buffer (for async decoding)
  struct RawImageData {
    double stamp;
    std::vector<uint8_t> data;
  };
  std::mutex raw_image_mutex;
  std::condition_variable raw_image_cv;
  std::deque<RawImageData> raw_image_queue;

  // ROS subscription
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;

  // Async processing
  std::atomic<bool> kill_switch;
  std::thread processing_thread;
  std::thread image_decode_thread;  // Async image decoding
  std::mutex task_queue_mutex;
  std::condition_variable task_cv;
  std::deque<ColorizeTask> task_queue;
};

}  // namespace glim
