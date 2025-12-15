#pragma once

#include <deque>
#include <mutex>
#include <memory>

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
 * @brief FOV colorized data stored in EstimationFrame::custom_data
 * Key: "rgb_fov_data"
 */
struct FrameRGBFovData {
  std::vector<Eigen::Vector4d> points;  // FOV-only points in sensor frame
  std::vector<uint32_t> colors;         // packed RGB (0x00RRGGBB)
  Eigen::Isometry3d T_world_sensor;     // Sensor pose when captured
};

/**
 * @brief Colorized point cloud data (FOV-only points with RGB colors)
 */
struct ColorizedPointCloud {
  double stamp;
  std::string frame_id;
  std::vector<Eigen::Vector4d> points;
  std::vector<uint32_t> colors;  // packed RGB (0x00RRGGBB)
  Eigen::Isometry3d T_world_sensor;  // Sensor pose in world frame (for raw_map accumulation)
};

/**
 * @brief FOV-only submap data (submap origin frame)
 * Used for submap-based map publishing and saving
 */
struct ColorizedSubmap {
  size_t submap_id;                       // Submap index
  double stamp;                           // Representative timestamp for trajectory lookup
  Eigen::Isometry3d T_world_origin;       // Submap pose in world frame
  std::vector<Eigen::Vector4d> points;    // FOV-only points in submap origin frame
  std::vector<uint32_t> colors;           // packed RGB (0x00RRGGBB)
};

/**
 * @brief Callbacks for RGB colorizer events
 */
struct RGBColorizerCallbacks {
  /// @brief Called when a frame has been colorized (FOV-only points)
  static CallbackSlot<void(const ColorizedPointCloud&)> on_frame_colorized;

  /// @brief Called when a submap has been colorized (FOV-only points)
  static CallbackSlot<void(const ColorizedSubmap&)> on_submap_colorized;
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

  // Calibration (loads from config_sensors.json)
  bool load_calibration();

  // Colorization
  struct ColorizedPoints {
    std::vector<Eigen::Vector4d> points;
    std::vector<uint32_t> colors;  // packed RGB (0x00RRGGBB)
  };

  // Task for processing
  struct ColorizeTask {
    double stamp;
    double image_stamp;  // Timestamp of matched image (for motion compensation)
    PreprocessedFrame::ConstPtr raw_frame;  // Keep reference to avoid copying
    cv::Mat image;
    Eigen::Isometry3d T_camera_points;
    Eigen::Isometry3d T_lidar_imu;  // LiDAR-IMU transformation
    Eigen::Matrix<double, 8, Eigen::Dynamic> imu_rate_trajectory;  // IMU-rate trajectory [t, x, y, z, qx, qy, qz, qw]
    Eigen::Isometry3d T_world_sensor;  // World pose for map accumulation
  };

  // Synchronous processing - returns FOV data to be stored in frame's custom_data
  std::shared_ptr<FrameRGBFovData> process_colorization_sync(const ColorizeTask& task);

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

  // Submap voxel downsampling configuration
  double submap_voxel_resolution;  // voxel size for submap downsampling (0 = no downsampling)

  // LiDAR-Camera calibration
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  Eigen::Isometry3d T_camera_lidar;

  // Decoded image buffer (ready for colorization)
  std::mutex image_buffer_mutex;
  std::deque<std::pair<double, cv::Mat>> image_buffer;

  // ROS subscription
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;

  // Undistortion maps initialized flag
  bool undist_maps_initialized;
};

}  // namespace glim
