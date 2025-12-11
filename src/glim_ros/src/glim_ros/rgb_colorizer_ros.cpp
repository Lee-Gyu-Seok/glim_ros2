#include <glim_ros/rgb_colorizer_ros.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/callbacks.hpp>
#include <unordered_map>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>

namespace glim {

// Bilinear interpolation for sub-pixel color sampling
// Returns BGR color sampled at sub-pixel coordinates (uf, vf)
inline cv::Vec3b bilinear_sample(const cv::Mat& image, float uf, float vf) {
  const int u0 = static_cast<int>(std::floor(uf));
  const int v0 = static_cast<int>(std::floor(vf));
  const int u1 = u0 + 1;
  const int v1 = v0 + 1;

  // Ensure we're within bounds for bilinear (need u1, v1 to be valid)
  const int max_u = image.cols - 1;
  const int max_v = image.rows - 1;

  // Clamp to valid range
  const int u0_clamped = std::max(0, std::min(u0, max_u));
  const int v0_clamped = std::max(0, std::min(v0, max_v));
  const int u1_clamped = std::max(0, std::min(u1, max_u));
  const int v1_clamped = std::max(0, std::min(v1, max_v));

  // Compute interpolation weights
  const float du = uf - u0;
  const float dv = vf - v0;
  const float w00 = (1.0f - du) * (1.0f - dv);
  const float w01 = (1.0f - du) * dv;
  const float w10 = du * (1.0f - dv);
  const float w11 = du * dv;

  // Sample 4 neighboring pixels
  const cv::Vec3b& p00 = image.at<cv::Vec3b>(v0_clamped, u0_clamped);
  const cv::Vec3b& p01 = image.at<cv::Vec3b>(v1_clamped, u0_clamped);
  const cv::Vec3b& p10 = image.at<cv::Vec3b>(v0_clamped, u1_clamped);
  const cv::Vec3b& p11 = image.at<cv::Vec3b>(v1_clamped, u1_clamped);

  // Weighted average for each channel
  cv::Vec3b result;
  for (int c = 0; c < 3; c++) {
    float val = w00 * p00[c] + w01 * p01[c] + w10 * p10[c] + w11 * p11[c];
    result[c] = static_cast<uchar>(std::min(255.0f, std::max(0.0f, val + 0.5f)));
  }
  return result;
}

// Define the static callback slots
CallbackSlot<void(const ColorizedPointCloud&)> RGBColorizerCallbacks::on_frame_colorized;
CallbackSlot<void(const ColorizedMap&)> RGBColorizerCallbacks::on_map_updated;
CallbackSlot<void(const ColorizedSubmap&)> RGBColorizerCallbacks::on_submap_colorized;

RGBColorizerROS::RGBColorizerROS() : logger(create_module_logger("rgb_color")), kill_switch(false) {
  logger->info("starting RGB colorizer ROS module");

  const Config config(GlobalConfig::get_config_path("config_ros"));

  enabled = config.param<bool>("glim_ros", "rgb_colorizer_enabled", false);
  image_topic = config.param<std::string>("glim_ros", "rgb_image_topic", "/image/compressed");
  sync_tolerance = config.param<double>("glim_ros", "rgb_sync_tolerance", 0.1);
  max_buffer_size = config.param<int>("glim_ros", "rgb_max_buffer_size", 30);

  // Z-buffer occlusion handling settings
  enable_z_buffer = config.param<bool>("glim_ros", "rgb_enable_z_buffer", true);
  splat_size_mode = config.param<std::string>("glim_ros", "rgb_splat_size_mode", "physical");
  fixed_splat_size = config.param<int>("glim_ros", "rgb_fixed_splat_size", 3);
  lidar_angular_resolution = config.param<double>("glim_ros", "rgb_lidar_angular_resolution", 0.2);  // degrees
  logger->info("Z-buffer occlusion handling: {} (mode: {}, fixed_size: {}, angular_res: {}°)",
               enable_z_buffer ? "enabled" : "disabled", splat_size_mode, fixed_splat_size, lidar_angular_resolution);

  // Image undistortion settings (for better color quality)
  enable_undistort = config.param<bool>("glim_ros", "rgb_enable_undistort", true);
  logger->info("Image undistortion: {}", enable_undistort ? "enabled" : "disabled");

  // Motion compensation settings (compensate for LiDAR-camera time difference)
  enable_motion_compensation = config.param<bool>("glim_ros", "rgb_enable_motion_compensation", true);
  logger->info("Motion compensation: {}", enable_motion_compensation ? "enabled" : "disabled");

  // Submap voxel downsampling settings - use GLIM's existing submap_downsample_resolution
  // This ensures RGB FOV submaps have the same density as original GLIM submaps
  {
    // Try GPU config first, then CPU config
    std::string sub_mapping_config;
    try {
      sub_mapping_config = GlobalConfig::get_config_path("config_sub_mapping_gpu");
      Config sub_mapping(sub_mapping_config);
      submap_voxel_resolution = sub_mapping.param<double>("sub_mapping", "submap_downsample_resolution", 0.08);
    } catch (...) {
      try {
        sub_mapping_config = GlobalConfig::get_config_path("config_sub_mapping_cpu");
        Config sub_mapping(sub_mapping_config);
        submap_voxel_resolution = sub_mapping.param<double>("sub_mapping", "submap_downsample_resolution", 0.08);
      } catch (...) {
        submap_voxel_resolution = 0.08;  // Default fallback
      }
    }
    logger->info("Submap voxel resolution: {}m (from GLIM sub_mapping config)", submap_voxel_resolution);
  }

  if (enabled) {
    if (load_calibration()) {
      logger->info("RGB colorizer enabled with calibration from config_sensors.json");
    } else {
      logger->warn("RGB colorizer: Failed to load calibration from config_sensors.json, disabling");
      enabled = false;
    }
  } else {
    logger->info("RGB colorizer is disabled");
  }

  if (enabled) {
    set_callbacks();
    // Start async processing threads
    processing_thread = std::thread(&RGBColorizerROS::processing_thread_func, this);
    image_decode_thread = std::thread(&RGBColorizerROS::image_decode_thread_func, this);
    logger->info("Started async colorization and image decode threads");
  }

  logger->info("ready");
}

RGBColorizerROS::~RGBColorizerROS() {
  // Stop async processing threads
  kill_switch = true;
  task_cv.notify_all();
  raw_image_cv.notify_all();

  if (processing_thread.joinable()) {
    processing_thread.join();
  }
  if (image_decode_thread.joinable()) {
    image_decode_thread.join();
  }
}

std::vector<GenericTopicSubscription::Ptr> RGBColorizerROS::create_subscriptions(rclcpp::Node& node) {
  if (!enabled) {
    return {};
  }

  rclcpp::QoS image_qos(10);
  image_qos.best_effort();
  image_sub = node.create_subscription<sensor_msgs::msg::CompressedImage>(
    image_topic, image_qos,
    std::bind(&RGBColorizerROS::image_callback, this, std::placeholders::_1));
  logger->info("Subscribed to image topic: {}", image_topic);

  return {};
}

void RGBColorizerROS::set_callbacks() {
  using std::placeholders::_1;
  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& frame) {
    on_new_frame(frame);
  });
  GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) {
    on_update_submaps(submaps);
  });
}

bool RGBColorizerROS::load_calibration() {
  // Load calibration from config_sensors.json
  try {
    const Config sensors_config(GlobalConfig::get_config_path("config_sensors"));

    // Check if camera parameters are available in config_sensors.json
    auto intrinsics = sensors_config.param<std::vector<double>>("sensors", "intrinsics", std::vector<double>());
    auto T_lidar_camera_vec = sensors_config.param<std::vector<double>>("sensors", "T_lidar_camera", std::vector<double>());
    auto dist_vec = sensors_config.param<std::vector<double>>("sensors", "distortion_coeffs", std::vector<double>());

    if (intrinsics.size() != 4) {
      logger->error("Invalid intrinsics in config_sensors.json (expected 4 values [fx, fy, cx, cy], got {})", intrinsics.size());
      return false;
    }
    if (T_lidar_camera_vec.size() != 7) {
      logger->error("Invalid T_lidar_camera in config_sensors.json (expected 7 values [tx, ty, tz, qx, qy, qz, qw], got {})", T_lidar_camera_vec.size());
      return false;
    }
    if (dist_vec.size() < 5) {
      logger->error("Invalid distortion_coeffs in config_sensors.json (expected at least 5 values, got {})", dist_vec.size());
      return false;
    }

    // Camera intrinsics: [fx, fy, cx, cy]
    double fx = intrinsics[0];
    double fy = intrinsics[1];
    double cx = intrinsics[2];
    double cy = intrinsics[3];
    camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Distortion coefficients: [k1, k2, p1, p2, k3] (plumb_bob format)
    double k1 = dist_vec[0];
    double k2 = dist_vec[1];
    double p1 = dist_vec[2];
    double p2 = dist_vec[3];
    double k3 = dist_vec.size() > 4 ? dist_vec[4] : 0.0;
    dist_coeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

    // T_lidar_camera: LiDAR -> Camera transformation
    // Format: [tx, ty, tz, qx, qy, qz, qw] (TUM format)
    double tx = T_lidar_camera_vec[0];
    double ty = T_lidar_camera_vec[1];
    double tz = T_lidar_camera_vec[2];
    double qx = T_lidar_camera_vec[3];
    double qy = T_lidar_camera_vec[4];
    double qz = T_lidar_camera_vec[5];
    double qw = T_lidar_camera_vec[6];

    Eigen::Quaterniond q(qw, qx, qy, qz);
    T_camera_lidar = Eigen::Isometry3d::Identity();
    T_camera_lidar.linear() = q.toRotationMatrix();
    T_camera_lidar.translation() = Eigen::Vector3d(tx, ty, tz);

    logger->info("Loaded calibration from config_sensors.json:");
    logger->info("  intrinsics: fx={:.2f}, fy={:.2f}, cx={:.2f}, cy={:.2f}", fx, fy, cx, cy);
    logger->info("  distortion: k1={:.5f}, k2={:.5f}, p1={:.5f}, p2={:.5f}, k3={:.5f}", k1, k2, p1, p2, k3);
    logger->info("  T_lidar_camera: t=[{:.4f}, {:.4f}, {:.4f}], q=[{:.4f}, {:.4f}, {:.4f}, {:.4f}]",
                 tx, ty, tz, qx, qy, qz, qw);
    return true;
  } catch (const std::exception& e) {
    logger->error("Failed to load calibration from config_sensors.json: {}", e.what());
    return false;
  }
}

void RGBColorizerROS::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  // Non-blocking: just queue raw data for async decoding
  double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  {
    std::lock_guard<std::mutex> lock(raw_image_mutex);

    // Drop old images if queue is getting too large
    while (raw_image_queue.size() >= max_buffer_size) {
      raw_image_queue.pop_front();
    }

    RawImageData raw_data;
    raw_data.stamp = stamp;
    raw_data.data = msg->data;  // Copy compressed data (small)
    raw_image_queue.push_back(std::move(raw_data));
  }
  raw_image_cv.notify_one();
}

void RGBColorizerROS::image_decode_thread_func() {
  logger->info("Image decode thread started");

  // Flag to initialize undistortion maps on first image
  bool undist_maps_initialized = false;

  while (!kill_switch) {
    RawImageData raw_data;

    // Wait for raw image data
    {
      std::unique_lock<std::mutex> lock(raw_image_mutex);
      raw_image_cv.wait(lock, [this]() {
        return !raw_image_queue.empty() || kill_switch;
      });

      if (kill_switch) {
        break;
      }

      raw_data = std::move(raw_image_queue.front());
      raw_image_queue.pop_front();
    }

    // Decode image (slow operation, done in this separate thread)
    try {
      cv::Mat image = cv::imdecode(cv::Mat(raw_data.data), cv::IMREAD_COLOR);
      if (image.empty()) {
        continue;
      }

      // Initialize undistortion maps on first valid image (if enabled)
      if (enable_undistort && !undist_maps_initialized) {
        cv::Size img_size(image.cols, image.rows);
        // Use same camera matrix for new output (no scaling)
        cv::initUndistortRectifyMap(
          camera_matrix, dist_coeffs, cv::Mat(),
          camera_matrix, img_size, CV_32FC1,
          undist_map1, undist_map2);
        logger->info("Initialized undistortion maps for {}x{} images", image.cols, image.rows);
        undist_maps_initialized = true;
      }

      // Apply undistortion if enabled
      cv::Mat final_image;
      if (enable_undistort && undist_maps_initialized) {
        cv::remap(image, final_image, undist_map1, undist_map2, cv::INTER_LINEAR);
      } else {
        final_image = image;
      }

      // Add to decoded image buffer
      {
        std::lock_guard<std::mutex> lock(image_buffer_mutex);
        image_buffer.emplace_back(raw_data.stamp, final_image);

        while (image_buffer.size() > max_buffer_size) {
          image_buffer.pop_front();
        }
      }
    } catch (const std::exception& e) {
      logger->warn("Failed to decode image: {}", e.what());
    }
  }

  logger->info("Image decode thread stopped");
}

void RGBColorizerROS::on_new_frame(const EstimationFrame::ConstPtr& frame) {
  if (!enabled) {
    return;
  }

  // Check if raw_frame and raw_points are available
  if (!frame->raw_frame || !frame->raw_frame->raw_points ||
      frame->raw_frame->raw_points->points.empty()) {
    return;
  }

  // Find matching image for this frame
  // Strategy: Use the most recent image that is not newer than the frame timestamp
  // This works for both real-time and rosbag playback scenarios
  cv::Mat matched_image;
  double matched_image_stamp = 0;
  double matched_dt = 0;
  {
    std::lock_guard<std::mutex> lock(image_buffer_mutex);

    // Find the latest image with timestamp <= frame->stamp + sync_tolerance
    for (auto it = image_buffer.rbegin(); it != image_buffer.rend(); ++it) {
      double img_stamp = it->first;
      double dt = frame->stamp - img_stamp;  // Positive if image is older than frame

      // Accept images that are up to sync_tolerance older, or slightly newer (within tolerance)
      if (dt >= -sync_tolerance) {
        matched_image = it->second;
        matched_image_stamp = img_stamp;  // Store image timestamp for motion compensation
        matched_dt = dt;
        break;  // Found the most recent valid image
      }
    }
  }

  // Statistics counters (static to persist across calls)
  static int total_frames_received = 0;
  static int frames_no_image_match = 0;
  static int frames_queued = 0;
  total_frames_received++;

  if (matched_image.empty()) {
    frames_no_image_match++;
    // Log occasionally to avoid spam
    if (frames_no_image_match % 100 == 0) {
      std::lock_guard<std::mutex> lock(image_buffer_mutex);
      if (!image_buffer.empty()) {
        double min_stamp = image_buffer.front().first;
        double max_stamp = image_buffer.back().first;
        logger->warn("no matching image: frame_stamp={}, buffer_range=[{}, {}], stats: received={}, no_match={}, queued={}",
          frame->stamp, min_stamp, max_stamp, total_frames_received, frames_no_image_match, frames_queued);
      } else {
        logger->warn("no matching image (buffer empty): received={}, no_match={}, queued={}",
          total_frames_received, frames_no_image_match, frames_queued);
      }
    }
    return;
  }
  frames_queued++;

  // Create async task - non-blocking to avoid affecting SLAM timing
  // Keep reference to raw_frame to avoid copying points
  ColorizeTask task;
  task.stamp = frame->stamp;
  task.image_stamp = matched_image_stamp;  // Store image timestamp for motion compensation
  task.raw_frame = frame->raw_frame;  // Just increment reference count, no copy
  task.image = matched_image;  // Shallow copy (reference counted)
  task.T_camera_points = T_camera_lidar;
  task.T_lidar_imu = frame->T_lidar_imu;  // LiDAR-IMU transformation
  task.imu_rate_trajectory = frame->imu_rate_trajectory;  // IMU-rate trajectory for motion compensation
  task.T_world_sensor = frame->T_world_sensor();  // World pose for map accumulation

  // Add to queue, drop old tasks if queue is full (max 20 to prevent delay buildup)
  {
    std::lock_guard<std::mutex> lock(task_queue_mutex);
    // Drop old tasks if queue is getting too large
    while (task_queue.size() >= 20) {
      task_queue.pop_front();
    }
    task_queue.push_back(std::move(task));
  }
  task_cv.notify_one();
}

void RGBColorizerROS::processing_thread_func() {
  logger->info("Colorization processing thread started");

  while (!kill_switch) {
    ColorizeTask task;

    // Wait for a task
    {
      std::unique_lock<std::mutex> lock(task_queue_mutex);
      task_cv.wait(lock, [this]() {
        return !task_queue.empty() || kill_switch;
      });

      if (kill_switch) {
        break;
      }

      task = std::move(task_queue.front());
      task_queue.pop_front();
    }

    // Process the task (colorization)
    // Access raw_points through the raw_frame reference (no copy)
    const auto& raw_points = task.raw_frame->raw_points->points;

    ColorizedPoints result = colorize_points_fov_only(
      raw_points.data(), raw_points.size(), task.image, task.T_camera_points, &task);

    if (result.points.empty()) {
      logger->debug("no FOV points for frame at stamp {}", task.stamp);
      continue;
    }

    // Create ColorizedPointCloud and notify subscribers
    ColorizedPointCloud colorized;
    colorized.stamp = task.stamp;
    colorized.frame_id = "lidar";
    colorized.points = std::move(result.points);
    colorized.colors = std::move(result.colors);
    colorized.T_world_sensor = task.T_world_sensor;

    logger->debug("colored frame with {} FOV points", colorized.points.size());
    RGBColorizerCallbacks::on_frame_colorized(colorized);

    // Store per-frame FOV data for later submap construction
    {
      std::lock_guard<std::mutex> lock(frame_fov_buffer_mutex);

      FrameFovData fov_data;
      fov_data.points = colorized.points;  // Points in sensor frame
      fov_data.colors = colorized.colors;
      fov_data.T_world_sensor = task.T_world_sensor;

      frame_fov_buffer.emplace_back(task.stamp, std::move(fov_data));

      // Limit buffer size
      while (frame_fov_buffer.size() > MAX_FRAME_FOV_BUFFER_SIZE) {
        frame_fov_buffer.pop_front();
      }
    }
  }

  logger->info("Colorization processing thread stopped");
}

void RGBColorizerROS::on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  if (!enabled) {
    return;
  }

  // Build submap FOV from stored per-frame FOV data
  // This avoids the image buffer timing issue by using already-colorized frame data
  const SubMap::ConstPtr latest_submap = submaps.back();
  const size_t submap_id = submaps.size() - 1;

  // Get timestamp range for frames in this submap
  if (latest_submap->odom_frames.empty()) {
    logger->warn("Submap {} has no odom_frames", submap_id);
    return;
  }

  const double stamp_start = latest_submap->odom_frames.front()->stamp;
  const double stamp_end = latest_submap->odom_frames.back()->stamp;

  // Submap origin pose (T_world_origin)
  const Eigen::Isometry3d T_world_origin = latest_submap->T_world_origin;
  const Eigen::Isometry3d T_origin_world = T_world_origin.inverse();

  // Collect FOV points from all frames in this submap's time range
  std::vector<Eigen::Vector4d> submap_points;
  std::vector<uint32_t> submap_colors;

  {
    std::lock_guard<std::mutex> lock(frame_fov_buffer_mutex);

    // Find frames within the submap's time range (with small tolerance)
    const double tolerance = 0.01;  // 10ms tolerance

    for (const auto& [stamp, fov_data] : frame_fov_buffer) {
      if (stamp >= stamp_start - tolerance && stamp <= stamp_end + tolerance) {
        // Transform points from sensor frame to submap origin frame
        // p_origin = T_origin_world * T_world_sensor * p_sensor
        const Eigen::Isometry3d T_origin_sensor = T_origin_world * fov_data.T_world_sensor;

        for (size_t i = 0; i < fov_data.points.size(); i++) {
          Eigen::Vector4d p_origin = T_origin_sensor * fov_data.points[i];
          p_origin.w() = 1.0;  // Ensure homogeneous coordinate
          submap_points.push_back(p_origin);
          submap_colors.push_back(fov_data.colors[i]);
        }
      }
    }
  }

  if (submap_points.empty()) {
    // Check if we have any frame data at all
    std::lock_guard<std::mutex> lock(frame_fov_buffer_mutex);
    if (frame_fov_buffer.empty()) {
      logger->warn("No FOV frames for submap {} (stamp range [{:.3f}, {:.3f}], frame buffer is EMPTY)",
                   submap_id, stamp_start, stamp_end);
    } else {
      double oldest = frame_fov_buffer.front().first;
      double newest = frame_fov_buffer.back().first;
      logger->warn("No FOV frames for submap {} (stamp range [{:.3f}, {:.3f}], buffer range [{:.3f}, {:.3f}])",
                   submap_id, stamp_start, stamp_end, oldest, newest);
    }
    return;
  }

  const size_t raw_count = submap_points.size();

  // Apply voxel downsampling if enabled
  if (submap_voxel_resolution > 0) {
    // Hash function for voxel grid key (3D integer coordinates)
    struct VoxelKey {
      int64_t x, y, z;
      bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
      }
    };
    struct VoxelKeyHash {
      size_t operator()(const VoxelKey& k) const {
        // Combine hashes using prime numbers
        size_t h = std::hash<int64_t>()(k.x);
        h ^= std::hash<int64_t>()(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
        h ^= std::hash<int64_t>()(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
      }
    };

    // Voxel accumulator: stores sum of positions and colors for averaging
    struct VoxelAccum {
      double sum_x = 0, sum_y = 0, sum_z = 0;
      uint64_t sum_r = 0, sum_g = 0, sum_b = 0;
      size_t count = 0;
    };

    std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxel_map;
    const double inv_res = 1.0 / submap_voxel_resolution;

    // Accumulate points into voxels
    for (size_t i = 0; i < submap_points.size(); i++) {
      const Eigen::Vector4d& p = submap_points[i];
      const uint32_t color = submap_colors[i];

      VoxelKey key;
      key.x = static_cast<int64_t>(std::floor(p.x() * inv_res));
      key.y = static_cast<int64_t>(std::floor(p.y() * inv_res));
      key.z = static_cast<int64_t>(std::floor(p.z() * inv_res));

      auto& accum = voxel_map[key];
      accum.sum_x += p.x();
      accum.sum_y += p.y();
      accum.sum_z += p.z();
      accum.sum_r += (color >> 16) & 0xFF;
      accum.sum_g += (color >> 8) & 0xFF;
      accum.sum_b += color & 0xFF;
      accum.count++;
    }

    // Build downsampled point cloud
    std::vector<Eigen::Vector4d> downsampled_points;
    std::vector<uint32_t> downsampled_colors;
    downsampled_points.reserve(voxel_map.size());
    downsampled_colors.reserve(voxel_map.size());

    for (const auto& [key, accum] : voxel_map) {
      Eigen::Vector4d avg_point;
      avg_point.x() = accum.sum_x / accum.count;
      avg_point.y() = accum.sum_y / accum.count;
      avg_point.z() = accum.sum_z / accum.count;
      avg_point.w() = 1.0;

      uint8_t avg_r = static_cast<uint8_t>(accum.sum_r / accum.count);
      uint8_t avg_g = static_cast<uint8_t>(accum.sum_g / accum.count);
      uint8_t avg_b = static_cast<uint8_t>(accum.sum_b / accum.count);
      uint32_t avg_color = (static_cast<uint32_t>(avg_r) << 16) |
                           (static_cast<uint32_t>(avg_g) << 8) |
                           static_cast<uint32_t>(avg_b);

      downsampled_points.push_back(avg_point);
      downsampled_colors.push_back(avg_color);
    }

    submap_points = std::move(downsampled_points);
    submap_colors = std::move(downsampled_colors);
  }

  // Send FOV-only colorized submap to RvizViewer via callback
  ColorizedSubmap colorized_submap;
  colorized_submap.submap_id = submap_id;
  colorized_submap.stamp = (stamp_start + stamp_end) / 2.0;  // Use middle timestamp for trajectory lookup
  colorized_submap.T_world_origin = T_world_origin;
  colorized_submap.points = std::move(submap_points);
  colorized_submap.colors = std::move(submap_colors);

  RGBColorizerCallbacks::on_submap_colorized(colorized_submap);
}

// Helper function to interpolate IMU pose from imu_rate_trajectory
// imu_rate_trajectory format: 8 x N matrix with [t, x, y, z, qx, qy, qz, qw] per column
static Eigen::Isometry3d interpolate_imu_pose(
  const Eigen::Matrix<double, 8, Eigen::Dynamic>& trajectory,
  double query_time) {

  if (trajectory.cols() == 0) {
    return Eigen::Isometry3d::Identity();
  }

  // Binary search for the right interval
  int left = 0;
  int right = trajectory.cols() - 1;

  // Handle edge cases
  if (query_time <= trajectory(0, left)) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = trajectory.block<3, 1>(1, left);
    Eigen::Quaterniond q(trajectory(7, left), trajectory(4, left), trajectory(5, left), trajectory(6, left));
    pose.linear() = q.toRotationMatrix();
    return pose;
  }
  if (query_time >= trajectory(0, right)) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = trajectory.block<3, 1>(1, right);
    Eigen::Quaterniond q(trajectory(7, right), trajectory(4, right), trajectory(5, right), trajectory(6, right));
    pose.linear() = q.toRotationMatrix();
    return pose;
  }

  // Binary search
  while (right - left > 1) {
    int mid = (left + right) / 2;
    if (trajectory(0, mid) <= query_time) {
      left = mid;
    } else {
      right = mid;
    }
  }

  // Linear interpolation between left and right
  double t0 = trajectory(0, left);
  double t1 = trajectory(0, right);
  double alpha = (query_time - t0) / (t1 - t0);

  // Interpolate translation
  Eigen::Vector3d trans0 = trajectory.block<3, 1>(1, left);
  Eigen::Vector3d trans1 = trajectory.block<3, 1>(1, right);
  Eigen::Vector3d trans = (1.0 - alpha) * trans0 + alpha * trans1;

  // Interpolate rotation (SLERP)
  Eigen::Quaterniond q0(trajectory(7, left), trajectory(4, left), trajectory(5, left), trajectory(6, left));
  Eigen::Quaterniond q1(trajectory(7, right), trajectory(4, right), trajectory(5, right), trajectory(6, right));
  Eigen::Quaterniond q = q0.slerp(alpha, q1);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = trans;
  pose.linear() = q.toRotationMatrix();
  return pose;
}

// FOV-only version: returns only points inside camera FOV with colors
// Optimized version with OpenMP parallel processing and Z-buffer occlusion handling
RGBColorizerROS::ColorizedPoints RGBColorizerROS::colorize_points_fov_only(
  const Eigen::Vector4d* points, int num_points,
  const cv::Mat& image, const Eigen::Isometry3d& T_camera_points,
  const ColorizeTask* task) {

  ColorizedPoints result;

  if (image.empty() || num_points == 0) {
    return result;
  }

  // Motion compensation setup
  // If enabled, we transform each LiDAR point to the camera's coordinate frame at the image capture time
  // This corrects for the time difference between when each LiDAR point was captured and when the image was taken
  const bool do_motion_comp = enable_motion_compensation && task != nullptr &&
                               task->imu_rate_trajectory.cols() > 0 &&
                               task->raw_frame != nullptr &&
                               !task->raw_frame->times.empty();

  // Pre-compute transforms for motion compensation
  Eigen::Isometry3d T_lidar_imu = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_world_imu_at_image = Eigen::Isometry3d::Identity();
  if (do_motion_comp) {
    T_lidar_imu = task->T_lidar_imu;
    T_imu_lidar = T_lidar_imu.inverse();
    // Get IMU pose at image capture time
    T_world_imu_at_image = interpolate_imu_pose(task->imu_rate_trajectory, task->image_stamp);
  }

  // Pre-extract camera intrinsics for direct projection (avoid OpenCV overhead)
  const double fx = camera_matrix.at<double>(0, 0);
  const double fy = camera_matrix.at<double>(1, 1);
  const double cx = camera_matrix.at<double>(0, 2);
  const double cy = camera_matrix.at<double>(1, 2);

  // Distortion coefficients (only needed when image is not undistorted)
  const double k1 = dist_coeffs.at<double>(0);
  const double k2 = dist_coeffs.at<double>(1);
  const double p1 = dist_coeffs.at<double>(2);
  const double p2 = dist_coeffs.at<double>(3);
  const double k3 = dist_coeffs.at<double>(4);

  // Check if we're using undistorted images (skip distortion calculation)
  const bool use_pinhole = enable_undistort && !undist_map1.empty();

  const int img_width = image.cols;
  const int img_height = image.rows;

  // Pre-allocate arrays for parallel processing
  // Each element stores: valid flag, point index, u, v coordinates (sub-pixel), depth
  std::vector<int> valid_indices(num_points, -1);  // -1 = invalid
  std::vector<float> pixel_u(num_points);  // Sub-pixel precision for bilinear interpolation
  std::vector<float> pixel_v(num_points);
  std::vector<float> depths(num_points);

  // Get point timestamps for motion compensation (if available)
  const std::vector<double>* point_times = nullptr;
  if (do_motion_comp && task != nullptr && task->raw_frame != nullptr) {
    point_times = &(task->raw_frame->times);
  }

  // Parallel projection calculation
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int i = 0; i < num_points; i++) {
    // Get the raw LiDAR point in LiDAR frame
    Eigen::Vector3d p_lidar = points[i].head<3>();

    // Motion compensation: Transform point to the LiDAR pose at image capture time
    // This ensures each point is projected from the correct viewpoint
    if (do_motion_comp && point_times != nullptr && static_cast<size_t>(i) < point_times->size()) {
      // Absolute time when this point was captured
      double point_abs_time = task->stamp + (*point_times)[i];

      // Get IMU pose at point capture time
      Eigen::Isometry3d T_world_imu_at_point = interpolate_imu_pose(task->imu_rate_trajectory, point_abs_time);

      // Transform: p_lidar (at point time) -> p_world -> p_lidar (at image time)
      // p_world = T_world_imu_at_point * T_imu_lidar * p_lidar
      // p_lidar_at_image = T_lidar_imu * T_imu_world_at_image * p_world
      //                  = T_lidar_imu * T_imu_world_at_image * T_world_imu_at_point * T_imu_lidar * p_lidar

      Eigen::Isometry3d T_imu_world_at_image = T_world_imu_at_image.inverse();
      Eigen::Isometry3d T_lidar_at_image_to_lidar_at_point = T_lidar_imu * T_imu_world_at_image * T_world_imu_at_point * T_imu_lidar;

      // Apply the motion compensation transform
      p_lidar = T_lidar_at_image_to_lidar_at_point.inverse() * p_lidar;
    }

    // Transform point to camera frame (T_camera_lidar already applied)
    const Eigen::Vector3d p_cam = T_camera_points * p_lidar;

    // Skip points behind camera (z <= 0.1m)
    if (p_cam.z() <= 0.1) {
      continue;
    }

    // Store depth for Z-buffer
    depths[i] = static_cast<float>(p_cam.z());

    // Normalized image coordinates
    const double x = p_cam.x() / p_cam.z();
    const double y = p_cam.y() / p_cam.z();

    float u, v;
    if (use_pinhole) {
      // Simple pinhole projection for undistorted images
      // (Image has been undistorted, so no need to apply distortion to points)
      u = static_cast<float>(fx * x + cx);
      v = static_cast<float>(fy * y + cy);
    } else {
      // Apply radial and tangential distortion for original (distorted) images
      const double r2 = x * x + y * y;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;

      const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      const double x_distorted = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double y_distorted = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

      // Project to pixel coordinates (sub-pixel precision for bilinear interpolation)
      u = static_cast<float>(fx * x_distorted + cx);
      v = static_cast<float>(fy * y_distorted + cy);
    }

    // Check if within image bounds (with margin for bilinear interpolation)
    // Need at least 0.5 pixel margin for bilinear sampling
    if (u >= 0.0f && u < static_cast<float>(img_width - 1) &&
        v >= 0.0f && v < static_cast<float>(img_height - 1)) {
      valid_indices[i] = i;
      pixel_u[i] = u;
      pixel_v[i] = v;
    }
  }

  // Z-buffer based occlusion handling
  if (enable_z_buffer) {
    // Initialize Z-buffer with max depth
    cv::Mat z_buffer(img_height, img_width, CV_32F, cv::Scalar(std::numeric_limits<float>::max()));
    cv::Mat point_index_buffer(img_height, img_width, CV_32S, cv::Scalar(-1));

    // Calculate splat sizes based on mode
    // For physical mode, pre-compute the constant splat size based on LiDAR angular resolution
    int physical_splat_size = 4;  // default fallback
    if (splat_size_mode == "physical") {
      // Physical calculation: angular_res (rad) * focal_length = pixels between LiDAR points
      // splat_size = 2 * angular_res * focal * margin (to ensure overlap)
      const double angular_res_rad = lidar_angular_resolution * M_PI / 180.0;
      const double avg_focal = (fx + fy) / 2.0;
      // Use 2x margin to ensure good coverage (accounts for point jitter, calibration error)
      physical_splat_size = static_cast<int>(std::ceil(angular_res_rad * avg_focal * 2.0));
      physical_splat_size = std::max(2, std::min(physical_splat_size, 12));  // clamp 2~12
    }

    auto get_splat_size = [this, physical_splat_size](float depth) -> int {
      if (splat_size_mode == "fixed") {
        return fixed_splat_size;
      } else if (splat_size_mode == "physical") {
        // Physical mode: constant splat size based on LiDAR angular resolution
        // (거리와 무관하게 일정 - 각해상도가 일정한 LiDAR 특성)
        return physical_splat_size;
      }
      // Adaptive mode (legacy, depth-based heuristic)
      if (!std::isfinite(depth) || depth <= 0) return 3;
      if (depth < 5.0f) return 2;   // Near: 2x2
      if (depth < 20.0f) return 4;  // Mid: 4x4
      if (depth < 50.0f) return 6;  // Far: 6x6
      return 8;                     // Very far: 8x8
    };

    // Pass 1: Fill Z-buffer with splatting
    for (int i = 0; i < num_points; i++) {
      if (valid_indices[i] < 0) continue;

      const int u = static_cast<int>(pixel_u[i]);  // Integer pixel for Z-buffer
      const int v = static_cast<int>(pixel_v[i]);
      const float depth = depths[i];
      const int splat_size = get_splat_size(depth);
      const int half_size = splat_size / 2;

      // Calculate splat region bounds
      const int y_min = std::max(0, v - half_size);
      const int y_max = std::min(img_height, v + half_size + 1);
      const int x_min = std::max(0, u - half_size);
      const int x_max = std::min(img_width, u + half_size + 1);

      // Apply Z-buffer test to splat region
      for (int py = y_min; py < y_max; ++py) {
        for (int px = x_min; px < x_max; ++px) {
          if (depth < z_buffer.at<float>(py, px)) {
            z_buffer.at<float>(py, px) = depth;
            point_index_buffer.at<int>(py, px) = i;
          }
        }
      }
    }

    // Pass 2: Collect visible points (those that won Z-buffer test at their center pixel)
    std::vector<bool> visible(num_points, false);
    for (int i = 0; i < num_points; i++) {
      if (valid_indices[i] < 0) continue;

      const int u = static_cast<int>(pixel_u[i]);  // Integer pixel for Z-buffer lookup
      const int v = static_cast<int>(pixel_v[i]);

      // A point is visible if it's the closest point at its projected pixel
      // or within a small tolerance (to handle ties)
      const float depth = depths[i];
      const float z_at_pixel = z_buffer.at<float>(v, u);
      const float tolerance = 0.05f;  // 5cm tolerance for depth comparison

      if (std::abs(depth - z_at_pixel) <= tolerance) {
        visible[i] = true;
      }
    }

    // Count visible points for memory allocation
    int visible_count = 0;
    for (int i = 0; i < num_points; i++) {
      if (visible[i]) visible_count++;
    }

    // Pre-allocate result vectors
    result.points.reserve(visible_count);
    result.colors.reserve(visible_count);

    // Extract colors only for visible points (using bilinear interpolation for sub-pixel precision)
    for (int i = 0; i < num_points; i++) {
      if (visible[i]) {
        // Use bilinear interpolation for smoother color sampling
        const cv::Vec3b bgr = bilinear_sample(image, pixel_u[i], pixel_v[i]);
        const uint32_t color = (static_cast<uint32_t>(bgr[2]) << 16) |  // R
                               (static_cast<uint32_t>(bgr[1]) << 8) |   // G
                               static_cast<uint32_t>(bgr[0]);           // B

        result.points.push_back(points[i]);
        result.colors.push_back(color);
      }
    }
  } else {
    // Original behavior without Z-buffer (for backwards compatibility)
    // Count valid points first (for efficient memory allocation)
    int valid_count = 0;
    for (int i = 0; i < num_points; i++) {
      if (valid_indices[i] >= 0) {
        valid_count++;
      }
    }

    // Pre-allocate result vectors
    result.points.reserve(valid_count);
    result.colors.reserve(valid_count);

    // Sequential color extraction with bilinear interpolation for sub-pixel precision
    for (int i = 0; i < num_points; i++) {
      if (valid_indices[i] >= 0) {
        // Use bilinear interpolation for smoother color sampling
        const cv::Vec3b bgr = bilinear_sample(image, pixel_u[i], pixel_v[i]);
        const uint32_t color = (static_cast<uint32_t>(bgr[2]) << 16) |  // R
                               (static_cast<uint32_t>(bgr[1]) << 8) |   // G
                               static_cast<uint32_t>(bgr[0]);           // B

        result.points.push_back(points[i]);
        result.colors.push_back(color);
      }
    }
  }

  return result;
}

// All-points version: returns colors for all points (gray for FOV outside)
std::vector<uint32_t> RGBColorizerROS::colorize_points_all(
  const Eigen::Vector4d* points, int num_points,
  const cv::Mat& image, const Eigen::Isometry3d& T_camera_points) {

  std::vector<uint32_t> colors(num_points, 0x808080);  // Default gray

  if (image.empty() || num_points == 0) {
    return colors;
  }

  // Transform points to camera frame and project
  std::vector<cv::Point3d> points_cam;
  std::vector<int> valid_indices;
  points_cam.reserve(num_points);
  valid_indices.reserve(num_points);

  for (int i = 0; i < num_points; i++) {
    Eigen::Vector3d p_cam = T_camera_points * points[i].head<3>();
    if (p_cam.z() > 0.1) {  // Only points in front of camera
      points_cam.emplace_back(p_cam.x(), p_cam.y(), p_cam.z());
      valid_indices.push_back(i);
    }
  }

  if (points_cam.empty()) {
    return colors;
  }

  // Project points to image
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(points_cam, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                    camera_matrix, dist_coeffs, image_points);

  // Extract colors using bilinear interpolation for sub-pixel precision
  for (size_t i = 0; i < valid_indices.size(); i++) {
    float uf = static_cast<float>(image_points[i].x);
    float vf = static_cast<float>(image_points[i].y);

    // Check bounds with margin for bilinear sampling
    if (uf >= 0.0f && uf < static_cast<float>(image.cols - 1) &&
        vf >= 0.0f && vf < static_cast<float>(image.rows - 1)) {
      cv::Vec3b bgr = bilinear_sample(image, uf, vf);
      uint8_t r = bgr[2];  // R from BGR
      uint8_t g = bgr[1];  // G
      uint8_t b = bgr[0];  // B from BGR
      colors[valid_indices[i]] = (static_cast<uint32_t>(r) << 16) |
                                  (static_cast<uint32_t>(g) << 8) |
                                  static_cast<uint32_t>(b);
    }
  }

  return colors;
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RGBColorizerROS();
}
