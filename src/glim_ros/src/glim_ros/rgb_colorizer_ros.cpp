#include <glim_ros/rgb_colorizer_ros.hpp>

#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>

namespace glim {

// Define the static callback slot
CallbackSlot<void(const ColorizedPointCloud&)> RGBColorizerCallbacks::on_frame_colorized;

RGBColorizerROS::RGBColorizerROS() : logger(create_module_logger("rgb_color")) {
  logger->info("starting RGB colorizer ROS module");

  const Config config(GlobalConfig::get_config_path("config_ros"));

  enabled = config.param<bool>("glim_ros", "rgb_colorizer_enabled", false);
  image_topic = config.param<std::string>("glim_ros", "rgb_image_topic", "/image/compressed");
  sync_tolerance = config.param<double>("glim_ros", "rgb_sync_tolerance", 0.1);
  max_buffer_size = config.param<int>("glim_ros", "rgb_max_buffer_size", 30);

  if (enabled) {
    std::string calib_file = config.param<std::string>("glim_ros", "rgb_calibration_file", "");
    if (!calib_file.empty() && load_calibration(calib_file)) {
      logger->info("RGB colorizer enabled with calibration: {}", calib_file);
    } else {
      logger->warn("RGB colorizer: Failed to load calibration file, disabling");
      enabled = false;
    }
  } else {
    logger->info("RGB colorizer is disabled");
  }

  if (enabled) {
    set_callbacks();
  }

  logger->info("ready");
}

RGBColorizerROS::~RGBColorizerROS() {
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

bool RGBColorizerROS::load_calibration(const std::string& filepath) {
  try {
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
      logger->error("Cannot open calibration file: {}", filepath);
      return false;
    }

    // Read file content and extract first JSON object (LiDAR-Camera calibration)
    std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    // Find the first { and its matching }
    size_t start = content.find('{');
    if (start == std::string::npos) {
      logger->error("No JSON object found in calibration file");
      return false;
    }

    int brace_count = 0;
    size_t end = start;
    for (size_t i = start; i < content.size(); i++) {
      if (content[i] == '{') brace_count++;
      else if (content[i] == '}') brace_count--;
      if (brace_count == 0) {
        end = i + 1;
        break;
      }
    }

    std::string json_str = content.substr(start, end - start);
    nlohmann::json j = nlohmann::json::parse(json_str);

    // Camera matrix
    double fx = j["camera_matrix"]["fx"];
    double fy = j["camera_matrix"]["fy"];
    double cx = j["camera_matrix"]["cx"];
    double cy = j["camera_matrix"]["cy"];
    camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    // Distortion coefficients
    double k1 = j["dist_coeffs"]["k1"];
    double k2 = j["dist_coeffs"]["k2"];
    double p1 = j["dist_coeffs"]["p1"];
    double p2 = j["dist_coeffs"]["p2"];
    double k3 = j["dist_coeffs"]["k3"];
    dist_coeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

    // Rotation (quaternion to rotation matrix)
    double qx = j["quaternions"]["x"];
    double qy = j["quaternions"]["y"];
    double qz = j["quaternions"]["z"];
    double qw = j["quaternions"]["w"];
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Translation
    double tx = j["tvecs"]["tx"];
    double ty = j["tvecs"]["ty"];
    double tz = j["tvecs"]["tz"];

    // Store T_camera_lidar
    T_camera_lidar = Eigen::Isometry3d::Identity();
    T_camera_lidar.linear() = R;
    T_camera_lidar.translation() = Eigen::Vector3d(tx, ty, tz);

    logger->info("Loaded calibration: fx={:.1f}, fy={:.1f}, cx={:.1f}, cy={:.1f}", fx, fy, cx, cy);
    return true;
  } catch (const std::exception& e) {
    logger->error("Failed to parse calibration file: {}", e.what());
    return false;
  }
}

void RGBColorizerROS::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  try {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (image.empty()) {
      return;
    }

    double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    std::lock_guard<std::mutex> lock(image_buffer_mutex);
    image_buffer.emplace_back(stamp, image);

    while (image_buffer.size() > max_buffer_size) {
      image_buffer.pop_front();
    }
  } catch (const std::exception& e) {
    logger->warn("Failed to decode image: {}", e.what());
  }
}

void RGBColorizerROS::on_new_frame(const EstimationFrame::ConstPtr& frame) {
  if (!enabled) {
    return;
  }

  if (!frame->frame || frame->frame->size() == 0) {
    return;
  }

  // Find matching image for this frame
  cv::Mat matched_image;
  double min_dt_found = sync_tolerance;
  {
    std::lock_guard<std::mutex> lock(image_buffer_mutex);
    for (const auto& img_pair : image_buffer) {
      double dt = std::abs(img_pair.first - frame->stamp);
      if (dt < min_dt_found) {
        min_dt_found = dt;
        matched_image = img_pair.second;
      }
    }
  }

  if (matched_image.empty()) {
    logger->debug("no matching image for frame at stamp {}", frame->stamp);
    return;
  }

  // For LiDAR frame points, use T_camera_lidar directly
  Eigen::Isometry3d T_camera_points = T_camera_lidar;
  if (frame->frame_id == FrameID::IMU) {
    // If points are in IMU frame, need to transform: T_camera_imu = T_camera_lidar * T_lidar_imu
    T_camera_points = T_camera_lidar * frame->T_lidar_imu.inverse();
  }

  // Colorize only FOV-visible points
  ColorizedPoints result = colorize_points_fov_only(
    frame->frame->points, frame->frame->size(), matched_image, T_camera_points);

  if (result.points.empty()) {
    logger->debug("no FOV points for frame at stamp {}", frame->stamp);
    return;
  }

  // Determine frame_id string
  std::string frame_id;
  switch (frame->frame_id) {
    case FrameID::LIDAR:
      frame_id = "lidar";
      break;
    case FrameID::IMU:
      frame_id = "imu";
      break;
    case FrameID::WORLD:
      frame_id = "map";
      break;
  }

  // Create ColorizedPointCloud and notify subscribers
  ColorizedPointCloud colorized;
  colorized.stamp = frame->stamp;
  colorized.frame_id = frame_id;
  colorized.points = std::move(result.points);
  colorized.colors = std::move(result.colors);

  logger->debug("colored frame {} with {} FOV points", frame->id, colorized.points.size());
  RGBColorizerCallbacks::on_frame_colorized(colorized);
}

void RGBColorizerROS::on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  if (!enabled) {
    return;
  }

  // Colorize the new submap points
  const SubMap::ConstPtr latest_submap = submaps.back();
  const double stamp_endpoint_R = latest_submap->odom_frames.back()->stamp;

  cv::Mat matched_image;
  {
    std::lock_guard<std::mutex> lock(image_buffer_mutex);
    double min_dt = sync_tolerance;
    for (const auto& img_pair : image_buffer) {
      double dt = std::abs(img_pair.first - stamp_endpoint_R);
      if (dt < min_dt) {
        min_dt = dt;
        matched_image = img_pair.second;
      }
    }
  }

  if (matched_image.empty()) {
    return;
  }

  // Submap points are in submap origin frame
  // At stamp_endpoint_R, the sensor is at T_origin_endpoint_R relative to submap origin
  Eigen::Isometry3d T_camera_submap_origin = T_camera_lidar * latest_submap->T_origin_endpoint_R.inverse();

  std::vector<uint32_t> colors = colorize_points_all(
    latest_submap->frame->points,
    latest_submap->frame->size(),
    matched_image,
    T_camera_submap_origin);

  // Store colors in submap's point cloud
  auto point_cloud_cpu = std::dynamic_pointer_cast<gtsam_points::PointCloudCPU>(
      std::const_pointer_cast<gtsam_points::PointCloud>(latest_submap->frame));

  if (point_cloud_cpu) {
    point_cloud_cpu->add_aux_attribute<uint32_t>("rgb_colors", colors);
    logger->debug("colored submap with {} points", colors.size());
  }
}

// FOV-only version: returns only points inside camera FOV with colors
RGBColorizerROS::ColorizedPoints RGBColorizerROS::colorize_points_fov_only(
  const Eigen::Vector4d* points, int num_points,
  const cv::Mat& image, const Eigen::Isometry3d& T_camera_points) {

  ColorizedPoints result;

  if (image.empty() || num_points == 0) {
    return result;
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
    return result;
  }

  // Project points to image
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(points_cam, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                    camera_matrix, dist_coeffs, image_points);

  // Extract colors - only keep points that project inside image (FOV)
  for (size_t i = 0; i < valid_indices.size(); i++) {
    int u = static_cast<int>(std::round(image_points[i].x));
    int v = static_cast<int>(std::round(image_points[i].y));

    if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
      cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);
      uint8_t r = bgr[2];
      uint8_t g = bgr[1];
      uint8_t b = bgr[0];
      uint32_t color = (static_cast<uint32_t>(r) << 16) |
                       (static_cast<uint32_t>(g) << 8) |
                       static_cast<uint32_t>(b);

      result.points.push_back(points[valid_indices[i]]);
      result.colors.push_back(color);
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

  // Extract colors
  for (size_t i = 0; i < valid_indices.size(); i++) {
    int u = static_cast<int>(std::round(image_points[i].x));
    int v = static_cast<int>(std::round(image_points[i].y));

    if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
      cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);
      uint8_t r = bgr[2];
      uint8_t g = bgr[1];
      uint8_t b = bgr[0];
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
