#include <glim_ext/rgb_colorizer.hpp>

#include <fstream>
#include <regex>
#include <nlohmann/json.hpp>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>

namespace glim {

RGBColorizerModule::RGBColorizerModule() : logger(create_module_logger("rgb_color")) {
  logger->info("starting RGB colorizer module");

  calibration_loaded = false;
  sync_tolerance = 0.1;
  max_buffer_size = 30;

  // Load config from config_rgb_colorizer.json
  const std::string config_path = GlobalConfig::get_config_path("config_rgb_colorizer");
  logger->info("looking for config at: {}", config_path);

  if (!config_path.empty()) {
    Config config(config_path);
    sync_tolerance = config.param<double>("rgb_colorizer", "sync_tolerance", 0.1);
    max_buffer_size = config.param<int>("rgb_colorizer", "max_buffer_size", 30);

    const auto calib_path = config.param<std::string>("rgb_colorizer", "calibration_file");
    if (calib_path) {
      calibration_loaded = load_calibration(*calib_path);
    }
  }

  if (!calibration_loaded) {
    logger->warn("calibration not loaded, RGB colorization disabled");
  }

  // Register callbacks - on_insert_image must come before on_new_frame
  OdometryEstimationCallbacks::on_insert_image.add([this](double stamp, const cv::Mat& image) {
    this->on_insert_image(stamp, image);
  });

  OdometryEstimationCallbacks::on_new_frame.add([this](const EstimationFrame::ConstPtr& frame) {
    this->on_new_frame(frame);
  });

  GlobalMappingCallbacks::on_update_submaps.add([this](const std::vector<SubMap::Ptr>& submaps) {
    this->on_update_submaps(submaps);
  });

  logger->info("ready");
}

RGBColorizerModule::~RGBColorizerModule() {
}

bool RGBColorizerModule::load_calibration(const std::string& filepath) {
  logger->info("loading calibration from: {}", filepath);

  std::ifstream file(filepath);
  if (!file.is_open()) {
    logger->error("failed to open calibration file: {}", filepath);
    return false;
  }

  // Read file content and remove comments (lines starting with #)
  std::stringstream buffer;
  std::string line;
  bool in_first_json = false;
  int brace_count = 0;

  while (std::getline(file, line)) {
    // Skip comment lines
    size_t first_non_space = line.find_first_not_of(" \t");
    if (first_non_space != std::string::npos && line[first_non_space] == '#') {
      continue;
    }

    // Track braces to get only the first JSON object (LiDAR-Camera)
    for (char c : line) {
      if (c == '{') {
        in_first_json = true;
        brace_count++;
      } else if (c == '}') {
        brace_count--;
      }
    }

    if (in_first_json) {
      buffer << line << "\n";
      if (brace_count == 0) {
        break;  // Stop after first complete JSON object
      }
    }
  }
  file.close();

  try {
    nlohmann::json j = nlohmann::json::parse(buffer.str());

    // Camera intrinsics
    double fx = j["camera_matrix"]["fx"];
    double fy = j["camera_matrix"]["fy"];
    double cx = j["camera_matrix"]["cx"];
    double cy = j["camera_matrix"]["cy"];
    double skew = j["camera_matrix"].value("skew", 0.0);

    camera_matrix = (cv::Mat_<double>(3, 3) <<
      fx, skew, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0);

    // Distortion coefficients
    double k1 = j["dist_coeffs"]["k1"];
    double k2 = j["dist_coeffs"]["k2"];
    double p1 = j["dist_coeffs"]["p1"];
    double p2 = j["dist_coeffs"]["p2"];
    double k3 = j["dist_coeffs"].value("k3", 0.0);

    dist_coeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

    // Extrinsics (quaternion to rotation matrix)
    double qx = j["quaternions"]["x"];
    double qy = j["quaternions"]["y"];
    double qz = j["quaternions"]["z"];
    double qw = j["quaternions"]["w"];

    rotation_lidar_to_camera = quaternion_to_rotation_matrix(qx, qy, qz, qw);

    // Translation
    double tx = j["tvecs"]["tx"];
    double ty = j["tvecs"]["ty"];
    double tz = j["tvecs"]["tz"];

    translation_lidar_to_camera = (cv::Mat_<double>(3, 1) << tx, ty, tz);

    logger->info("calibration loaded successfully");
    logger->info("  fx={:.2f}, fy={:.2f}, cx={:.2f}, cy={:.2f}", fx, fy, cx, cy);
    logger->info("  k1={:.4f}, k2={:.4f}, p1={:.4f}, p2={:.4f}", k1, k2, p1, p2);
    logger->info("  rotation: qx={:.4f}, qy={:.4f}, qz={:.4f}, qw={:.4f}", qx, qy, qz, qw);
    logger->info("  translation: tx={:.4f}, ty={:.4f}, tz={:.4f}", tx, ty, tz);

    return true;
  } catch (const std::exception& e) {
    logger->error("failed to parse calibration file: {}", e.what());
    return false;
  }
}

cv::Mat RGBColorizerModule::quaternion_to_rotation_matrix(double x, double y, double z, double w) {
  // Normalize quaternion
  double norm = std::sqrt(x*x + y*y + z*z + w*w);
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  cv::Mat R = (cv::Mat_<double>(3, 3) <<
    1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w), 2.0*(x*z + y*w),
    2.0*(x*y + z*w), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w),
    2.0*(x*z - y*w), 2.0*(y*z + x*w), 1.0 - 2.0*(x*x + y*y));

  return R;
}

void RGBColorizerModule::project_points_to_image(
    const Eigen::Vector4d* points_lidar,
    int num_points,
    const cv::Mat& image,
    std::vector<Eigen::Vector4f>& colors_out) {

  colors_out.resize(num_points);

  if (!calibration_loaded || image.empty()) {
    // Default to white color
    for (int i = 0; i < num_points; i++) {
      colors_out[i] = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    }
    return;
  }

  // Pre-extract rotation and translation values
  double r00 = rotation_lidar_to_camera.at<double>(0, 0);
  double r01 = rotation_lidar_to_camera.at<double>(0, 1);
  double r02 = rotation_lidar_to_camera.at<double>(0, 2);
  double r10 = rotation_lidar_to_camera.at<double>(1, 0);
  double r11 = rotation_lidar_to_camera.at<double>(1, 1);
  double r12 = rotation_lidar_to_camera.at<double>(1, 2);
  double r20 = rotation_lidar_to_camera.at<double>(2, 0);
  double r21 = rotation_lidar_to_camera.at<double>(2, 1);
  double r22 = rotation_lidar_to_camera.at<double>(2, 2);
  double tx = translation_lidar_to_camera.at<double>(0, 0);
  double ty = translation_lidar_to_camera.at<double>(1, 0);
  double tz = translation_lidar_to_camera.at<double>(2, 0);

  // Collect valid 3D points for projection
  std::vector<cv::Point3f> points_cam;
  std::vector<int> valid_indices;
  points_cam.reserve(num_points);
  valid_indices.reserve(num_points);

  for (int i = 0; i < num_points; i++) {
    const auto& pt = points_lidar[i];

    // Check for NaN/Inf
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) || !std::isfinite(pt.z())) {
      colors_out[i] = Eigen::Vector4f(0.5f, 0.5f, 0.5f, 1.0f);
      continue;
    }

    // Transform LiDAR point to camera frame
    double x_cam = r00 * pt.x() + r01 * pt.y() + r02 * pt.z() + tx;
    double y_cam = r10 * pt.x() + r11 * pt.y() + r12 * pt.z() + ty;
    double z_cam = r20 * pt.x() + r21 * pt.y() + r22 * pt.z() + tz;

    // Skip points behind camera
    if (z_cam <= 0.0) {
      colors_out[i] = Eigen::Vector4f(0.5f, 0.5f, 0.5f, 1.0f);
      continue;
    }

    points_cam.emplace_back(static_cast<float>(x_cam), static_cast<float>(y_cam), static_cast<float>(z_cam));
    valid_indices.push_back(i);
  }

  if (points_cam.empty()) {
    return;
  }

  // Project to 2D using OpenCV (handles distortion)
  std::vector<cv::Point2f> points_2d;
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);  // Identity rotation (already transformed)
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);  // Zero translation (already transformed)
  cv::projectPoints(points_cam, rvec, tvec, camera_matrix, dist_coeffs, points_2d);

  // Extract RGB colors from image
  for (size_t j = 0; j < points_2d.size(); j++) {
    int i = valid_indices[j];
    int u = static_cast<int>(points_2d[j].x);
    int v = static_cast<int>(points_2d[j].y);

    if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
      cv::Vec3b bgr = image.at<cv::Vec3b>(v, u);
      colors_out[i] = Eigen::Vector4f(
        bgr[2] / 255.0f,  // R
        bgr[1] / 255.0f,  // G
        bgr[0] / 255.0f,  // B
        1.0f              // A
      );
    } else {
      colors_out[i] = Eigen::Vector4f(0.5f, 0.5f, 0.5f, 1.0f);
    }
  }
}

void RGBColorizerModule::on_new_frame(const EstimationFrame::ConstPtr& frame) {
  if (!calibration_loaded) {
    return;
  }

  if (!frame->frame || frame->frame->size() == 0) {
    return;
  }

  // Find matching image synchronously
  cv::Mat matched_image;
  {
    std::lock_guard<std::mutex> lock(image_mutex);

    double min_dt = std::numeric_limits<double>::max();
    for (const auto& img_pair : image_buffer) {
      double dt = std::abs(img_pair.first - frame->stamp);
      if (dt < min_dt) {
        min_dt = dt;
        if (dt < sync_tolerance) {
          matched_image = img_pair.second;
        }
      }
    }
  }

  if (matched_image.empty()) {
    logger->debug("no matching image for frame at stamp {}", frame->stamp);
    return;
  }

  // Project points and get colors
  std::vector<Eigen::Vector4f> colors;
  project_points_to_image(frame->frame->points, frame->frame->size(), matched_image, colors);

  // Add colors directly to the frame's point cloud
  auto point_cloud_cpu = std::dynamic_pointer_cast<gtsam_points::PointCloudCPU>(
      std::const_pointer_cast<gtsam_points::PointCloud>(frame->frame));

  if (point_cloud_cpu) {
    point_cloud_cpu->add_aux_attribute<Eigen::Vector4f>("colors", colors);
    logger->debug("colored frame {} with {} points", frame->id, colors.size());
  }
}

void RGBColorizerModule::on_insert_image(double stamp, const cv::Mat& image) {
  if (!calibration_loaded) {
    return;
  }

  std::lock_guard<std::mutex> lock(image_mutex);
  image_buffer.emplace_back(stamp, image.clone());

  // Keep buffer size limited
  while (image_buffer.size() > max_buffer_size) {
    image_buffer.pop_front();
  }
}

void RGBColorizerModule::on_update_submaps(const std::vector<SubMap::Ptr>& submaps) {
  // Submaps should already have colors from the odometry frames
  // This callback can be used for additional processing if needed
}

}  // namespace glim
