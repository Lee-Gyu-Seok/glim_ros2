#pragma once

#include <mutex>
#include <deque>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

#include <glim/util/extension_module.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>

namespace glim {

class RGBColorizerModule : public ExtensionModule {
public:
  RGBColorizerModule();
  ~RGBColorizerModule();

private:
  bool load_calibration(const std::string& filepath);
  cv::Mat quaternion_to_rotation_matrix(double x, double y, double z, double w);
  void project_points_to_image(
    const Eigen::Vector4d* points_lidar,
    int num_points,
    const cv::Mat& image,
    std::vector<Eigen::Vector4f>& colors_out);

  void on_new_frame(const EstimationFrame::ConstPtr& frame);
  void on_insert_image(double stamp, const cv::Mat& image);
  void on_update_submaps(const std::vector<SubMap::Ptr>& submaps);

private:
  // Calibration parameters
  bool calibration_loaded;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
  cv::Mat rotation_lidar_to_camera;
  cv::Mat translation_lidar_to_camera;
  double sync_tolerance;

  // Image buffer
  std::mutex image_mutex;
  std::deque<std::pair<double, cv::Mat>> image_buffer;
  size_t max_buffer_size;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim
