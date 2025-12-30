#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  rclcpp::spin(glim);

  // Create Log directory
  std::string log_dir = std::string(GLIM_ROS_SOURCE_DIR) + "/Log";
  if (!std::filesystem::exists(log_dir)) {
    std::filesystem::create_directories(log_dir);
    spdlog::info("Created log directory: {}", log_dir);
  }

  // Create map folder with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  std::string dump_path = log_dir + "/map_" + ss.str();

  glim->wait();
  glim->save(dump_path);
  spdlog::info("Map saved to: {}", dump_path);

  // Shutdown ROS2 after all processing is complete
  rclcpp::shutdown();

  return 0;
}