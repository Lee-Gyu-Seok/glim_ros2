#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  // Record start time for elapsed time calculation
  auto program_start_time = std::chrono::high_resolution_clock::now();
  auto wall_start_time = std::chrono::system_clock::now();

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  rclcpp::spin(glim);
  rclcpp::shutdown();

  // Record time after spin ends (data processing complete)
  auto spin_end_time = std::chrono::high_resolution_clock::now();
  double spin_elapsed_sec = std::chrono::duration<double>(spin_end_time - program_start_time).count();

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

  // Record time before wait/save
  auto wait_start_time = std::chrono::high_resolution_clock::now();

  glim->wait();
  glim->save(dump_path);

  // Record final time
  auto program_end_time = std::chrono::high_resolution_clock::now();
  auto wall_end_time = std::chrono::system_clock::now();

  // Calculate elapsed times
  double total_elapsed_sec = std::chrono::duration<double>(program_end_time - program_start_time).count();
  double wait_save_elapsed_sec = std::chrono::duration<double>(program_end_time - wait_start_time).count();
  double processing_elapsed_sec = spin_elapsed_sec;

  // Format wall clock times
  auto wall_start_t = std::chrono::system_clock::to_time_t(wall_start_time);
  auto wall_end_t = std::chrono::system_clock::to_time_t(wall_end_time);
  std::stringstream start_ss, end_ss;
  start_ss << std::put_time(std::localtime(&wall_start_t), "%Y-%m-%d %H:%M:%S");
  end_ss << std::put_time(std::localtime(&wall_end_t), "%Y-%m-%d %H:%M:%S");

  // Calculate formatted time
  int total_min = static_cast<int>(total_elapsed_sec) / 60;
  int total_sec = static_cast<int>(total_elapsed_sec) % 60;
  int proc_min = static_cast<int>(processing_elapsed_sec) / 60;
  int proc_sec = static_cast<int>(processing_elapsed_sec) % 60;

  // Save elapsed time information to Log folder
  std::ofstream elapsed_time_ofs(dump_path + "/elapsed_time.txt");
  if (elapsed_time_ofs.is_open()) {
    elapsed_time_ofs << "# GLIM Elapsed Time Report" << std::endl;
    elapsed_time_ofs << "# ========================" << std::endl;
    elapsed_time_ofs << std::endl;
    elapsed_time_ofs << "# Wall Clock Time" << std::endl;
    elapsed_time_ofs << "start_time: " << start_ss.str() << std::endl;
    elapsed_time_ofs << "end_time: " << end_ss.str() << std::endl;
    elapsed_time_ofs << std::endl;
    elapsed_time_ofs << "# Elapsed Time (seconds)" << std::endl;
    elapsed_time_ofs << std::fixed << std::setprecision(3);
    elapsed_time_ofs << "total_elapsed_sec: " << total_elapsed_sec << std::endl;
    elapsed_time_ofs << "processing_elapsed_sec: " << processing_elapsed_sec << std::endl;
    elapsed_time_ofs << "wait_save_elapsed_sec: " << wait_save_elapsed_sec << std::endl;
    elapsed_time_ofs << std::endl;
    elapsed_time_ofs << "# Elapsed Time (formatted)" << std::endl;
    elapsed_time_ofs << "total_elapsed: " << total_min << "m " << total_sec << "s" << std::endl;
    elapsed_time_ofs << "processing_elapsed: " << proc_min << "m " << proc_sec << "s" << std::endl;
    elapsed_time_ofs.close();
    spdlog::info("Elapsed time saved to: {}/elapsed_time.txt", dump_path);
  }

  spdlog::info("Map saved to: {}", dump_path);
  spdlog::info("Total elapsed time: {:.1f} sec ({} min {} sec)", total_elapsed_sec, total_min, total_sec);

  return 0;
}
