/**
 * @brief IMU Calibration Validator - Headless version (no GUI required)
 *
 * This module validates IMU-LiDAR calibration by comparing:
 * 1. Angular velocity from LiDAR odometry vs IMU gyroscope
 * 2. Linear acceleration direction vs gravity direction
 * 3. Time offset between IMU and LiDAR
 *
 * Results are logged periodically and saved to a file at the end.
 */
#include <deque>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <fstream>
#include <numeric>
#include <gtsam/geometry/Rot3.h>

#include <glim/odometry/callbacks.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/concurrent_vector.hpp>

namespace glim {

struct ValidationStats {
  // Angular velocity error statistics
  double avg_angular_vel_error = 0.0;
  double max_angular_vel_error = 0.0;
  double min_angular_vel_error = std::numeric_limits<double>::max();

  // Gravity alignment error statistics
  double avg_gravity_error = 0.0;
  double max_gravity_error = 0.0;
  double min_gravity_error = std::numeric_limits<double>::max();

  // Time offset estimation
  double estimated_imu_t_offset = 0.0;
  double nid_score = 0.0;  // Normalized Information Distance (lower is better)

  // Sample count
  int sample_count = 0;
};

class IMUCalibrationValidatorHeadless : public ExtensionModule {
public:
  IMUCalibrationValidatorHeadless() : logger(create_module_logger("imu_valid")) {
    logger->info("Starting IMU Calibration Validator (headless mode)");

    window_size = 10.0;
    log_interval = 30.0;  // Log stats every 30 seconds
    last_log_time = 0.0;

    OdometryEstimationCallbacks::on_insert_imu.add([this](const double stamp, const auto& a, const auto& w) {
      on_insert_imu(stamp, a, w);
    });
    OdometryEstimationCallbacks::on_new_frame.add([this](const auto& frame) {
      on_new_frame(frame);
    });

    kill_switch = false;
    thread = std::thread([this] { task(); });
  }

  ~IMUCalibrationValidatorHeadless() {
    kill_switch = true;
    thread.join();

    // Save final statistics
    save_statistics();
  }

private:
  void on_insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
    input_imu_queue.push_back((Eigen::Matrix<double, 7, 1>() << stamp, linear_acc, angular_vel).finished());
  }

  void on_new_frame(const EstimationFrame::ConstPtr& frame) {
    input_frame_queue.push_back(frame->clone_wo_points());
  }

  void task() {
    const auto interval = std::chrono::milliseconds(100);
    auto last_time = std::chrono::high_resolution_clock::now();

    while (!kill_switch) {
      const auto now = std::chrono::high_resolution_clock::now();
      const auto elapsed = now - last_time;
      if (elapsed < interval) {
        std::this_thread::sleep_for(interval - elapsed);
      }
      last_time = now;

      const auto new_imu_data = input_imu_queue.get_all_and_clear();
      const auto new_frames = input_frame_queue.get_all_and_clear();

      imu_window.insert(imu_window.end(), new_imu_data.begin(), new_imu_data.end());
      frame_window.insert(frame_window.end(), new_frames.begin(), new_frames.end());

      if (imu_window.empty() || frame_window.empty()) {
        continue;
      }

      while (imu_window.back()[0] - imu_window.front()[0] > window_size) {
        imu_window.pop_front();
      }

      while (frame_window.back()->stamp - frame_window.front()->stamp > window_size) {
        frame_window.pop_front();
      }

      if (frame_window.size() < 10) {
        continue;
      }

      validate();
    }
  }

  void validate() {
    const double current_time = frame_window.back()->stamp;

    // Calculate angular velocity from LiDAR odometry
    std::vector<Eigen::Vector3d> angular_vel_lidar(frame_window.size());
    for (size_t i = 0; i < frame_window.size(); i++) {
      const size_t left = (i > 0) ? i - 1 : 0;
      const size_t right = std::min(i + 1, frame_window.size() - 1);

      const double t0 = frame_window[left]->stamp;
      const double t1 = frame_window[right]->stamp;
      if (t1 - t0 < 1e-6) {
        angular_vel_lidar[i] = Eigen::Vector3d::Zero();
        continue;
      }

      const Eigen::Isometry3d delta = frame_window[left]->T_world_imu.inverse() * frame_window[right]->T_world_imu;
      angular_vel_lidar[i] = gtsam::SO3::Logmap(gtsam::SO3(delta.linear())) / (t1 - t0);
    }

    // Find corresponding IMU data
    const auto find_corresponding_imu_data = [&](const double imu_t_offset) {
      std::vector<size_t> imu_cursors(frame_window.size());
      size_t imu_cursor = 0;

      for (size_t i = 0; i < frame_window.size(); i++) {
        const auto& frame = frame_window[i];
        while (imu_cursor < imu_window.size() - 1 &&
               std::abs(imu_window[imu_cursor + 1][0] + imu_t_offset - frame->stamp) <
                 std::abs(imu_window[imu_cursor][0] + imu_t_offset - frame->stamp)) {
          imu_cursor++;
        }
        imu_cursors[i] = imu_cursor;
      }
      return imu_cursors;
    };

    // Calculate NID for time offset estimation
    const auto calc_nid = [&](const double imu_t_offset) {
      const auto imu_cursors = find_corresponding_imu_data(imu_t_offset);

      std::vector<double> lidar_w(frame_window.size());
      std::vector<double> imu_w(frame_window.size());

      double w_max = 0.0;
      double w_min = std::numeric_limits<double>::max();

      for (size_t i = 0; i < frame_window.size(); i++) {
        lidar_w[i] = angular_vel_lidar[i].norm();
        imu_w[i] = imu_window[imu_cursors[i]].middleRows<3>(4).norm();

        w_max = std::max(w_max, std::max(lidar_w[i], imu_w[i]));
        w_min = std::min(w_min, std::min(lidar_w[i], imu_w[i]));
      }

      if (w_max - w_min < 1e-6) return 1.0;

      const int bins = 10;
      Eigen::VectorXi hist_imu = Eigen::VectorXi::Zero(bins);
      Eigen::VectorXi hist_lidar = Eigen::VectorXi::Zero(bins);
      Eigen::MatrixXi hist_joint = Eigen::MatrixXi::Zero(bins, bins);

      for (size_t i = 0; i < lidar_w.size(); i++) {
        int bin_lidar = static_cast<int>(bins * (lidar_w[i] - w_min) / (w_max - w_min));
        int bin_imu = static_cast<int>(bins * (imu_w[i] - w_min) / (w_max - w_min));

        bin_lidar = std::clamp(bin_lidar, 0, bins - 1);
        bin_imu = std::clamp(bin_imu, 0, bins - 1);

        hist_imu[bin_imu]++;
        hist_lidar[bin_lidar]++;
        hist_joint(bin_lidar, bin_imu)++;
      }

      Eigen::VectorXd hist_r = hist_lidar.cast<double>() / imu_w.size();
      Eigen::VectorXd hist_s = hist_imu.cast<double>() / imu_w.size();
      Eigen::MatrixXd hist_rs = hist_joint.cast<double>() / imu_w.size();

      double Hr = (-hist_r.array() * (hist_r.array() + 1e-6).log()).sum();
      double Hs = (-hist_s.array() * (hist_s.array() + 1e-6).log()).sum();
      double Hrs = (-hist_rs.array() * (hist_rs.array() + 1e-6).log()).sum();

      double MI = Hr + Hs - Hrs;
      double NID = (Hrs - MI) / (Hrs + 1e-6);

      return NID;
    };

    // Find best time offset
    double best_nid = std::numeric_limits<double>::max();
    double best_imu_t_offset = 0.0;
    for (double imu_t_offset = -0.2; imu_t_offset <= 0.2; imu_t_offset += 0.001) {
      const double nid = calc_nid(imu_t_offset);
      if (nid < best_nid) {
        best_nid = nid;
        best_imu_t_offset = imu_t_offset;
      }
    }

    // Calculate errors with best offset
    const auto imu_cursors = find_corresponding_imu_data(best_imu_t_offset);

    std::vector<double> angular_vel_errors;
    std::vector<double> gravity_errors;

    for (size_t i = 0; i < frame_window.size(); i++) {
      const auto& frame = frame_window[i];
      const auto& imu = imu_window[imu_cursors[i]];

      // Angular velocity error
      Eigen::Vector3d imu_angular_vel = imu.middleRows<3>(4);
      double w_error = (angular_vel_lidar[i] - imu_angular_vel).norm();
      angular_vel_errors.push_back(w_error);

      // Gravity alignment error (how well IMU z-axis aligns with gravity)
      Eigen::Vector3d acc_world = frame->T_world_imu.linear() * imu.middleRows<3>(1);
      double gravity_error = (acc_world.normalized() - Eigen::Vector3d::UnitZ()).norm();
      gravity_errors.push_back(gravity_error);
    }

    // Update statistics
    std::lock_guard<std::mutex> lock(stats_mutex);

    for (double e : angular_vel_errors) {
      cumulative_angular_vel_errors.push_back(e);
    }
    for (double e : gravity_errors) {
      cumulative_gravity_errors.push_back(e);
    }
    imu_t_offset_samples.push_back(best_imu_t_offset);
    nid_samples.push_back(best_nid);

    // Periodic logging
    if (current_time - last_log_time > log_interval) {
      last_log_time = current_time;
      log_current_stats();
    }
  }

  void log_current_stats() {
    if (cumulative_angular_vel_errors.empty()) return;

    ValidationStats stats = compute_stats();

    logger->info("=== IMU Calibration Validation Report ===");
    logger->info("Samples: {}", stats.sample_count);
    logger->info("Angular velocity error (rad/s): avg={:.4f}, min={:.4f}, max={:.4f}",
                 stats.avg_angular_vel_error, stats.min_angular_vel_error, stats.max_angular_vel_error);
    logger->info("Gravity alignment error: avg={:.4f}, min={:.4f}, max={:.4f}",
                 stats.avg_gravity_error, stats.min_gravity_error, stats.max_gravity_error);
    logger->info("Estimated IMU time offset: {:.3f} sec", stats.estimated_imu_t_offset);
    logger->info("NID score: {:.4f} (lower is better, <0.5 is good)", stats.nid_score);

    // Provide recommendations
    if (std::abs(stats.estimated_imu_t_offset) > 0.01) {
      logger->warn("Recommendation: Consider setting imu_time_offset to {:.3f} in config_ros.json",
                   stats.estimated_imu_t_offset);
    }
    if (stats.avg_gravity_error > 0.2) {
      logger->warn("Recommendation: Gravity alignment error is high. Check IMU orientation calibration (T_lidar_imu rotation).");
    }
    if (stats.avg_angular_vel_error > 0.1) {
      logger->warn("Recommendation: Angular velocity error is high. Check IMU-LiDAR extrinsic calibration or IMU noise parameters.");
    }
  }

  ValidationStats compute_stats() {
    ValidationStats stats;
    stats.sample_count = cumulative_angular_vel_errors.size();

    if (stats.sample_count == 0) return stats;

    // Angular velocity errors
    stats.avg_angular_vel_error = std::accumulate(cumulative_angular_vel_errors.begin(),
                                                   cumulative_angular_vel_errors.end(), 0.0) / stats.sample_count;
    stats.min_angular_vel_error = *std::min_element(cumulative_angular_vel_errors.begin(),
                                                     cumulative_angular_vel_errors.end());
    stats.max_angular_vel_error = *std::max_element(cumulative_angular_vel_errors.begin(),
                                                     cumulative_angular_vel_errors.end());

    // Gravity errors
    stats.avg_gravity_error = std::accumulate(cumulative_gravity_errors.begin(),
                                               cumulative_gravity_errors.end(), 0.0) / stats.sample_count;
    stats.min_gravity_error = *std::min_element(cumulative_gravity_errors.begin(),
                                                 cumulative_gravity_errors.end());
    stats.max_gravity_error = *std::max_element(cumulative_gravity_errors.begin(),
                                                 cumulative_gravity_errors.end());

    // Time offset (use median for robustness)
    if (!imu_t_offset_samples.empty()) {
      std::vector<double> sorted_offsets = imu_t_offset_samples;
      std::sort(sorted_offsets.begin(), sorted_offsets.end());
      stats.estimated_imu_t_offset = sorted_offsets[sorted_offsets.size() / 2];
    }

    // NID score (average)
    if (!nid_samples.empty()) {
      stats.nid_score = std::accumulate(nid_samples.begin(), nid_samples.end(), 0.0) / nid_samples.size();
    }

    return stats;
  }

  void save_statistics() {
    std::lock_guard<std::mutex> lock(stats_mutex);

    if (cumulative_angular_vel_errors.empty()) {
      logger->info("No IMU validation data collected");
      return;
    }

    ValidationStats stats = compute_stats();

    logger->info("========================================");
    logger->info("Final IMU Calibration Validation Report");
    logger->info("========================================");
    logger->info("Total samples analyzed: {}", stats.sample_count);
    logger->info("");
    logger->info("Angular Velocity Error (LiDAR vs IMU):");
    logger->info("  Average: {:.4f} rad/s", stats.avg_angular_vel_error);
    logger->info("  Min:     {:.4f} rad/s", stats.min_angular_vel_error);
    logger->info("  Max:     {:.4f} rad/s", stats.max_angular_vel_error);
    logger->info("");
    logger->info("Gravity Alignment Error:");
    logger->info("  Average: {:.4f}", stats.avg_gravity_error);
    logger->info("  Min:     {:.4f}", stats.min_gravity_error);
    logger->info("  Max:     {:.4f}", stats.max_gravity_error);
    logger->info("");
    logger->info("Time Synchronization:");
    logger->info("  Estimated IMU time offset: {:.3f} sec", stats.estimated_imu_t_offset);
    logger->info("  NID score: {:.4f}", stats.nid_score);
    logger->info("");

    // Recommendations
    logger->info("=== Recommendations ===");

    bool has_issues = false;

    if (std::abs(stats.estimated_imu_t_offset) > 0.01) {
      logger->warn("- TIME OFFSET: Set 'imu_time_offset': {:.3f} in config_ros.json", stats.estimated_imu_t_offset);
      has_issues = true;
    }

    if (stats.avg_gravity_error > 0.3) {
      logger->warn("- ROTATION: Gravity error is HIGH ({:.3f}). Check T_lidar_imu rotation in config_sensors.json", stats.avg_gravity_error);
      has_issues = true;
    } else if (stats.avg_gravity_error > 0.15) {
      logger->warn("- ROTATION: Gravity error is moderate ({:.3f}). Consider refining T_lidar_imu rotation", stats.avg_gravity_error);
      has_issues = true;
    }

    if (stats.avg_angular_vel_error > 0.15) {
      logger->warn("- ANGULAR VEL: Error is HIGH ({:.3f} rad/s). Check IMU gyro calibration or extrinsics", stats.avg_angular_vel_error);
      has_issues = true;
    } else if (stats.avg_angular_vel_error > 0.08) {
      logger->warn("- ANGULAR VEL: Error is moderate ({:.3f} rad/s). Consider adjusting imu_gyro_noise", stats.avg_angular_vel_error);
      has_issues = true;
    }

    if (stats.nid_score > 0.7) {
      logger->warn("- CORRELATION: NID score is poor ({:.3f}). IMU and LiDAR data may be poorly synchronized", stats.nid_score);
      has_issues = true;
    }

    if (!has_issues) {
      logger->info("IMU calibration appears to be good!");
    }

    logger->info("========================================");
  }

private:
  ConcurrentVector<Eigen::Matrix<double, 7, 1>> input_imu_queue;
  ConcurrentVector<EstimationFrame::ConstPtr> input_frame_queue;

  double window_size;
  double log_interval;
  double last_log_time;

  std::atomic_bool kill_switch;
  std::thread thread;

  std::deque<Eigen::Matrix<double, 7, 1>> imu_window;
  std::deque<EstimationFrame::ConstPtr> frame_window;

  // Cumulative statistics
  std::mutex stats_mutex;
  std::vector<double> cumulative_angular_vel_errors;
  std::vector<double> cumulative_gravity_errors;
  std::vector<double> imu_t_offset_samples;
  std::vector<double> nid_samples;

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::IMUCalibrationValidatorHeadless();
}
