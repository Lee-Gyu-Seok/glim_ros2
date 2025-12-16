#include <deque>
#include <mutex>
#include <thread>
#include <iostream>
#include <set>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/logging.hpp>
#include <glim/util/config.hpp>
#include <glim/util/profiler.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/extension_module.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Scancontext/Scancontext.h>

namespace glim {

/**
 * @brief ScanContext-based loop detector
 * @note  TODO: make some hard-coded parameters configurable
 *
 */
class ScanContextLoopDetector : public ExtensionModule {
public:
  /**
   * @brief Construct loop detector
   *
   */
  ScanContextLoopDetector() : logger(create_module_logger("scancontext")) {
    logger->debug("Creating ScanContext manager...");
    sc.reset(new SCManager);

    // Load parameters from dedicated config file
    // Note: get_config_path() appends .json automatically, so don't include it in the name
    auto config_path = GlobalConfig::get_config_path("config_scan_context");
    logger->debug("Loading ScanContext config from: {}", config_path);

    // Create a separate Config instance to load the ScanContext config file
    Config config(config_path);

    // ScanContext parameters (configurable)
    sc->LIDAR_HEIGHT = config.param<double>("scan_context", "lidar_height", 2.0);
    sc->PC_NUM_RING = config.param<int>("scan_context", "pc_num_ring", 20);
    sc->PC_NUM_SECTOR = config.param<int>("scan_context", "pc_num_sector", 60);
    sc->PC_MAX_RADIUS = config.param<double>("scan_context", "pc_max_radius", 80.0);
    sc->PC_FOV = config.param<double>("scan_context", "pc_fov", 360.0);
    sc->PC_FOV_OFFSET = config.param<double>("scan_context", "pc_fov_offset", -60.0);
    sc->NUM_EXCLUDE_RECENT = config.param<int>("scan_context", "num_exclude_recent", 50);
    sc->NUM_CANDIDATES_FROM_TREE = config.param<int>("scan_context", "num_candidates_from_tree", 10);
    sc->SEARCH_RATIO = config.param<double>("scan_context", "search_ratio", 0.1);
    sc->SC_DIST_THRES = config.param<double>("scan_context", "sc_dist_threshold", 0.2);
    sc->TREE_MAKING_PERIOD_ = config.param<int>("scan_context", "tree_making_period", 50);

    // Update derived parameters
    sc->updateDerivedParams();

    // Loop detector parameters
    min_movement_threshold = config.param<double>("scan_context", "min_movement_threshold", 1.0);
    inlier_fraction_threshold = config.param<double>("scan_context", "inlier_fraction_threshold", 0.7);
    icp_max_iterations = config.param<int>("scan_context", "icp_max_iterations", 5);
    icp_num_threads = config.param<int>("scan_context", "icp_num_threads", 4);
    loop_noise_score = config.param<double>("scan_context", "loop_noise_score", 0.5);

    // Odometry-based dynamic ground estimation parameters
    use_odometry_ground = config.param<bool>("scan_context", "use_odometry_ground", true);
    initial_lidar_height = config.param<double>("scan_context", "lidar_height", 2.0);
    ground_initialized = false;
    initial_sensor_z = 0.0;

    logger->debug("ScanContext parameters:");
    logger->debug("  lidar_height: {}", sc->LIDAR_HEIGHT);
    logger->debug("  pc_num_ring: {}", sc->PC_NUM_RING);
    logger->debug("  pc_num_sector: {}", sc->PC_NUM_SECTOR);
    logger->debug("  pc_max_radius: {}", sc->PC_MAX_RADIUS);
    logger->debug("  pc_fov: {}", sc->PC_FOV);
    logger->debug("  pc_fov_offset: {}", sc->PC_FOV_OFFSET);
    logger->debug("  sc_dist_threshold: {}", sc->SC_DIST_THRES);
    logger->debug("  min_movement_threshold: {}", min_movement_threshold);
    logger->debug("  inlier_fraction_threshold: {}", inlier_fraction_threshold);
    logger->debug("  loop_noise_score: {}", loop_noise_score);
    logger->debug("  use_odometry_ground: {}", use_odometry_ground);
    logger->debug("  initial_lidar_height: {}", initial_lidar_height);
    logger->info("ScanContext loop detector ready");

    frame_count = 0;

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&ScanContextLoopDetector::on_new_frame, this, _1));
    GlobalMappingCallbacks::on_insert_submap.add(std::bind(&ScanContextLoopDetector::on_new_submap, this, _1));
    GlobalMappingCallbacks::on_smoother_update.add(std::bind(&ScanContextLoopDetector::on_smoother_update, this, _1, _2, _3));

    kill_switch = false;
    thread = std::thread([this] { loop_detection_task(); });
  }

  /**
   * @brief Destroy the ScanContextLoopDetector object
   *
   */
  ~ScanContextLoopDetector() {
    kill_switch = true;
    if (thread.joinable()) {
      thread.join();
    }
  }

  /**
   * @brief Odometry estimation frame input callback
   *
   * @param odom_frame
   */
  void on_new_frame(const EstimationFrame::ConstPtr& odom_frame) { odom_frames_queue.push_back(odom_frame); }

  /**
   * @brief Submap input callback
   *
   * @param submap
   */
  void on_new_submap(const SubMap::ConstPtr& submap) { new_submaps_queue.push_back(submap); }

  /**
   * @brief Global mapping optimization callback
   *        Here, we inject loop constaints into the global mapping factor graph
   *
   * @param isam2
   * @param new_factors
   * @param new_values
   */
  void on_smoother_update(gtsam_points::ISAM2Ext& isam2, gtsam::NonlinearFactorGraph& new_factors, gtsam::Values& new_values) {
    if (loop_factors.empty()) {
      return;
    }

    const auto factors = loop_factors.get_all_and_clear();
    new_factors.add(factors);
  }

  /**
   * @brief Loop detection thread
   *
   */
  void loop_detection_task() {
    Profiler::instance().set_thread_name("scan_context");
    Eigen::Isometry3d last_T_odom_sensor = Eigen::Isometry3d::Identity();

    // Loop
    while (!kill_switch) {
      auto odom_frames = odom_frames_queue.get_all_and_clear();
      auto new_submaps = new_submaps_queue.get_all_and_clear();
      submaps.insert(submaps.end(), new_submaps.begin(), new_submaps.end());

      for (const auto& odom_frame : odom_frames) {
        const Eigen::Isometry3d delta = last_T_odom_sensor.inverse() * odom_frame->T_world_sensor();
        if (delta.translation().norm() < min_movement_threshold) {
          // Do not insert the frame into the ScanContext manager while the sensor is stopping
          continue;
        }

        // Add the current frame into the SC manager
        const int current = frame_count++;
        frame_index_map[current] = odom_frame->id;
        last_T_odom_sensor = odom_frame->T_world_sensor();

        // Calculate dynamic ground height based on odometry
        double ground_z_offset = 0.0;
        if (use_odometry_ground) {
          const double current_sensor_z = odom_frame->T_world_sensor().translation().z();

          if (!ground_initialized) {
            // Initialize with first frame's sensor z position
            initial_sensor_z = current_sensor_z;
            ground_initialized = true;
            logger->info("Odometry-based ground estimation initialized: sensor_z={:.3f}, initial_height={:.3f}",
                         initial_sensor_z, initial_lidar_height);
          }

          // Calculate current ground z in world frame
          // ground_z = initial_sensor_z - initial_lidar_height (assumed ground level)
          // current height above ground = current_sensor_z - ground_z
          // z_offset to apply = current_sensor_z - initial_sensor_z (relative height change)
          ground_z_offset = current_sensor_z - initial_sensor_z;
        }

        const auto& frame = odom_frame->frame;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.resize(frame->size());
        for (int i = 0; i < frame->size(); i++) {
          cloud.at(i).getVector4fMap() = frame->points[i].cast<float>();
          // Apply ground normalization: subtract the relative height change
          // This makes all point clouds as if they were captured at the initial sensor height
          if (use_odometry_ground) {
            cloud.at(i).z -= static_cast<float>(ground_z_offset);
          }
        }

        Profiler::instance().start("scan_context");
        sc->makeAndSaveScancontextAndKeys(cloud);
        auto loop = sc->detectLoopClosureID();
        Profiler::instance().stop("scan_context");

        if (loop.first < 0) {
          continue;
        }

        // Detected loops will be validated later
        loop_candidates.push_back(std::make_tuple(current, loop.first, loop.second));
      }

      // Validate loop candidates
      while (!loop_candidates.empty()) {
        // Safety check: ensure submaps is not empty before accessing
        if (submaps.empty()) {
          break;
        }

        const auto loop_candidate = loop_candidates.front();
        const int sc_idx1 = std::get<0>(loop_candidate);
        const int sc_idx2 = std::get<1>(loop_candidate);
        const double heading = std::get<2>(loop_candidate);

        // Safety check: ensure frame_index_map contains the required keys
        auto it1 = frame_index_map.find(sc_idx1);
        auto it2 = frame_index_map.find(sc_idx2);
        if (it1 == frame_index_map.end() || it2 == frame_index_map.end()) {
          logger->warn("frame_index_map missing key: sc_idx1={}, sc_idx2={}", sc_idx1, sc_idx2);
          loop_candidates.pop_front();
          continue;
        }

        const int frame_id1 = it1->second;
        const int frame_id2 = it2->second;

        // Safety check: ensure submaps.back() has valid odom_frames
        if (submaps.back()->odom_frames.empty()) {
          logger->warn("latest submap has no odom_frames");
          break;
        }

        // The frame is newer than the latest submap
        if (frame_id1 > submaps.back()->odom_frames.back()->id) {
          break;
        }
        loop_candidates.pop_front();

        Eigen::Isometry3d T_origin1_frame1 = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d T_origin2_frame2 = Eigen::Isometry3d::Identity();
        const auto submap1 = find_submap(frame_id1, T_origin1_frame1);
        const auto submap2 = find_submap(frame_id2, T_origin2_frame2);

        if (!submap1 || !submap2) {
          // Failed to find corresponding submaps
          continue;
        }

        // Check for duplicate loop closures between the same submap pair
        const int min_id = std::min(submap1->id, submap2->id);
        const int max_id = std::max(submap1->id, submap2->id);
        const auto loop_key = std::make_pair(min_id, max_id);
        if (registered_loops.count(loop_key) > 0) {
          logger->debug("skipping duplicate loop candidate=({}, {})", submap1->id, submap2->id);
          continue;
        }

        logger->debug("validate loop candidate=({}, {})", submap1->id, submap2->id);

        // Initial guess for the relative pose between submaps
        Eigen::Isometry3d T_frame1_frame2 = Eigen::Isometry3d::Identity();
        // For limited FOV LiDAR (< 360°), sector shift does NOT represent yaw difference
        // because the same scene is only visible when looking in the same direction.
        // Only use heading estimate for full 360° LiDAR.
        if (sc->PC_FOV >= 360.0) {
          T_frame1_frame2.linear() = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        }
        // For limited FOV, use Identity rotation (assume same heading direction)

        Eigen::Isometry3d T_origin1_origin2 = T_origin1_frame1 * T_frame1_frame2 * T_origin2_frame2.inverse();
        Profiler::instance().start("scan_context/icp_validation");
        bool valid = validate_loop(submap1->frame, submap2->frame, T_origin1_origin2);
        Profiler::instance().stop("scan_context/icp_validation");
        if (!valid) {
          continue;
        }

        // TODO: should check if it's close to the current estimate?
        logger->info("Loop detected!! ({}, {})", submap1->id, submap2->id);

        // Register the loop to prevent duplicates
        registered_loops.insert(loop_key);

        using gtsam::symbol_shorthand::X;
        // Use robust noise model (Cauchy) like SC-PGO to handle potential outliers
        // Higher loop_noise_score = weaker constraint = more robust to outliers
        gtsam::Vector6 robust_noise_vec;
        robust_noise_vec << loop_noise_score, loop_noise_score, loop_noise_score, loop_noise_score, loop_noise_score, loop_noise_score;
        const auto base_noise = gtsam::noiseModel::Diagonal::Variances(robust_noise_vec);
        const auto robust_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1.0), base_noise);
        auto factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(submap1->id), X(submap2->id), gtsam::Pose3(T_origin1_origin2.matrix()), robust_noise);
        loop_factors.push_back(factor);
      }
    }
  }

  /**
   * @brief Find out the submap corresponding to the $frame_id
   *
   * @param frame_id           ID of the frame
   * @param T_origin_frame     [out] The frame's pose w.r.t. the submap origin
   * @return SubMap::ConstPtr  Corresponding submap
   */
  SubMap::ConstPtr find_submap(const int frame_id, Eigen::Isometry3d& T_origin_frame) {
    auto submap = std::lower_bound(submaps.begin(), submaps.end(), frame_id, [=](const SubMap::ConstPtr& submap, const int id) { return submap->frames.back()->id < id; });
    if (submap == submaps.end()) {
      return nullptr;
    }

    auto found = std::find_if((*submap)->frames.begin(), (*submap)->frames.end(), [=](const EstimationFrame::ConstPtr& frame) { return frame->id == frame_id; });
    if (found == (*submap)->frames.end()) {
      return nullptr;
    }

    const Eigen::Isometry3d T_world_origin = (*submap)->frames[(*submap)->frames.size() / 2]->T_world_sensor();
    T_origin_frame = T_world_origin.inverse() * (*found)->T_world_sensor();
    return (*submap);
  }

  /**
   * @brief Validate a loop candidate by applying scan matching
   *
   * @param frame1              Loop begin frame
   * @param frame2              Loop end frame
   * @param T_frame1_frame2     [in/out] Relative pose between frame1 and frame2
   * @return true               Loop candidate is valid
   * @return false              Loop candidate is false-positive
   */
  bool validate_loop(const gtsam_points::PointCloud::ConstPtr& frame1, const gtsam_points::PointCloud::ConstPtr& frame2, Eigen::Isometry3d& T_frame1_frame2) const {
    gtsam::Values values;
    values.insert(0, gtsam::Pose3::Identity());
    values.insert(1, gtsam::Pose3(T_frame1_frame2.matrix()));

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3::Identity(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

    auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(0, 1, frame1, frame2);
    factor->set_num_threads(icp_num_threads);
    graph.add(factor);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setlambdaInitial(1e-12);
    lm_params.setMaxIterations(icp_max_iterations);
    // LM optimization callback - use debug level to avoid terminal spam
    lm_params.callback = [this](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) { logger->debug(status.to_string()); };
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);
    values = optimizer.optimize();

    T_frame1_frame2 = Eigen::Isometry3d(values.at<gtsam::Pose3>(1).matrix());

    logger->debug("inliear_fraction={}", factor->inlier_fraction());

    return factor->inlier_fraction() > inlier_fraction_threshold;
  }

private:
  ConcurrentVector<EstimationFrame::ConstPtr> odom_frames_queue;
  ConcurrentVector<SubMap::ConstPtr> new_submaps_queue;

  ConcurrentVector<gtsam::NonlinearFactor::shared_ptr> loop_factors;

  int frame_count;
  std::unordered_map<int, int> frame_index_map;
  std::deque<std::tuple<int, int, double>> loop_candidates;

  std::vector<SubMap::ConstPtr> submaps;

  // Track registered loop closures to prevent duplicates
  std::set<std::pair<int, int>> registered_loops;

  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<SCManager> sc;

  // Configurable parameters
  double min_movement_threshold;
  double inlier_fraction_threshold;
  int icp_max_iterations;
  int icp_num_threads;
  double loop_noise_score;  // Higher = weaker constraint, more robust to outliers

  // Odometry-based dynamic ground estimation
  bool use_odometry_ground;      // Enable/disable odometry-based ground estimation
  double initial_lidar_height;   // Initial sensor height above ground (used with LIDAR_HEIGHT)
  bool ground_initialized;       // Flag to track if ground estimation has been initialized
  double initial_sensor_z;       // Z position of sensor at initialization

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::ScanContextLoopDetector();
}