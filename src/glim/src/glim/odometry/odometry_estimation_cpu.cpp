#include <glim/odometry/odometry_estimation_cpu.hpp>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <glim/util/config.hpp>
#include <glim/util/profiler.hpp>
#include <glim/common/imu_integration.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/common/cloud_covariance_estimation.hpp>

#include <glim/odometry/callbacks.hpp>

// small_gicp headers for CPU-based GICP (header-only)
#include <small_gicp/ann/flat_container.hpp>
#include <small_gicp/ann/incremental_voxelmap.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/points/point_cloud.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationCPUParams::OdometryEstimationCPUParams() : OdometryEstimationIMUParams() {
  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  registration_type = config.param<std::string>("odometry_estimation", "registration_type", "VGICP");
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 5);
  lru_thresh = config.param<int>("odometry_estimation", "lru_thresh", 100);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 0.1);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 0.5);
  ivox_min_dist = config.param<double>("odometry_estimation", "ivox_min_dist", 0.1);

  vgicp_resolution = config.param<double>("odometry_estimation", "vgicp_resolution", 0.2);
  vgicp_voxelmap_levels = config.param<int>("odometry_estimation", "vgicp_voxelmap_levels", 2);
  vgicp_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "vgicp_voxelmap_scaling_factor", 2.0);

  // Small GICP params
  small_gicp_max_correspondence_distance = config.param<double>("odometry_estimation", "small_gicp_max_correspondence_distance", 1.0);
  small_gicp_rotation_eps = config.param<double>("odometry_estimation", "small_gicp_rotation_eps", 0.1) * M_PI / 180.0;  // degrees to radians
  small_gicp_translation_eps = config.param<double>("odometry_estimation", "small_gicp_translation_eps", 1e-3);
  small_gicp_voxel_resolution = config.param<double>("odometry_estimation", "small_gicp_voxel_resolution", 0.5);
}

OdometryEstimationCPUParams::~OdometryEstimationCPUParams() {}

OdometryEstimationCPU::OdometryEstimationCPU(const OdometryEstimationCPUParams& params) : OdometryEstimationIMU(std::make_unique<OdometryEstimationCPUParams>(params)) {
  last_T_target_imu.setIdentity();
  if (params.registration_type == "GICP") {
    target_ivox.reset(new gtsam_points::iVox(params.ivox_resolution));
    target_ivox->voxel_insertion_setting().set_min_dist_in_cell(params.ivox_min_dist);
    target_ivox->set_lru_horizon(params.lru_thresh);
    target_ivox->set_neighbor_voxel_mode(1);
  } else if (params.registration_type == "VGICP") {
    target_voxelmaps.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps[i] = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      target_voxelmaps[i]->set_lru_horizon(params.lru_thresh);
    }
  } else if (params.registration_type == "SmallGICP") {
    // SmallGICP uses IncrementalVoxelMap with LRU-based memory management
    target_small_gicp_voxelmap = std::make_shared<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerCov>>(params.small_gicp_voxel_resolution);
    target_small_gicp_voxelmap->lru_horizon = params.lru_thresh;
    target_small_gicp_voxelmap->lru_clear_cycle = 10;
    target_small_gicp_voxelmap->set_search_offsets(7);  // Use 7-neighbor search for better correspondence
    spdlog::info("Using SmallGICP registration with IncrementalVoxelMap: voxel_res={}, lru_horizon={}, max_corr_dist={}, rot_eps={:.4f}deg, trans_eps={}",
                 params.small_gicp_voxel_resolution,
                 params.lru_thresh,
                 params.small_gicp_max_correspondence_distance,
                 params.small_gicp_rotation_eps * 180.0 / M_PI,
                 params.small_gicp_translation_eps);
  } else {
    spdlog::error("unknown registration type for odometry_estimation_cpu ({})", params.registration_type);
    abort();
  }
}

OdometryEstimationCPU::~OdometryEstimationCPU() {}

gtsam::NonlinearFactorGraph OdometryEstimationCPU::create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  const int last = current - 1;

  if (current == 0) {
    last_T_target_imu = frames[current]->T_world_imu;
    update_target(current, frames[current]->T_world_imu);
    return gtsam::NonlinearFactorGraph();
  }

  const Eigen::Isometry3d pred_T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
  const Eigen::Isometry3d pred_T_target_imu = last_T_target_imu * pred_T_last_current;

  gtsam::Values values;
  values.insert(X(current), gtsam::Pose3(pred_T_target_imu.matrix()));

  Eigen::Isometry3d T_target_imu;

  // SmallGICP uses its own registration pipeline with IncrementalVoxelMap
  if (params->registration_type == "SmallGICP") {
    // Convert current frame to small_gicp::PointCloud format
    auto source = std::make_shared<small_gicp::PointCloud>();
    source->resize(frames[current]->frame->size());
    for (size_t i = 0; i < frames[current]->frame->size(); i++) {
      source->point(i) = frames[current]->frame->points[i];
      if (frames[current]->frame->has_covs()) {
        source->cov(i) = frames[current]->frame->covs[i];
      }
    }

    // Use header-only Registration class with GICP factor
    // IncrementalVoxelMap provides both target points and nearest neighbor search
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
    registration.criteria.rotation_eps = params->small_gicp_rotation_eps;
    registration.criteria.translation_eps = params->small_gicp_translation_eps;
    registration.rejector.max_dist_sq = params->small_gicp_max_correspondence_distance * params->small_gicp_max_correspondence_distance;
    registration.optimizer.max_iterations = params->max_iterations;
    registration.reduction.num_threads = params->num_threads;

    // Align using small_gicp Registration class with IncrementalVoxelMap as both target and search structure
    auto result = registration.align(*target_small_gicp_voxelmap, *source, *target_small_gicp_voxelmap, pred_T_target_imu);

    T_target_imu = result.T_target_source;
  } else {
    // Create frame-to-model matching factor (GICP/VGICP)
    gtsam::NonlinearFactorGraph matching_cost_factors;
    if (params->registration_type == "GICP") {
      auto gicp_factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(
        gtsam::Pose3(),
        X(current),
        target_ivox,
        frames[current]->frame,
        target_ivox);
      gicp_factor->set_max_correspondence_distance(params->ivox_resolution * 2.0);
      gicp_factor->set_num_threads(params->num_threads);
      matching_cost_factors.add(gicp_factor);
    } else if (params->registration_type == "VGICP") {
      for (const auto& voxelmap : target_voxelmaps) {
        auto vgicp_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
        vgicp_factor->set_num_threads(params->num_threads);
        matching_cost_factors.add(vgicp_factor);
      }
    }

    gtsam::NonlinearFactorGraph graph;
    graph.add(matching_cost_factors);

    gtsam_points::LevenbergMarquardtExtParams lm_params;
    lm_params.setMaxIterations(params->max_iterations);
    lm_params.setAbsoluteErrorTol(0.1);

    gtsam::Pose3 last_estimate = values.at<gtsam::Pose3>(X(current));
    lm_params.termination_criteria = [&](const gtsam::Values& values) {
      const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
      const gtsam::Pose3 delta = last_estimate.inverse() * current_pose;

      const double delta_t = delta.translation().norm();
      const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
      last_estimate = current_pose;

      if (delta_t < 1e-10 && delta_r < 1e-10) {
        // Maybe failed to solve the linear system
        return false;
      }

      // Convergence check
      return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
    };

    // Optimize
    // lm_params.setDiagonalDamping(true);
    gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
    arena->execute([&] {
#endif
      values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
    });
#endif

    T_target_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  }
  Eigen::Isometry3d T_last_current = last_T_target_imu.inverse() * T_target_imu;
  T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  frames[current]->T_world_imu = frames[last]->T_world_imu * T_last_current;
  new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  gtsam::NonlinearFactorGraph factors;

  // Get linearized matching cost factors
  // const auto linearized = optimizer.last_linearized();
  // for (int i = linearized->size() - matching_cost_factors.size(); i < linearized->size(); i++) {
  //   factors.emplace_shared<gtsam::LinearContainerFactor>(linearized->at(i), values);
  // }

  // TODO: Extract a relative pose covariance from a frame-to-model matching result?
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(T_target_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, 1e3));

  update_target(current, T_target_imu);
  last_T_target_imu = T_target_imu;

  return factors;
}

void OdometryEstimationCPU::fallback_smoother() {}

void OdometryEstimationCPU::update_target(const int current, const Eigen::Isometry3d& T_target_imu) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  auto frame = frames[current]->frame;
  if (current >= 5) {
    frame = gtsam_points::random_sampling(frames[current]->frame, params->target_downsampling_rate, mt);
  }

  auto transformed = gtsam_points::transform(frame, T_target_imu);
  if (params->registration_type == "GICP") {
    target_ivox->insert(*transformed);
  } else if (params->registration_type == "VGICP") {
    for (auto& target_voxelmap : target_voxelmaps) {
      target_voxelmap->insert(*transformed);
    }
  } else if (params->registration_type == "SmallGICP") {
    // For SmallGICP, use IncrementalVoxelMap with LRU-based memory management
    // Convert to small_gicp::PointCloud format before inserting
    auto source_for_insert = std::make_shared<small_gicp::PointCloud>();
    source_for_insert->resize(transformed->size());
    for (size_t i = 0; i < transformed->size(); i++) {
      source_for_insert->point(i) = transformed->points[i];
      if (transformed->has_covs()) {
        source_for_insert->cov(i) = transformed->covs[i];
      }
    }
    // The voxelmap handles old voxel cleanup automatically via LRU
    target_small_gicp_voxelmap->insert(*source_for_insert);
  }

  // Update target point cloud (just for visualization).
  // This is not necessary for mapping and can safely be removed.
  if (frames.size() % 50 == 0) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = frames.size() - 1;
    frame->stamp = frames.back()->stamp;

    frame->T_lidar_imu = frames.back()->T_lidar_imu;
    frame->T_world_lidar = frame->T_lidar_imu.inverse();
    frame->T_world_imu.setIdentity();

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    frame->frame_id = FrameID::IMU;

    if (params->registration_type == "GICP") {
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_ivox->voxel_points());
    } else if (params->registration_type == "VGICP") {
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_voxelmaps[0]->voxel_points());
    } else if (params->registration_type == "SmallGICP" && target_small_gicp_voxelmap) {
      // Convert small_gicp voxelmap points to gtsam_points format for visualization
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(small_gicp::traits::voxel_points(*target_small_gicp_voxelmap));
    }

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);

    if (target_ivox_frame) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {target_ivox_frame};
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    }

    target_ivox_frame = frame;
  }
}

}  // namespace glim
