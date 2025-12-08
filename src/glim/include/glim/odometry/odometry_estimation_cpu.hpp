#pragma once

#include <glim/odometry/odometry_estimation_imu.hpp>

namespace gtsam_points {

class GaussianVoxelMapCPU;
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace small_gicp {
struct PointCloud;
struct AxisAlignedProjection;
template <typename PointCloud, typename Projection>
struct KdTree;
template <bool HasNormals, bool HasCovs>
struct FlatContainer;
using FlatContainerCov = FlatContainer<false, true>;
template <typename VoxelContents>
struct IncrementalVoxelMap;
}  // namespace small_gicp

namespace glim {

/**
 * @brief Parameters for OdometryEstimationCPU
 */
struct OdometryEstimationCPUParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  virtual ~OdometryEstimationCPUParams();

public:
  // Registration params
  std::string registration_type;    ///< Registration type (GICP, VGICP, or SmallGICP)
  int max_iterations;               ///< Maximum number of iterations
  int lru_thresh;                   ///< LRU cache threshold
  double target_downsampling_rate;  ///< Downsampling rate for points to be inserted into the target

  double ivox_resolution;  ///< iVox resolution (for GICP)
  double ivox_min_dist;    ///< Minimum distance between points in an iVox cell (for GICP)

  double vgicp_resolution;               ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;             ///< Multi-resolution voxelmap levesl (for VGICP)
  double vgicp_voxelmap_scaling_factor;  ///< Multi-resolution voxelmap scaling factor (for VGICP)

  // Small GICP params
  double small_gicp_max_correspondence_distance;  ///< Max correspondence distance for SmallGICP
  double small_gicp_rotation_eps;                 ///< Rotation tolerance for convergence [rad]
  double small_gicp_translation_eps;              ///< Translation tolerance for convergence
  double small_gicp_voxel_resolution;             ///< Voxel resolution for SmallGICP IncrementalVoxelMap
};

/**
 * @brief CPU-based semi-tightly coupled LiDAR-IMU odometry
 */
class OdometryEstimationCPU : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

private:
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const boost::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;

  virtual void fallback_smoother() override;

  void update_target(const int current, const Eigen::Isometry3d& T_target_imu);

private:
  // Registration params
  std::mt19937 mt;                                                                   ///< RNG
  Eigen::Isometry3d last_T_target_imu;                                               ///< Last IMU pose w.r.t. target model
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> target_voxelmaps;  ///< VGICP target voxelmap
  std::shared_ptr<gtsam_points::iVox> target_ivox;                                   ///< GICP target iVox
  EstimationFrame::ConstPtr target_ivox_frame;                                       ///< Target points (just for visualization)

  // Small GICP target model (using IncrementalVoxelMap with LRU-based memory management)
  std::shared_ptr<small_gicp::IncrementalVoxelMap<small_gicp::FlatContainerCov>> target_small_gicp_voxelmap;  ///< SmallGICP target voxelmap with LRU
};

}  // namespace glim
