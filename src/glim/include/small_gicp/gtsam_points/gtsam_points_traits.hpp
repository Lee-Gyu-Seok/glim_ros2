// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
// Adapter for gtsam_points::PointCloud to work with small_gicp
#pragma once

#include <gtsam_points/types/point_cloud.hpp>
#include <small_gicp/points/traits.hpp>

namespace small_gicp {

namespace traits {

/// @brief Traits for gtsam_points::PointCloud
template <>
struct Traits<gtsam_points::PointCloud> {
  using Points = gtsam_points::PointCloud;

  static size_t size(const Points& points) { return points.size(); }

  static bool has_points(const Points& points) { return points.has_points(); }
  static bool has_normals(const Points& points) { return points.has_normals(); }
  static bool has_covs(const Points& points) { return points.has_covs(); }

  static Eigen::Vector4d point(const Points& points, size_t i) { return points.points[i]; }
  static Eigen::Vector4d normal(const Points& points, size_t i) { return points.normals[i]; }
  static Eigen::Matrix4d cov(const Points& points, size_t i) { return points.covs[i]; }
};

/// @brief Traits for gtsam_points::PointCloud::ConstPtr (shared_ptr<const PointCloud>)
template <>
struct Traits<gtsam_points::PointCloud::ConstPtr> {
  using Points = gtsam_points::PointCloud::ConstPtr;

  static size_t size(const Points& points) { return points->size(); }

  static bool has_points(const Points& points) { return points->has_points(); }
  static bool has_normals(const Points& points) { return points->has_normals(); }
  static bool has_covs(const Points& points) { return points->has_covs(); }

  static Eigen::Vector4d point(const Points& points, size_t i) { return points->points[i]; }
  static Eigen::Vector4d normal(const Points& points, size_t i) { return points->normals[i]; }
  static Eigen::Matrix4d cov(const Points& points, size_t i) { return points->covs[i]; }
};

/// @brief Traits for gtsam_points::PointCloud* (raw pointer)
template <>
struct Traits<gtsam_points::PointCloud*> {
  using Points = gtsam_points::PointCloud*;

  static size_t size(const Points& points) { return points->size(); }

  static bool has_points(const Points& points) { return points->has_points(); }
  static bool has_normals(const Points& points) { return points->has_normals(); }
  static bool has_covs(const Points& points) { return points->has_covs(); }

  static Eigen::Vector4d point(const Points& points, size_t i) { return points->points[i]; }
  static Eigen::Vector4d normal(const Points& points, size_t i) { return points->normals[i]; }
  static Eigen::Matrix4d cov(const Points& points, size_t i) { return points->covs[i]; }
};

/// @brief Traits for const gtsam_points::PointCloud* (const raw pointer)
template <>
struct Traits<const gtsam_points::PointCloud*> {
  using Points = const gtsam_points::PointCloud*;

  static size_t size(const Points& points) { return points->size(); }

  static bool has_points(const Points& points) { return points->has_points(); }
  static bool has_normals(const Points& points) { return points->has_normals(); }
  static bool has_covs(const Points& points) { return points->has_covs(); }

  static Eigen::Vector4d point(const Points& points, size_t i) { return points->points[i]; }
  static Eigen::Vector4d normal(const Points& points, size_t i) { return points->normals[i]; }
  static Eigen::Matrix4d cov(const Points& points, size_t i) { return points->covs[i]; }
};

}  // namespace traits
}  // namespace small_gicp
