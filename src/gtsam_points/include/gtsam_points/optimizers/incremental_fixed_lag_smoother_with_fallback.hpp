// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <atomic>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam/inference/Symbol.h>

namespace gtsam_points {

class IncrementalFixedLagSmootherExtWithFallback : public IncrementalFixedLagSmootherExt {
public:
  IncrementalFixedLagSmootherExtWithFallback(double smootherLag, const ISAM2Params& parameters);
  ~IncrementalFixedLagSmootherExtWithFallback() override;

  void set_fix_variable_types(const std::vector<std::pair<char, int>>& var_types) { fix_variable_types = var_types; }

  Result update(
    const gtsam::NonlinearFactorGraph& newFactors = gtsam::NonlinearFactorGraph(),
    const gtsam::Values& newTheta = gtsam::Values(),  //
    const KeyTimestampMap& timestamps = KeyTimestampMap(),
    const gtsam::FactorIndices& factorsToRemove = gtsam::FactorIndices()) override;

  bool fallbackHappened() const { return fallback_happend; }

  // Get the minimum valid key index for a given symbol character
  // Returns -1 if no keys exist for that character
  int getMinKeyIndex(char symbol_char) const {
    int min_idx = -1;
    for (const auto& kv : stamps) {
      gtsam::Symbol sym(kv.first);
      if (sym.chr() == symbol_char) {
        int idx = static_cast<int>(sym.index());
        if (min_idx < 0 || idx < min_idx) {
          min_idx = idx;
        }
      }
    }
    return min_idx;
  }

  gtsam::Values calculateEstimate() const override;

  const gtsam::Value& calculateEstimate(gtsam::Key key) const;

  template <class VALUE>
  VALUE calculateEstimate(Key key) const {
    // First check if key exists in our cached values (for marginalized keys)
    auto cached = values.find(key);

    try {
      const VALUE value = smoother->calculateEstimate<VALUE>(key);
      if (cached != values.end()) {
        values.insert_or_assign(key, value);
      }

      return value;
    } catch (const gtsam::ValuesKeyDoesNotExist& e) {
      // Key was marginalized out - return cached value if available
      std::cerr << "warning: key " << gtsam::Symbol(key) << " does not exist (marginalized), ";
      if (cached != values.end()) {
        std::cerr << "returning cached value" << std::endl;
        return cached->value.cast<VALUE>();
      }
      std::cerr << "no cached value available" << std::endl;
      throw;
    } catch (std::exception& e) {
      std::cerr << "warning: an exception was caught in fixed-lag smoother calculateEstimate!!" << std::endl;
      std::cerr << "       : " << e.what() << std::endl;

      fallback_smoother();

      try {
        const auto& value = smoother->calculateEstimate(key);
        if (cached != values.end()) {
          cached->value = value;
        }
        return value.cast<VALUE>();
      } catch (const gtsam::ValuesKeyDoesNotExist& e2) {
        // After fallback, key still doesn't exist - return cached value
        if (cached != values.end()) {
          std::cerr << "warning: returning cached value for " << gtsam::Symbol(key) << " after fallback" << std::endl;
          return cached->value.cast<VALUE>();
        }
        throw;
      } catch (std::exception& e2) {
        std::cerr << "error: exception after fallback in calculateEstimate!!" << std::endl;
        std::cerr << "     : " << e2.what() << std::endl;
        // Return cached value if available
        if (cached != values.end()) {
          return cached->value.cast<VALUE>();
        }
        throw;  // Re-throw if no cached value
      }
    }
  }

  const NonlinearFactorGraph& getFactors() const { return smoother->getFactors(); }
  const GaussianFactorGraph& getLinearFactors() const { return smoother->getLinearFactors(); }
  const Values& getLinearizationPoint() const { return smoother->getLinearizationPoint(); }
  const VectorValues& getDelta() const { return smoother->getDelta(); }

  Matrix marginalCovariance(Key key) const { return smoother->marginalCovariance(key); }

private:
  void update_fallback_state();
  void fallback_smoother() const;

private:
  double current_stamp;
  mutable gtsam::Values values;
  mutable gtsam::NonlinearFactorGraph factors;
  mutable std::atomic_bool fallback_happend;
  std::unordered_map<gtsam::Key, std::vector<gtsam::NonlinearFactor::shared_ptr>> factor_map;
  mutable gtsam::FixedLagSmootherKeyTimestampMap stamps;

  mutable std::unique_ptr<IncrementalFixedLagSmootherExt> smoother;

  std::vector<std::pair<char, int>> fix_variable_types;  // (chr, type)  type: 0=Pose3, 1=Point3
};
}  // namespace gtsam_points