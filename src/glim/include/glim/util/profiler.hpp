#pragma once

#include <mutex>
#include <string>
#include <chrono>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <algorithm>
#include <numeric>

namespace glim {

/**
 * @brief Thread-safe profiler for measuring computational times
 *
 * Usage:
 *   Profiler::instance().start("odometry_icp");
 *   // ... computation ...
 *   Profiler::instance().stop("odometry_icp");
 *
 * Or use RAII helper:
 *   {
 *     ProfilerScope scope("odometry_icp");
 *     // ... computation ...
 *   }
 */
class Profiler {
public:
  struct TimingStats {
    int count = 0;
    double total_ms = 0.0;
    double min_ms = std::numeric_limits<double>::max();
    double max_ms = 0.0;
    std::vector<double> samples;  // Store recent samples for percentile calculation
    static constexpr int MAX_SAMPLES = 1000;

    void add(double ms) {
      count++;
      total_ms += ms;
      min_ms = std::min(min_ms, ms);
      max_ms = std::max(max_ms, ms);
      if (samples.size() < MAX_SAMPLES) {
        samples.push_back(ms);
      }
    }

    double avg_ms() const { return count > 0 ? total_ms / count : 0.0; }

    double percentile(double p) const {
      if (samples.empty()) return 0.0;
      std::vector<double> sorted = samples;
      std::sort(sorted.begin(), sorted.end());
      int idx = static_cast<int>(p * (sorted.size() - 1));
      return sorted[idx];
    }
  };

  static Profiler& instance() {
    static Profiler profiler;
    return profiler;
  }

  void start(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    start_times_[name] = std::chrono::high_resolution_clock::now();
  }

  void stop(const std::string& name) {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = start_times_.find(name);
    if (it != start_times_.end()) {
      double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - it->second).count();
      stats_[name].add(elapsed_ms);
      start_times_.erase(it);
    }
  }

  void record(const std::string& name, double elapsed_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_[name].add(elapsed_ms);
  }

  TimingStats get_stats(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = stats_.find(name);
    if (it != stats_.end()) {
      return it->second;
    }
    return TimingStats();
  }

  std::unordered_map<std::string, TimingStats> get_all_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
  }

  void save(const std::string& path) const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::ofstream ofs(path + "/profiling_stats.txt");
    if (!ofs.is_open()) return;

    ofs << "# GLIM Profiling Statistics Report" << std::endl;
    ofs << "# =================================" << std::endl;
    ofs << std::endl;
    ofs << "# Format: name | count | total(ms) | avg(ms) | min(ms) | max(ms) | p50(ms) | p95(ms) | p99(ms)" << std::endl;
    ofs << std::endl;

    // Collect and sort by category
    std::vector<std::pair<std::string, TimingStats>> sorted_stats(stats_.begin(), stats_.end());
    std::sort(sorted_stats.begin(), sorted_stats.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (const auto& [name, stat] : sorted_stats) {
      if (stat.count == 0) continue;

      ofs << std::fixed;
      ofs << name << std::endl;
      ofs << "  count:    " << stat.count << std::endl;
      ofs << "  total:    " << std::setprecision(1) << stat.total_ms << " ms" << std::endl;
      ofs << "  average:  " << std::setprecision(3) << stat.avg_ms() << " ms" << std::endl;
      ofs << "  min:      " << std::setprecision(3) << stat.min_ms << " ms" << std::endl;
      ofs << "  max:      " << std::setprecision(3) << stat.max_ms << " ms" << std::endl;
      ofs << "  p50:      " << std::setprecision(3) << stat.percentile(0.50) << " ms" << std::endl;
      ofs << "  p95:      " << std::setprecision(3) << stat.percentile(0.95) << " ms" << std::endl;
      ofs << "  p99:      " << std::setprecision(3) << stat.percentile(0.99) << " ms" << std::endl;
      ofs << std::endl;
    }

    // Summary by category
    ofs << "# Summary by Category" << std::endl;
    ofs << "# -------------------" << std::endl;

    std::unordered_map<std::string, double> category_totals;
    for (const auto& [name, stat] : stats_) {
      std::string category = name.substr(0, name.find('/'));
      category_totals[category] += stat.total_ms;
    }

    for (const auto& [category, total] : category_totals) {
      ofs << category << ": " << std::setprecision(1) << total << " ms ("
          << std::setprecision(2) << total / 1000.0 << " sec)" << std::endl;
    }

    ofs.close();
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.clear();
    start_times_.clear();
  }

private:
  Profiler() = default;

  mutable std::mutex mutex_;
  std::unordered_map<std::string, TimingStats> stats_;
  std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> start_times_;
};

/**
 * @brief RAII helper for automatic profiling
 */
class ProfilerScope {
public:
  explicit ProfilerScope(const std::string& name) : name_(name) {
    Profiler::instance().start(name_);
  }

  ~ProfilerScope() {
    Profiler::instance().stop(name_);
  }

  ProfilerScope(const ProfilerScope&) = delete;
  ProfilerScope& operator=(const ProfilerScope&) = delete;

private:
  std::string name_;
};

// Convenience macros
#define GLIM_PROFILE_SCOPE(name) glim::ProfilerScope _profiler_scope_##__LINE__(name)
#define GLIM_PROFILE_START(name) glim::Profiler::instance().start(name)
#define GLIM_PROFILE_STOP(name) glim::Profiler::instance().stop(name)

}  // namespace glim
