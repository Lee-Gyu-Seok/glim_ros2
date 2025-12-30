#pragma once

#include <mutex>
#include <thread>
#include <string>
#include <chrono>
#include <vector>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <algorithm>
#include <numeric>
#include <map>

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
 *
 * Thread naming:
 *   Profiler::instance().set_thread_name("scan_context");
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

  // Set the name for the current thread (call once at thread start)
  void set_thread_name(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    thread_names_[std::this_thread::get_id()] = name;
  }

  // Get the name of the current thread
  std::string get_thread_name() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = thread_names_.find(std::this_thread::get_id());
    if (it != thread_names_.end()) {
      return it->second;
    }
    return "main";
  }

  void start(const std::string& name) {
    auto thread_id = std::this_thread::get_id();
    std::lock_guard<std::mutex> lock(mutex_);
    auto key = std::make_pair(thread_id, name);
    start_times_[key] = std::chrono::high_resolution_clock::now();
  }

  void stop(const std::string& name) {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto thread_id = std::this_thread::get_id();
    std::lock_guard<std::mutex> lock(mutex_);

    auto key = std::make_pair(thread_id, name);
    auto it = start_times_.find(key);
    if (it != start_times_.end()) {
      double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - it->second).count();
      stats_[key].add(elapsed_ms);
      start_times_.erase(it);
    }
  }

  void record(const std::string& name, double elapsed_ms) {
    auto thread_id = std::this_thread::get_id();
    std::lock_guard<std::mutex> lock(mutex_);
    auto key = std::make_pair(thread_id, name);
    stats_[key].add(elapsed_ms);
  }

  TimingStats get_stats(const std::string& name) const {
    auto thread_id = std::this_thread::get_id();
    std::lock_guard<std::mutex> lock(mutex_);
    auto key = std::make_pair(thread_id, name);
    auto it = stats_.find(key);
    if (it != stats_.end()) {
      return it->second;
    }
    return TimingStats();
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

    // Group stats by thread name
    std::map<std::string, std::vector<std::pair<std::string, TimingStats>>> thread_stats;

    for (const auto& [key, stat] : stats_) {
      const auto& thread_id = key.first;
      const auto& name = key.second;

      std::string thread_name = "main";
      auto name_it = thread_names_.find(thread_id);
      if (name_it != thread_names_.end()) {
        thread_name = name_it->second;
      }

      thread_stats[thread_name].emplace_back(name, stat);
    }

    // Sort thread names for consistent output
    std::vector<std::string> sorted_thread_names;
    for (const auto& [thread_name, _] : thread_stats) {
      sorted_thread_names.push_back(thread_name);
    }
    std::sort(sorted_thread_names.begin(), sorted_thread_names.end());

    // Define processing order for main thread (pipeline order)
    auto get_order = [](const std::string& name) -> int {
      if (name == "preprocess") return 0;
      if (name == "odometry") return 1;
      if (name == "sub_mapping") return 2;
      if (name == "global_mapping") return 3;
      return 100;  // Unknown items go to the end
    };

    // Output stats grouped by thread
    for (const auto& thread_name : sorted_thread_names) {
      auto& stats = thread_stats[thread_name];

      // Sort by processing order for main thread, alphabetically for others
      std::sort(stats.begin(), stats.end(),
                [&get_order, &thread_name](const auto& a, const auto& b) {
                  if (thread_name == "main") {
                    int order_a = get_order(a.first);
                    int order_b = get_order(b.first);
                    if (order_a != order_b) return order_a < order_b;
                  }
                  return a.first < b.first;
                });

      ofs << "# ----------------------------------------" << std::endl;
      ofs << "# Thread: " << thread_name << std::endl;
      ofs << "# ----------------------------------------" << std::endl;
      ofs << std::endl;

      for (const auto& [name, stat] : stats) {
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
    }

    // Summary by thread
    ofs << "# ========================================" << std::endl;
    ofs << "# Summary by Thread" << std::endl;
    ofs << "# ========================================" << std::endl;

    for (const auto& thread_name : sorted_thread_names) {
      double thread_total = 0.0;
      for (const auto& [name, stat] : thread_stats[thread_name]) {
        thread_total += stat.total_ms;
      }
      ofs << thread_name << ": " << std::setprecision(1) << thread_total << " ms ("
          << std::setprecision(2) << thread_total / 1000.0 << " sec)" << std::endl;
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

  // Hash function for pair<thread::id, string>
  struct PairHash {
    std::size_t operator()(const std::pair<std::thread::id, std::string>& p) const {
      auto h1 = std::hash<std::thread::id>{}(p.first);
      auto h2 = std::hash<std::string>{}(p.second);
      return h1 ^ (h2 << 1);
    }
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::thread::id, std::string> thread_names_;
  std::unordered_map<std::pair<std::thread::id, std::string>, TimingStats, PairHash> stats_;
  std::unordered_map<std::pair<std::thread::id, std::string>, std::chrono::high_resolution_clock::time_point, PairHash> start_times_;
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
#define GLIM_PROFILE_SET_THREAD_NAME(name) glim::Profiler::instance().set_thread_name(name)

}  // namespace glim
