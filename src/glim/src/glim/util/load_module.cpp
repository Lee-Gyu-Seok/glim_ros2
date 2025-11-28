#include <glim/util/load_module.hpp>

#include <dlfcn.h>
#include <spdlog/spdlog.h>

namespace glim {

bool check_so_exists(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle != nullptr) {
    dlclose(handle);
    return true;
  }
  return false;
}

std::string resolve_auto_module(const std::string& so_name, const std::string& gpu_so, const std::string& cpu_so) {
  if (so_name != "auto") {
    return so_name;
  }

  // Try GPU first, fallback to CPU
  if (check_so_exists(gpu_so)) {
    spdlog::info("Auto-detected GPU module: {}", gpu_so);
    return gpu_so;
  } else {
    spdlog::info("GPU module not found, using CPU module: {}", cpu_so);
    return cpu_so;
  }
}

void open_so(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("failed to open {}", so_name);
    spdlog::warn("{}", dlerror());
  }
}

void* load_symbol(const std::string& so_name, const std::string& symbol_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("failed to open {}", so_name);
    spdlog::warn("{}", dlerror());
    return nullptr;
  }

  auto* func = dlsym(handle, symbol_name.c_str());
  if (func == nullptr) {
    spdlog::warn("failed to find symbol={} in {}", symbol_name, so_name);
    spdlog::warn("{}", dlerror());
  }

  return func;
}

}  // namespace glim
