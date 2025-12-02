#include <glim_ext/rgb_colorizer.hpp>

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::RGBColorizerModule();
}
