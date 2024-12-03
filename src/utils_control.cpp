#include "utils_control.hpp"

namespace utils
{

control::Config_PID parse_PID(json const& config_in)
{
  const auto config = control::Config_PID{ .object_idx = config_in["object_idx"],
                                           .timestep = config_in["timestep"],
                                           .reference_value = config_in["reference_value"],
                                           .Kp = config_in["Kp"],
                                           .Ki = config_in["Ki"],
                                           .Kd = config_in["Kd"] };

  return config;
}

}  // namespace utils