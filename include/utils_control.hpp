#pragma once

#include "common.hpp"

#include "json.hpp"

namespace control
{
struct Config_PID
{
  size_t object_idx = 0;
  double timestep = 0.1;
  double reference_value = 1.57079632679;
  double Kp = 1.0;
  double Ki = 0.0;
  double Kd = 0.0;
};
}  // namespace control

namespace utils
{

using json = nlohmann::json;

/**
 * @brief      Parses the configuration from a Json file
 *
 * @param      config_in  The configuration in
 *
 * @return     The Config struct generated
 */
control::Config_PID parse_PID(json const& config_in);

}  // namespace utils