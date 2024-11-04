#pragma once

#include "common.hpp"

#include "json.hpp"

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
Config parse(json const& config_in)
{
  const auto config = Config{ .window_height = config_in["window_height"],
                              .window_width = config_in["window_width"],
                              .timestep_physics = config_in["timestep_physics"],
                              .framerate = config_in["framerate"],
                              .num_particles = config_in["num_particles"],
                              .particle_COR = config_in["particle_COR"],
                              .pixels_to_meters_ratio = config_in["pixels_to_meters_ratio"],
                              .gravity_flag = config_in["gravity_flag"] };

  return config;
}

}  // namespace utils