#pragma once

#include "common.hpp"

#include "json.hpp"

namespace utils
{

	using json = nlohmann::json;

	Config parse(json const& config_in)
	{
	  const auto config = Config{.window_height = config_in["window_height"],
	                             .window_width = config_in["window_width"],
	                             .timestep_physics = config_in["timestep_physics"],
	                             .num_particles = config_in["num_particles"],
	                             .particle_COR = config_in["particle_COR"],
	                             .pixels_to_meters_ratio = config_in["pixels_to_meters_ratio"],
	                             .gravity_flag = config_in["gravity_flag"]};

	  return config;
}
	
}