#include "utils.hpp"

namespace utils
{

Config parse(json const& config_in)
{
  const auto config = Config{ .window_height = config_in["window_height"],
                              .window_width = config_in["window_width"],
                              .timestep_physics = config_in["timestep_physics"],
                              .framerate = config_in["framerate"],
                              .num_particles = config_in["num_particles"],
                              .particle_COR = config_in["particle_COR"],
                              .pixels_to_meters_ratio = config_in["pixels_to_meters_ratio"],
                              .gravity_flag = config_in["gravity_flag"],
                              .collision_check = config_in["collision_check"] };

  return config;
}

Eigen::Vector2d closestPointInLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end,
                                   const Eigen::Vector2d& point)
{
  // Calculate the line vector
  const Eigen::Vector2d difference_pos = line_end - line_start;

  // Calculate the vector from the beginning to the object
  const Eigen::Vector2d start_to_point = point - line_start;

  const double top = start_to_point.dot(difference_pos);
  const double bottom = difference_pos.squaredNorm();

  // Get the ratio of the projection of the point vector to the constraint vector and clamp it to the endpoints
  const double ratio = std::clamp(top / bottom, 0.0, 1.0);

  return { line_start(0) + difference_pos(0) * ratio, line_start(1) + difference_pos(1) * ratio };
}

}  // namespace utils