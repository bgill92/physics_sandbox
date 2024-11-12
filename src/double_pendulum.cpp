#include <cmath>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include "common.hpp"
#include "constraints.hpp"
#include "physics.hpp"
#include "particle.hpp"
#include "simulator.hpp"
#include "utils.hpp"

namespace
{

std::pair<std::vector<Object>, std::vector<constraints::Constraint>>
generateDoublePendulum(const physics::State& center, const double mass_1, const double mass_2, const double radius_1,
                       const double radius_2, const double angle_1, const double angle_2, const double length_1,
                       const double length_2)
{
  std::vector<Object> objects;
  std::vector<constraints::Constraint> constraints;

  const auto x_start = center(physics::STATE_X_POS_IDX);
  const auto y_start = center(physics::STATE_Y_POS_IDX);

  const auto deg2rad = std::numbers::pi / 180;

  const auto angle_1_adjusted = (angle_1 - 90);
  const auto x_1 = x_start + length_1 * std::cos(angle_1_adjusted * deg2rad);
  const auto y_1 = y_start + length_1 * std::sin(angle_1_adjusted * deg2rad);

  const auto angle_2_adjusted = angle_1_adjusted + angle_2;
  const auto x_2 = x_1 + length_2 * std::cos((angle_2_adjusted)*deg2rad);
  const auto y_2 = y_1 + length_2 * std::sin((angle_2_adjusted)*deg2rad);

  Particle p1{ radius_1, mass_1, { x_1, y_1, 0, 0, 0, 0 }, sf::Color::Red };

  Particle p2{ radius_2, mass_2, { x_2, y_2, 0, 0, 0, 0 }, sf::Color::Green };

  objects.push_back(p1);

  objects.push_back(p2);

  constraints::CircleConstraint constraint_1{ 0, center, length_1 };

  constraints::DistanceConstraint constraint_2{ 0, 1, length_2 };

  constraints.push_back(constraint_1);

  constraints.push_back(constraint_2);

  return { objects, constraints };
}

}  // namespace

int main()
{
  // Get config
  // Parse the config file
  const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  std::ifstream config_file{ config_file_path };

  const auto config = utils::parse(utils::json::parse(config_file));

  const auto object_and_constraint_generator_func =
      [&](const Config& config) -> std::pair<std::vector<Object>, std::vector<constraints::Constraint>> {
    physics::State center{ 5, 5, 0, 0, 0, 0 };
    const auto mass_1 = 1.0;
    const auto mass_2 = 2.0;
    const auto radius_1 = 0.25;
    const auto radius_2 = 0.4;
    const auto angle_1 = 150;
    const auto angle_2 = 30;
    const auto length_1 = 1.0;
    const auto length_2 = 2.0;

    return generateDoublePendulum(center, mass_1, mass_2, radius_1, radius_2, angle_1, angle_2, length_1, length_2);
  };

  auto simulator = Simulator(config, object_and_constraint_generator_func);

  simulator.run();

  return 0;
}