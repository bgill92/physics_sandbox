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
generateNPendulum(const physics::State& center, const std::vector<double>& masses, const std::vector<double>& radii,
                  const std::vector<double>& angles, const std::vector<double>& lengths)
{
  std::vector<Object> objects;
  std::vector<constraints::Constraint> constraints;

  const auto x_start = center(physics::STATE_X_POS_IDX);
  const auto y_start = center(physics::STATE_Y_POS_IDX);

  const auto deg2rad = std::numbers::pi / 180;

  const auto size = masses.size();

  if (size != radii.size())
  {
    std::cout << "Sizes are not the same\n";
    exit(0);
  }
  if (size != angles.size())
  {
    std::cout << "Sizes are not the same\n";
    exit(0);
  }
  if (size != lengths.size())
  {
    std::cout << "Sizes are not the same\n";
    exit(0);
  }

  double angle_adjusted;
  double x = x_start;
  double y = y_start;

  for (size_t idx = 0; idx < masses.size(); idx++)
  {
    if (idx == 0)
    {
      angle_adjusted = angles.at(0) - 90;
    }
    else
    {
      angle_adjusted += angles.at(idx);
    }

    x += lengths.at(idx) * std::cos(angle_adjusted * deg2rad);
    y += lengths.at(idx) * std::sin(angle_adjusted * deg2rad);

    Particle p{ radii.at(idx), masses.at(idx), { x, y, 0, 0, 0, 0 }, sf::Color::Red };

    objects.push_back(p);

    if (idx == 0)
    {
      constraints::CircleConstraint constraint{ 0, center, lengths.at(idx) };
      constraints.push_back(constraint);
    }
    else
    {
      constraints::DistanceConstraint constraint{ idx - 1, idx, lengths.at(idx) };
      constraints.push_back(constraint);
    }
  }

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

    // const std::vector<double> masses  {1.0, 2.0, 3.0};
    // const std::vector<double> radii   {0.1, 0.2, 0.3};
    // const std::vector<double> angles  {120, 30, 30};
    // const std::vector<double> lengths {1.0, 1.0, 1.0};

    const int n = 50;

    const auto masses = std::vector<double>(n, 1.0);
    const auto radii = std::vector<double>(n, 0.05);
    auto angles = std::vector<double>(n, 2);
    angles.at(0) = 90;
    const auto lengths = std::vector<double>(n, 0.1);

    return generateNPendulum(center, masses, radii, angles, lengths);
  };

  auto simulator = Simulator(config, object_and_constraint_generator_func);

  simulator.run();

  return 0;
}