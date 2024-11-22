#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "common.hpp"
#include "constraints.hpp"
#include "physics.hpp"
#include "particle.hpp"
#include "simulator.hpp"
#include "utils.hpp"

#include <SFML/Graphics.hpp>

int main()
{
  // Get config
  // Parse the config file
  const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  std::ifstream config_file{ config_file_path };

  const auto config = utils::parse(utils::json::parse(config_file));

  const auto object_and_constraint_generator_func =
      [](const Config& config) -> std::pair<std::vector<Object>, std::vector<constraints::Constraint>> {
    constraints::LinearConstraint constraint_1{ 0, physics::State{ 2, 5, 0, 0, 0, 0 },
                                                physics::State{ 8, 5, 0, 0, 0, 0 } };

    const Eigen::Vector2d v1{ 0.0, 0 };

    const Eigen::Vector2d v2{ -1.4, 0 };

    constraints::DistanceConstraint constraint_2{ 0, v1, 1, v2, 0 };

    Rectangle r1{ 0.5, 0.2, 1, { 5, 5, 0, 0, 0, 0 }, sf::Color::Red };

    Rectangle r2{ 3, 0.1, 1, { 5, 6.4, 80 * deg2rad, 0, 0, 0 }, sf::Color::Green };

    std::vector<Object> objects;

    std::vector<constraints::Constraint> constraints;

    objects.push_back(r1);

    objects.push_back(r2);

    constraints.push_back(constraint_1);

    constraints.push_back(constraint_2);

    return { objects, constraints };
    // return { objects, {} };
  };

  auto simulator = Simulator(config, object_and_constraint_generator_func, 0);

  simulator.run();

  return 0;
}