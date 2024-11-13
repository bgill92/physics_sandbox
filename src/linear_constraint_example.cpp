#include <iostream>
#include <fstream>
#include <string>
#include <vector>

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
    constraints::LinearConstraint constraint_1{ 0, physics::State{ 2, 5, 0, 0, 0, 0 }, physics::State{ 8, 5, 0, 0, 0, 0 }};

    constraints::DistanceConstraint constraint_2{ 0, 1, 3 };

    Particle p1{ 0.25, 1, { 5, 5, 0, 0, 0, 0 }, sf::Color::Red };

    Particle p2{ 0.5, 10, { 2, 5, 0, 0, 0, 0 }, sf::Color::Red };

    std::vector<Object> objects;

    std::vector<constraints::Constraint> constraints;

    objects.push_back(p1);

    objects.push_back(p2);

    constraints.push_back(constraint_1);

    constraints.push_back(constraint_2);

    return { objects, constraints };
    // return { objects, {} };
  };

  auto simulator = Simulator(config, object_and_constraint_generator_func);

  simulator.run();

  return 0;
}