#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>
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
    constraints::CircleConstraint constraint{ 0, physics::State{ 5, 5, 0, 0, 0, 0 }, 3 };

    Particle p1{ 0.5, 1, { 2, 5, 0, 0, -1, 0 }, sf::Color::Red };

    std::vector<Object> objects;

    std::vector<constraints::Constraint> constraints;

    objects.push_back(p1);

    // Particle p2{ 0.5, 100, { 5, 2, 0, 0, 0, 0 }, sf::Color::Green };

    // objects.push_back(p2);

    constraints.push_back(constraint);

    return { objects, constraints };
  };

  auto simulator = Simulator(config, object_and_constraint_generator_func);

  simulator.run();

  return 0;
}