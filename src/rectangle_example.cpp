#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "constraints.hpp"
#include "common.hpp"
#include "physics.hpp"
#include "rectangle.hpp"
#include "simulator.hpp"
#include "utils.hpp"

#include <SFML/Graphics.hpp>

int main()
{
  // Parse the config file
  const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  std::ifstream config_file{ config_file_path };

  const auto config = utils::parse(utils::json::parse(config_file));

  const auto object_and_constraint_generator_func =
      [](const Config& config) -> std::pair<std::vector<Object>, std::vector<constraints::Constraint>> {
    std::vector<Object> objects;

    // const auto angle_rad = 30*deg2rad;
    const auto angle_rad = 30 * deg2rad;

    std::cout << "angle_rad: " << angle_rad << "\n";

    // Rectangle rectangle {2, 0.2, 1, {5, 5, angle_rad, 0, 0, 0}, sf::Color::Red};
    Rectangle rectangle{ 2, 1, 1, { 5, 5, angle_rad, 0, 0, 0 }, sf::Color::Red };

    objects.push_back(rectangle);

    return { objects, {} };
  };

  auto simulator = Simulator(config, object_and_constraint_generator_func);

  simulator.run();

  return 0;
}
