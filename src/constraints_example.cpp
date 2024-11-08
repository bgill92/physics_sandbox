#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "common.hpp"
#include "physics.hpp"
#include "particle.hpp"
#include "simulator.hpp"
#include "utils.hpp"

#include <SFML/Graphics.hpp>

int main()
{
  // // Get config
  // // Parse the config file
  // const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  // std::ifstream config_file{ config_file_path };

  // const auto config = utils::parse(utils::json::parse(config_file));

  // const auto object_generator_func = [](const Config& config) -> std::vector<Object> {
  //   physics::CircleConstraint constraint{ physics::State{ 5, 5, 0, 0, 0, 0 }, 3 };

  //   Particle p1{ 0.5, 1, { 2, 5, 0, 0, 0, 0 }, sf::Color::Red, constraint };

  //   // Particle p2{ 50, 100, { 800, 200, 0, -100, 0, 0 }, sf::Color::Green };

  //   std::vector<Object> objects;

  //   objects.push_back(p1);

  //   return objects;
  // };

  // auto simulator = Simulator(config, object_generator_func);

  // simulator.run();

  return 0;
}