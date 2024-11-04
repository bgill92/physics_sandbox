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
#include "particle.hpp"
#include "simulator.hpp"
#include "utils.hpp"

#include <SFML/Graphics.hpp>

int main()
{
  // Parse the config file
  const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  std::ifstream config_file{ config_file_path };

  const auto config = utils::parse(utils::json::parse(config_file));

  const auto object_generator_func = [](const Config& config) -> std::vector<Object> {
    return generateParticles(config);
  };

  auto simulator = Simulator(config, object_generator_func);

  simulator.run();

  return 0;
}
