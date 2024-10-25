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
#include "utils.hpp"

#include <SFML/Graphics.hpp>

int main()
{
	// Get config
  // Parse the config file
  const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  std::ifstream config_file{config_file_path};

  const auto config = utils::parse(utils::json::parse(config_file));

  // create the window
  sf::RenderWindow window(sf::VideoMode(config.window_width, config.window_height), "Simulator");
  window.setFramerateLimit(60);

	return 0; 
}