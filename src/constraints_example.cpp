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

  std::ifstream config_file{ config_file_path };

  const auto config = utils::parse(utils::json::parse(config_file));

  // create the window
  sf::RenderWindow window(sf::VideoMode(config.window_width, config.window_height), "Simulator");
  window.setFramerateLimit(60);

  physics::CircleConstraint constraint{ physics::State{ 5, 5, 0, 0, 0, 0 }, 3 };

  Particle p1{ 0.5, 1, { 2, 5, 0, 0, 0, 0 }, sf::Color::Red, constraint };

  // Particle p2{ 50, 100, { 800, 200, 0, -100, 0, 0 }, sf::Color::Green };

  std::vector<Object> objects;

  objects.push_back(p1);

  // objects.push_back(p2);

  // std::vector<physics::Constraint> constraints;

  // constraints.push_back(constraint);

  auto physics_manager = physics::PhysicsManager(config, objects);

  auto drawer_manager = graphics::DrawerManager(config, objects, window);

  // create a clock to track the elapsed time
  sf::Clock clock;

  // run the main loop
  while (window.isOpen())
  {
    // Get the start time of the loop
    const auto start_time = std::chrono::high_resolution_clock::now();

    // handle events
    sf::Event event;
    while (window.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    // draw it

    window.clear();

    for (size_t i = 0; i < objects.size(); i++)
    {
      physics_manager.step(i);

      drawer_manager.drawObject(i);
    }

    window.display();

    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in microseconds (or any other unit)
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  }

  return 0;
}