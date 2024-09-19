#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "physics.hpp"
#include "object.hpp"
#include "particle.hpp"

#include <SFML/Graphics.hpp>

namespace
{
const unsigned int WINDOW_WIDTH = 1000;
const unsigned int WINDOW_HEIGHT = 1000;
const double TIMESTEP = 0.1;
}  // namespace

int main()
{
  // create the window
  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulator");
  window.setFramerateLimit(60);

  const double circle_radius = 50.0;

  auto particles = generateParticles(100, WINDOW_HEIGHT, WINDOW_WIDTH);

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

    for (size_t i = 0; i < particles.size(); i++)
    {
      auto& particle = particles.at(i);
      // Apply forces
      applyGravity(particle.getParticle());

      // Update object
      physics::updateObject(particle.getParticle(), TIMESTEP);

      // Resolve collisions with other particles
      for (size_t j = i + 1; j < particles.size(); j++)
      {
        physics::collisionCheckOtherParticles(particle, particles.at(j));
      }

      // Resolve collisions with wall
      physics::collisionCheckWall(particle.getParticle(),  WINDOW_HEIGHT, WINDOW_WIDTH);
      particle.getParticle().clearForces();

      particle.setDrawPosition(WINDOW_HEIGHT);
      window.draw(particle.getShape());      
    }

    window.display();
  }

  return 0;
}
