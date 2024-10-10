#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "common.hpp"
#include "integrator"
#include "physics.hpp"
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

  auto physics_manager = PhysicsManager(particles, integrator::RK4<physics::stateVector>)

  // Particle p1 {50, 1, {100, 450, 0, 50, 0 ,0}, sf::Color::Red};
  // Particle p2 {25, 1, {900, 410, 0, -50, 0 ,0}, sf::Color::Green};

  // std::vector<Particle> particles;

  // particles.push_back(p1);
  // particles.push_back(p2);

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
      // physics::applyGravity(particle.getDynamics().getStateObject());

      // Update object
      physics::updateParticle(particle, TIMESTEP);

      // Resolve collisions with other particles
      for (size_t j = i + 1; j < particles.size(); j++)
      {
        physics::collisionCheckOtherParticles(particle, particles.at(j));
      }

      // Resolve collisions with wall
      physics::collisionCheckWall(particle, WINDOW_HEIGHT, WINDOW_WIDTH);
      particle.getDynamics().getStateObject().clearForces();

      particle.getGraphics().setDrawPosition(particle.getDynamics().getStateObject(), WINDOW_HEIGHT);
      window.draw(particle.getGraphics().getShape());
    }

    window.display();

    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in microseconds (or any other unit)
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Output the duration
    std::cout << "Loop time: " << duration.count() << " microseconds" << std::endl;


  }

  return 0;
}
