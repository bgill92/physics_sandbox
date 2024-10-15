#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "common.hpp"
#include "integrator.hpp"
#include "physics.hpp"
#include "particle.hpp"

#include <SFML/Graphics.hpp>

namespace
{
const unsigned int WINDOW_WIDTH = 1000;
const unsigned int WINDOW_HEIGHT = 1000;
const double TIMESTEP = 0.01;
}  // namespace

int main()
{
  // create the window
  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulator");
  window.setFramerateLimit(60);

  auto objects = generateParticles(100, WINDOW_HEIGHT, WINDOW_WIDTH);


  // Particle p1 {50, 1, {100, 450, 0, 50, 0 ,0}, sf::Color::Red};
  // Particle p2 {25, 1, {900, 410, 0, -50, 0 ,0}, sf::Color::Green};

  // std::vector<Object> objects;

  // objects.push_back(Particle(50, 1, physics::State(100, 450, 0, 50, 0 ,0), sf::Color::Red));
  // objects.push_back(Particle(25, 1, physics::State(900, 410, 0, -50, 0 ,0), sf::Color::Green));

  // particles.emplace_back(50, 1, physics::State(100, 450, 0, 50, 0 ,0), sf::Color::Red);
  // particles.emplace_back(25, 1, physics::State(900, 410, 0, -50, 0 ,0), sf::Color::Green);

  // particles.push_back(p1);
  // particles.push_back(p2);

  auto physics_manager = physics::PhysicsManager(TIMESTEP, objects);

  // create a clock to track the elapsed time
  sf::Clock clock;

  // size_t count = 0;

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

      if ( Particle* particle = std::get_if<Particle>(&objects.at(i)))
      {

        physics_manager.step(i);

        particle->getGraphics().setDrawPosition(particle->getDynamics().getState(), WINDOW_HEIGHT);
        window.draw(particle->getGraphics().getShape());        

      }
    }

    window.display();

    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in microseconds (or any other unit)
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Output the duration
    std::cout << "Loop time: " << duration.count() << " microseconds" << std::endl;

    // count++;

  }

  return 0;
}
