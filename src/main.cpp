#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "physics.hpp"
#include "object.hpp"
#include "particle.hpp"

#include <SFML/Graphics.hpp>

namespace {
    const unsigned int WINDOW_WIDTH  = 1000;
    const unsigned int WINDOW_HEIGHT = 1000;
    const double COEFFICIENT_OF_RESTITUTION = 0.5;
    const double TIMESTEP = 0.1;
}

sf::Vector2f convertToDrawPosition(const Particle& particle)
{
    return {static_cast<float>(particle.getState(physics::STATE_VECTOR_X_POS_IDX)), static_cast<float>(WINDOW_HEIGHT) - static_cast<float>(particle.getState(physics::STATE_VECTOR_Y_POS_IDX))};
}

void collisionCheckAndResolve(Particle& particle)
{
    // If a particle hits the bottom
    if ((particle.getState(physics::STATE_VECTOR_Y_POS_IDX) - particle.getRadius()) < 0)
    {
        particle.setState(physics::STATE_VECTOR_Y_POS_IDX, particle.getRadius());
        particle.setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX, -1 * particle.getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX)*COEFFICIENT_OF_RESTITUTION);
    }
    // If a particle hits the top
    if ((particle.getState(physics::STATE_VECTOR_Y_POS_IDX) + particle.getRadius()) > WINDOW_HEIGHT)
    {
        particle.setState(physics::STATE_VECTOR_Y_POS_IDX, WINDOW_HEIGHT - particle.getRadius());
        particle.setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX, -1 * particle.getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX)*COEFFICIENT_OF_RESTITUTION);
    }
    // If a particle hits the right side
    if ((particle.getRadius() + particle.getState(physics::STATE_VECTOR_X_POS_IDX)) > WINDOW_WIDTH)
    {
        particle.setState(physics::STATE_VECTOR_X_POS_IDX, WINDOW_WIDTH - particle.getRadius());
        particle.setState(physics::STATE_VECTOR_X_LIN_VEL_IDX, -1 * particle.getState(physics::STATE_VECTOR_X_LIN_VEL_IDX)*COEFFICIENT_OF_RESTITUTION);
    }
    // If the particle hits the left side
    if (particle.getState(physics::STATE_VECTOR_X_POS_IDX) < particle.getRadius())
    {
        particle.setState(physics::STATE_VECTOR_X_POS_IDX, particle.getRadius());
        particle.setState(physics::STATE_VECTOR_X_LIN_VEL_IDX, -1 * particle.getState(physics::STATE_VECTOR_X_LIN_VEL_IDX)*COEFFICIENT_OF_RESTITUTION);
    }

}

int main()
{
    // create the window
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulator");
    window.setFramerateLimit(60);

    const double circle_radius = 50.0;

    sf::CircleShape shape(circle_radius);
    shape.setOrigin(circle_radius, circle_radius);
    shape.setFillColor(sf::Color::Green);

    auto p = Particle(circle_radius, 1, {450, 450, 0, 10, 100, 0});

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
            if(event.type == sf::Event::Closed)
                window.close();
        }

        // draw it

        applyGravity(p);

        // physics::update(p, TIMESTEP);
        physics::updateObject(p, TIMESTEP);
        collisionCheckAndResolve(p);
        p.clearForces();

        window.clear();
        shape.setPosition(convertToDrawPosition(p));
        window.draw(shape);
        window.display();

        // Get the end time of the loop
        const auto end_time = std::chrono::high_resolution_clock::now(); 

        // Calculate how much time the loop took
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;

        // std::cout <<  "Time in milliseconds: " << elapsed_time.count() << "\n";
    }

    return 0;
}
