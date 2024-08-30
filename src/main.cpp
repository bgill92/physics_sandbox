#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

namespace {
    const unsigned int WINDOW_WIDTH  = 1000;
    const unsigned int WINDOW_HEIGHT = 1000;
    static constexpr size_t STATE_VECTOR_SIZE = 6;
    const double COEFFICIENT_OF_RESTITUTION = 0.9;
    const double TIMESTEP = 0.01;
}

// State is x, y, z, vx, vy, vz
using stateVector = Eigen::Matrix<double, STATE_VECTOR_SIZE, 1>;

struct Particle {

    Particle() = delete;

    Particle(double mass, double radius, const stateVector& state) : mass_{mass}, radius_{radius}, state_{state} {};

    double mass_;
    double radius_;
    stateVector state_;
    Eigen::Vector3d force_accumulator_ = {0, 0, 0};

    void clearForces() {this->force_accumulator_ = {0, 0, 0};};
};

void update(Particle& particle, const double timestep)
{
    // Update is Runge-Kutta 4
    // Essentially we calculate the derivative at four points, take a weighted average of the derivatives,
    // and find the next state using that weighted derivative average
    const auto evaluateDerivative = [&particle](const stateVector& derivative, const double timestep)
    {
        stateVector state;
        // Calculate the next state given the derivative
        state.segment(0, 3) = particle.state_.segment(0, 3) + derivative.segment(0, 3) * timestep;
        // [dx, dy, dz] = [ dx, dy, dz] + [ddx, ddy, ddz] * timestep
        state.segment(3, 3) = particle.state_.segment(3, 3) + derivative.segment(3, 3) * timestep;

        // Calculate the derivative at the next state
        stateVector new_derivative;
        new_derivative.segment(0, 3) = state.segment(3, 3);
        new_derivative.segment(3, 3) = particle.force_accumulator_/particle.mass_;

        return new_derivative;
    };

    // k1 is the slope at the beginning of the time step
    // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
    // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the midpoint.
    // Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate of the slope at the endpoint.
    const auto k1 = evaluateDerivative(stateVector(), 0.0);
    const auto k2 = evaluateDerivative(k1, timestep/2);
    const auto k3 = evaluateDerivative(k2, timestep/2);
    const auto k4 = evaluateDerivative(k3, timestep);

    particle.state_ += (1.0/6.0) * timestep * (k1 + 2.0*k2 + 2.0*k3 + k4); 
}

void collisionCheckAndResolve(Particle& particle)
{
    // If the particle is below the ground, set it to be at the ground
    // and also set the velocity to go up
    if ((particle.state_(1) - 2*particle.radius_) < 0)
    {
        particle.state_(1) = 2*particle.radius_;
        particle.state_(4) = -1 * particle.state_(4)*COEFFICIENT_OF_RESTITUTION;
    }
    if ((2*particle.radius_ + particle.state_(0)) > WINDOW_WIDTH)
    {
        particle.state_(0) = WINDOW_WIDTH - 2*particle.radius_;
        particle.state_(3) = -1 * particle.state_(3)*COEFFICIENT_OF_RESTITUTION;
    }
    if (particle.state_(0) < 0)
    {
        particle.state_(0) = 0;
        particle.state_(3) = -1 * particle.state_(3)*COEFFICIENT_OF_RESTITUTION;
    }
}

void applyGravity(Particle& particle)
{
    particle.force_accumulator_(1) += -9.81; 
}

sf::Vector2f convertToDrawPosition(const Particle& particle)
{
    return {static_cast<float>(particle.state_(0)), static_cast<float>(WINDOW_HEIGHT) - static_cast<float>(particle.state_(1))};
}

Particle createParticle(const double mass, const double radius, const stateVector& state)
{
    return {mass, radius, state};
}

int main()
{
    // create the window
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulator");

    const float circle_radius = 50.0;

    sf::CircleShape shape(circle_radius);
    shape.setFillColor(sf::Color::Green);

    auto p = createParticle(1, circle_radius, {450, 450, 0, 10, 100, 0});

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

        update(p, TIMESTEP);
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

        std::cout <<  "Time in milliseconds: " << elapsed_time.count() << "\n";
    }

    return 0;
}
