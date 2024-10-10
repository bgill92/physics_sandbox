#pragma once

#include "common.hpp"
#include "dynamics.hpp"
#include "graphics.hpp"
#include "object.hpp"

class Particle final : public ObjectBase 
{
public:
  Particle() = delete;

  Particle(const double radius, const double mass, const physics::stateVector& state, const sf::Color& color)
  : radius_{ radius }, point_mass_{mass, state}, graphics_{radius, color} {};

  double getRadius()
  {
    return radius_;
  };

  dynamics::PointMass& getDynamics() {return point_mass_;}

  graphics::CircleGraphics& getGraphics() {return graphics_;}

  // Overwritten functions

  physics::State& getState() override {point_mass_.getState();}

  physics::stateVector& getStateVector() override {point_mass_.getState().getStateVector();}

  double getMass() override {point_mass_.getState().getMass();}

  // physics::commandVector& getForces() override {point_mass_.getStateObject().getForces();}

  // physics::commandVector getAcceleration() override {point_mass_.getStateObject().getForces()/point_mass_.getStateObject().getMass()};

  // double getMass() const override {point_mass_.getStateObject().getMass()};

  // void setState(const physics::stateVector& state) override {point_mass_.getStateObject().setState(state);};

  // void setForces(const physics::commandVector& forces) override {point_mass_.getStateObject().setForces(forces);};

private:
  double radius_;
  dynamics::PointMass point_mass_;
  graphics::CircleGraphics graphics_;
};

// std::vector<Particle> generateParticles(const size_t num_particles, const unsigned int WINDOW_HEIGHT, const unsigned int WINDOW_WIDTH);