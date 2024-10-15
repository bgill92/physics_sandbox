#pragma once

#include <memory>

#include "common.hpp"
#include "dynamics.hpp"
#include "graphics.hpp"
#include "object.hpp"

class Particle
{
public:
  Particle() = delete;

  Particle(const double radius, const double mass, const physics::State& state, const sf::Color& color)
  : radius_{ radius }, point_mass_{mass, state}, graphics_{radius, color} {};

  double getRadius()
  {
    return radius_;
  };

  dynamics::PointMass& getDynamics() {return point_mass_;}

  graphics::CircleGraphics& getGraphics() {return graphics_;}

  void collisionCheckWall(const double WINDOW_HEIGHT, const double WINDOW_WIDTH, const double COEFFICIENT_OF_RESTITUTION);

private:
  double radius_;
  dynamics::PointMass point_mass_;
  graphics::CircleGraphics graphics_;
};

std::vector<Object> generateParticles(const size_t num_particles, const unsigned int WINDOW_HEIGHT, const unsigned int WINDOW_WIDTH);