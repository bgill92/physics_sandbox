#pragma once

#include "common.hpp"
#include "dynamics.hpp"
#include "circle_graphics.hpp"

class Particle
{
public:
  Particle() = delete;

  Particle(const double radius, const double mass, const physics::State& state, const sf::Color& color)
    : radius_{ radius }, point_mass_{ mass, state }, graphics_{ radius, color } {};

  double getRadius()
  {
    return radius_;
  };

  dynamics::PointMass& getDynamics()
  {
    return point_mass_;
  }

  graphics::CircleGraphics& getGraphics()
  {
    return graphics_;
  }

private:
  double radius_;
  dynamics::PointMass point_mass_;
  graphics::CircleGraphics graphics_;
};

/**
 * @brief      Generates Particles based on the configuration
 *
 * @param[in]  config  The configuration
 *
 * @return     A vector of Particles
 */
std::vector<Object> generateParticles(const Config& config);