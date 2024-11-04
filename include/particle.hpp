#pragma once

#include <memory>
#include <optional>

#include "constraints.hpp"
#include "common.hpp"
#include "dynamics.hpp"
#include "graphics.hpp"

class Particle
{
public:
  Particle() = delete;

  Particle(const double radius, const double mass, const physics::State& state, const sf::Color& color,
           std::optional<physics::Constraint> constraint = std::nullopt)
    : radius_{ radius }, point_mass_{ mass, state }, graphics_{ radius, color }, constraint_{ constraint } {};

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

  std::optional<physics::Constraint>& getConstraint()
  {
    return constraint_;
  }

  // void collisionCheckWall(const double WINDOW_HEIGHT, const double WINDOW_WIDTH, const double COEFFICIENT_OF_RESTITUTION);

private:
  double radius_;
  dynamics::PointMass point_mass_;
  graphics::CircleGraphics graphics_;

  std::optional<physics::Constraint> constraint_;
};

/**
 * @brief      Generates Particles based on the configuration
 *
 * @param[in]  config  The configuration
 *
 * @return     A vector of Particles
 */
std::vector<Object> generateParticles(const Config& config);