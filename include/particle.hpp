#pragma once

#include "physics.hpp"
#include "object.hpp"

#include <SFML/Graphics.hpp>

class Particle : public Object
{
public:
  Particle() = delete;

  Particle(const double radius, const double mass, const physics::stateVector& state)
    : Object{ mass, state }, radius_{ radius } {};

  double getRadius()
  {
    return radius_;
  };

private:
  double radius_;
};

sf::Vector2f convertToDrawPosition(const Particle& particle, const unsigned int WINDOW_HEIGHT);

struct DrawableParticle
{
  DrawableParticle() = delete;

  DrawableParticle(Particle& particle, const sf::Color& color) : particle_{std::move(particle)}
  {
    circle_ = sf::CircleShape(particle_.getRadius());
    circle_.setOrigin(particle_.getRadius(), particle_.getRadius());
    circle_.setFillColor(color);
  }

  Particle& getParticle() {return particle_;};

  sf::CircleShape& getShape() {return circle_;};

  void setDrawPosition(const unsigned int WINDOW_HEIGHT)
  {
    circle_.setPosition(convertToDrawPosition(particle_, WINDOW_HEIGHT));
  }

private:
  Particle particle_;
  sf::CircleShape circle_;
};

std::vector<DrawableParticle> generateParticles(const size_t num_particles, const unsigned int WINDOW_HEIGHT, const unsigned int WINDOW_WIDTH);