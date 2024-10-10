#pragma once

#include "common.hpp"

#include <SFML/Graphics.hpp>

namespace graphics
{

struct CircleGraphics
{
  CircleGraphics() = delete;
  CircleGraphics(const double radius, const sf::Color& color = sf::Color::Red) : radius_{radius}
  {
    circle_ = sf::CircleShape(radius);
    circle_.setOrigin(radius, radius);
    circle_.setFillColor(color);
  }

  sf::CircleShape& getShape()
  {
    return circle_;
  };

  void setDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
  {
    circle_.setPosition(convertToDrawPosition(state, WINDOW_HEIGHT));
  }

private:
  double radius_;
  sf::CircleShape circle_;

  sf::Vector2f convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT);
};

}  // namespace graphics