#pragma once

#include <SFML/Graphics.hpp>

namespace graphics
{

struct CircleGraphics
{
  CircleGraphics() = delete;
  CircleGraphics(const sf::Color& color = sf::Color::Red)
  {
    circle_ = sf::CircleShape(particle_.getRadius());
    circle_.setOrigin(particle_.getRadius(), particle_.getRadius());
    circle_.setFillColor(color);
  }

  sf::CircleShape& getShape()
  {
    return circle_;
  };

private:
  sf::CircleShape circle_;
};

}  // namespace graphics