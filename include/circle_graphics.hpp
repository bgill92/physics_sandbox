#pragma once

#include <SFML/Graphics.hpp>

#include "common.hpp"

namespace graphics
{
/**
 * @brief      Contains the necessary information for drawing a circle using SFML
 */
struct CircleGraphics
{
  CircleGraphics() = delete;
  CircleGraphics(const double radius, const sf::Color& color = sf::Color::Red) : radius_{ radius }
  {
    circle_ = sf::CircleShape(radius);
    circle_.setOrigin(radius, radius);
    circle_.setFillColor(color);
  }

  sf::CircleShape& getShape()
  {
    return circle_;
  };

  /**
   * @brief      Sets the draw position of the circle
   *
   * @param[in]  state          The state of the object
   * @param[in]  WINDOW_HEIGHT  The window height
   */
  void setDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
  {
    circle_.setPosition(convertToDrawPosition(state, WINDOW_HEIGHT));
  }

private:
  double radius_;
  sf::CircleShape circle_;

  /**
   * @brief      Converts the y position from physics space to draw space
   *
   * @param[in]  state          The state
   * @param[in]  WINDOW_HEIGHT  The window height
   *
   * @return     A SFML vector of x and y position that it uses for drawing
   */
  sf::Vector2f convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT);
};
}  // namespace graphics