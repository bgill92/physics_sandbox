#pragma once

#include <SFML/Graphics.hpp>

#include <iostream>

#include "common.hpp"
#include "common_graphics.hpp"

namespace graphics
{
/**
 * @brief      Contains the necessary information for drawing a rectangle using SFML
 */
struct RectangleGraphics
{
  RectangleGraphics() = delete;
  RectangleGraphics(const double length, const double height, const sf::Color& color = sf::Color::Red)
    : length_{ length }, height_{ height }
  {
    rectangle_ = sf::RectangleShape({ static_cast<float>(length), static_cast<float>(height) });
    rectangle_.setOrigin(length / 2, height / 2);
    rectangle_.setFillColor(color);
  }

  sf::RectangleShape& getShape()
  {
    return rectangle_;
  };

  /**
   * @brief      Sets the draw position of the rectangle
   *
   * @param[in]  state          The state of the object
   * @param[in]  WINDOW_HEIGHT  The window height
   */
  void setDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
  {
    rectangle_.setPosition(convertToDrawPosition(state, WINDOW_HEIGHT));
    rectangle_.setRotation(-state(physics::STATE_THETA_IDX) * rad2deg);
  }

private:
  double length_;
  double height_;
  sf::RectangleShape rectangle_;
};
}  // namespace graphics