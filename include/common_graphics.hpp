#pragma once

#include "common.hpp"

#include <SFML/Graphics.hpp>

namespace graphics
{

/**
 * @brief      Converts the y position from physics space to draw space
 *
 * @param[in]  state          The state
 * @param[in]  WINDOW_HEIGHT  The window height
 *
 * @return     A SFML vector of x and y position that it uses for drawing
 */
sf::Vector2f convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT);

}  // namespace graphics