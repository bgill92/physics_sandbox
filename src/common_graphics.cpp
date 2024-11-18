#include "common_graphics.hpp"

namespace graphics
{
sf::Vector2f convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
{
  return { static_cast<float>(state(physics::STATE_X_POS_IDX)),
           static_cast<float>(WINDOW_HEIGHT) - static_cast<float>(state(physics::STATE_Y_POS_IDX)) };
}
}  // namespace graphics