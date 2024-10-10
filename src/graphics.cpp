#include "graphics.hpp"

namespace graphics
{
	sf::Vector2f CircleGraphics::convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
	{
  return { static_cast<float>(state.getState(physics::STATE_VECTOR_X_POS_IDX)),
           static_cast<float>(WINDOW_HEIGHT) - static_cast<float>(state.getState(physics::STATE_VECTOR_Y_POS_IDX)) };
	}
}