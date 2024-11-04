#include "graphics.hpp"
#include "particle.hpp"

namespace graphics
{

void Draw::operator()(Particle& particle)
{
  // Set the scale based on the config
  particle.getGraphics().getShape().setScale(config_.pixels_to_meters_ratio, config_.pixels_to_meters_ratio);

  // Get the state and multiply the position by the scale
  auto state = particle.getDynamics().getState();
  state.head<3>() *= config_.pixels_to_meters_ratio;

  // Set the draw position
  particle.getGraphics().setDrawPosition(state, config_.window_height);
  window_.draw(particle.getGraphics().getShape());
}

void DrawerManager::drawObject(const size_t idx)
{
  std::visit(drawer_, objects_.at(idx));
}

sf::Vector2f CircleGraphics::convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
{
  return { static_cast<float>(state(physics::STATE_X_POS_IDX)),
           static_cast<float>(WINDOW_HEIGHT) - static_cast<float>(state(physics::STATE_Y_POS_IDX)) };
}
}  // namespace graphics