#include "graphics.hpp"
#include "particle.hpp"

#include <cmath>
#include <iostream>
#include <numbers>

namespace graphics
{

void Draw::operator()(Particle& particle)
{
  // Set the scale based on the config
  particle.getGraphics().getShape().setScale(config_.pixels_to_meters_ratio, config_.pixels_to_meters_ratio);

  // Get the state and multiply the position by the scale
  auto state = particle.getDynamics().getState();
  state.head<2>() *= config_.pixels_to_meters_ratio;

  // Set the draw position
  particle.getGraphics().setDrawPosition(state, config_.window_height);
  window_.draw(particle.getGraphics().getShape());
}

void Draw::operator()(Rectangle& rectangle)
{
  // Set the scale based on the config
  rectangle.getGraphics().getShape().setScale(config_.pixels_to_meters_ratio, config_.pixels_to_meters_ratio);

  // Get the state and multiply the position by the scale
  auto state = rectangle.getDynamics().getState();
  state.head<2>() *= config_.pixels_to_meters_ratio;

  // Set the draw position
  rectangle.getGraphics().setDrawPosition(state, config_.window_height);
  window_.draw(rectangle.getGraphics().getShape());
}

void DrawerManager::drawObject(const size_t idx)
{
  std::visit(drawer_, objects_.at(idx));
}

void DrawConstraint::operator()(constraints::CircleConstraint& constraint)
{
  // Get the center of the circle constraint
  const auto center_of_constraint = constraint.getCenterOfConstraint();

  const auto get_current_state_lambda = [&](auto& object) -> physics::State& { return object.getDynamics().getState(); };

  // Get the current state
  const auto& state = std::visit(get_current_state_lambda, objects_.at(constraint.getObjectIdx()));

  // Calculate the radius of the circle
  const auto length = std::hypot(state(physics::STATE_X_POS_IDX) - center_of_constraint(physics::STATE_X_POS_IDX),
                                 state(physics::STATE_Y_POS_IDX) - center_of_constraint(physics::STATE_Y_POS_IDX));

  // Width of the rectangle
  const float width = 0.1;

  // Create the drawable rectangle
  sf::RectangleShape line(sf::Vector2f(length, width));

  // Set the scaling
  line.setScale(config_.pixels_to_meters_ratio, config_.pixels_to_meters_ratio);

  // Set the origin of the rectangle to the middle of the leftmost side
  line.setOrigin(0, width / 2);

  // Calculate what the angle of the rectangle should be
  const auto angle = std::atan2(state(physics::STATE_Y_POS_IDX) - center_of_constraint(physics::STATE_Y_POS_IDX),
                                state(physics::STATE_X_POS_IDX) - center_of_constraint(physics::STATE_X_POS_IDX)) *
                     (rad2deg);

  // Rotate the rectangle that many degrees
  line.rotate(-angle);

  // Calculate the scaled window height
  const auto scaled_window_height = config_.window_height / config_.pixels_to_meters_ratio;

  line.setPosition(center_of_constraint(physics::STATE_X_POS_IDX) * config_.pixels_to_meters_ratio,
                   (scaled_window_height - center_of_constraint(physics::STATE_Y_POS_IDX)) *
                       config_.pixels_to_meters_ratio);

  window_.draw(line);
}

void DrawConstraint::operator()(constraints::DistanceConstraint& constraint)
{
  const auto get_current_state_lambda = [&](auto& object) -> physics::State& { return object.getDynamics().getState(); };

  // Get the state of the first object
  const auto& state_1 = std::visit(get_current_state_lambda, objects_.at(constraint.getObjectIdx()));

  // Get the state of the second object
  const auto& state_2 = std::visit(get_current_state_lambda, objects_.at(constraint.getSecondObjectIdx()));

  // Calculate the distance between objects
  const auto distance = ((state_2 - state_1).head<2>()).norm();

  // Width of the rectangle
  const float width = 0.1;

  // Create the drawable rectangle
  sf::RectangleShape line(sf::Vector2f(distance, width));

  // Set the scaling
  line.setScale(config_.pixels_to_meters_ratio, config_.pixels_to_meters_ratio);

  // Set the origin of the rectangle to the middle of the leftmost side
  line.setOrigin(0, width / 2);

  // Calculate what the angle of the rectangle should be
  const auto angle = std::atan2(state_2(physics::STATE_Y_POS_IDX) - state_1(physics::STATE_Y_POS_IDX),
                                state_2(physics::STATE_X_POS_IDX) - state_1(physics::STATE_X_POS_IDX)) *
                     (rad2deg);

  // Rotate the rectangle that many degrees
  line.rotate(-angle);

  // Calculate the scaled window height
  const auto scaled_window_height = config_.window_height / config_.pixels_to_meters_ratio;

  line.setPosition(state_1(physics::STATE_X_POS_IDX) * config_.pixels_to_meters_ratio,
                   (scaled_window_height - state_1(physics::STATE_Y_POS_IDX)) * config_.pixels_to_meters_ratio);

  window_.draw(line);
}

void DrawConstraint::operator()(constraints::LinearConstraint& constraint)
{
  // Get the start and endpoints
  const auto& end = constraint.getEnd();
  const auto& start = constraint.getStart();

  // Calculate the distance between the endpoints of the constraint
  const auto distance = ((end - start).head<2>()).norm();

  // Width of the rectangle
  const float width = 0.1;

  // Create the drawable rectangle
  sf::RectangleShape line(sf::Vector2f(distance, width));

  // Set the scaling
  line.setScale(config_.pixels_to_meters_ratio, config_.pixels_to_meters_ratio);

  // Set the origin of the rectangle to the middle of the leftmost side
  line.setOrigin(0, width / 2);

  // Calculate what the angle of the rectangle should be
  const auto angle = std::atan2(end(physics::STATE_Y_POS_IDX) - start(physics::STATE_Y_POS_IDX),
                                end(physics::STATE_X_POS_IDX) - start(physics::STATE_X_POS_IDX)) *
                     (180 / std::numbers::pi);

  // Rotate the rectangle that many degrees
  line.rotate(-angle);

  // Calculate the scaled window height
  const auto scaled_window_height = config_.window_height / config_.pixels_to_meters_ratio;

  line.setPosition(start(physics::STATE_X_POS_IDX) * config_.pixels_to_meters_ratio,
                   (scaled_window_height - start(physics::STATE_Y_POS_IDX)) * config_.pixels_to_meters_ratio);

  window_.draw(line);
}

void DrawerManager::drawConstraints()
{
  for (auto& constraint : constraints_)
  {
    std::visit(constraint_drawer_, constraint);
  }
}

}  // namespace graphics