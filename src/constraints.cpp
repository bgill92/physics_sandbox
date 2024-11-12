#include "constraints.hpp"

#include <iostream>

namespace constraints
{

void Constrain::operator()(const CircleConstraint& constraint)
{
  // Get the state
  physics::State& state = current_states_.at(constraint.getObjectIdx()).get();

  // Get the difference in position between the current state and the
  Eigen::Vector2d difference_pos = (state - constraint.getCenterOfConstraint()).head<2>();

  // Normalize the distance to the center
  difference_pos.normalize();

  // Set the position of the Object after it is constrainted
  state.head<2>() = (difference_pos * constraint.getRadius()) + constraint.getCenterOfConstraint().head<2>();
}

void Constrain::operator()(const DistanceConstraint& constraint)
{
  // Get the state of the first object
  physics::State& state_1 = current_states_.at(constraint.getObjectIdx()).get();

  // Get the state of the second object
  physics::State& state_2 = current_states_.at(constraint.getSecondObjectIdx()).get();

  // Get the difference in position between the two states
  Eigen::Vector2d difference_pos = (state_2 - state_1).head<2>();

  // Calculate the distance between the two objects
  const auto distance = difference_pos.norm();

  // The error between the current distance and the constraint distance
  const auto error = distance - constraint.getDistance();

  // Normalize the difference vector
  difference_pos.normalize();

  const auto get_mass_lambda = [&](auto& object) -> double { return object.getDynamics().getMass(); };

  // Get the mass of object 1
  const auto mass_1 = std::visit(get_mass_lambda, objects_.at(constraint.getObjectIdx()));

  // Get the mass of object 2
  const auto mass_2 = std::visit(get_mass_lambda, objects_.at(constraint.getSecondObjectIdx()));

  const auto inverse_mass_1 = 1 / mass_1;

  const auto inverse_mass_2 = 1 / mass_2;

  const auto inverse_mass_total = inverse_mass_1 + inverse_mass_2;

  // Set the position of the first object
  state_1.head<2>() += (inverse_mass_1 / (inverse_mass_total)) * (error * difference_pos);

  // Set the position of the second object
  state_2.head<2>() += -(inverse_mass_2 / (inverse_mass_total)) * (error * difference_pos);
}

template <hasDynamics T>
physics::State& getState(T& object)
{
  return object.getDynamics().getState();
}

ConstraintsManager::ConstraintsManager(std::vector<Object>& objects, std::vector<Constraint>& constraints)
  : objects_{ objects }, constraints_{ constraints }
{
  previous_states_.resize(objects_.size());
  const auto get_current_state_lambda = [&](auto& object) -> physics::State& {
    return getState<decltype(object)>(object);
  };

  for (auto& object : objects)
  {
    current_states_.push_back(std::ref(std::visit(get_current_state_lambda, object)));
  }
}

void ConstraintsManager::evaluateConstraints()
{
  if (constraints_.empty())
  {
    return;
  }

  Constrain constrain{ this->objects_, this->current_states_ };

  for (const auto& constraint : constraints_)
  {
    std::visit(constrain, constraint);
  }
}

void VelocityUpdater::operator()(const CircleConstraint& constraint)
{
  // Get the state of the object
  physics::State& state = current_states_.at(constraint.getObjectIdx()).get();

  physics::State& previous_state = previous_states_.at(constraint.getObjectIdx());

  // Get the difference in position
  Eigen::Vector3d diff_in_pos = (state.head<3>() - previous_state.head<3>());

  // Set the velocity
  state.tail<3>() = diff_in_pos / timestep_;
}

void VelocityUpdater::operator()(const DistanceConstraint& constraint)
{
  // Get the state of the object
  physics::State& state_1 = current_states_.at(constraint.getObjectIdx()).get();

  physics::State& previous_state_1 = previous_states_.at(constraint.getObjectIdx());

  // Get the difference in position
  Eigen::Vector3d diff_in_pos_1 = (state_1.head<3>() - previous_state_1.head<3>());

  // Set the velocity
  state_1.tail<3>() = diff_in_pos_1 / timestep_;

  // Get the state of the object
  physics::State& state_2 = current_states_.at(constraint.getSecondObjectIdx()).get();

  physics::State& previous_state_2 = previous_states_.at(constraint.getSecondObjectIdx());

  // Get the difference in position
  Eigen::Vector3d diff_in_pos_2 = (state_2.head<3>() - previous_state_2.head<3>());

  // Set the velocity
  state_2.tail<3>() = diff_in_pos_2 / timestep_;
}

void ConstraintsManager::updateVelocityAfterConstraint(const double timestep)
{
  VelocityUpdater velocity_updater{ previous_states_, current_states_, timestep };

  for (const auto& constraint : constraints_)
  {
    // std::visit(update_velocity, constraint);
    std::visit(velocity_updater, constraint);
  }
}

}  // namespace constraints