#include "constraints.hpp"

namespace constraints
{

void Constrain::operator()(const CircleConstraint& constraint)
{
  // Get the state
  physics::State& state = current_states_.at(constraint.getObjectIdx()).get();

  // Get the difference in position between the current state and the
  Eigen::Vector3d difference_pos = (state - constraint.getCenterOfConstraint()).head<3>();

  // Normalize the distance to the center
  difference_pos.normalize();

  // Set the position of the Object after it is constrainted
  state.head<3>() = (difference_pos * constraint.getRadius()) + constraint.getCenterOfConstraint().head<3>();    
}

template<hasDynamics T>
physics::State& getState(T& object)
{
  return object.getDynamics().getState();
}

ConstraintsManager::ConstraintsManager(std::vector<Object>& objects, std::vector<Constraint>& constraints) : objects_{objects}, constraints_{constraints}
{
  previous_states_.resize(objects_.size());
  const auto get_current_state_lambda = [&](auto& object) -> physics::State&
  {
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

  Constrain constrain{this->objects_, this->current_states_};

  for (const auto& constraint : constraints_)
  {
    std::visit(constrain, constraint);      
  }
}

void ConstraintsManager::updateVelocityAfterConstraint(const double timestep)
{

  const auto update_velocity = [&] (const auto& constraint)
  {
    // Get the state of the object
    physics::State& state = current_states_.at(constraint.getObjectIdx());

    physics::State& previous_state = previous_states_.at(constraint.getObjectIdx());

    // Get the difference in position
    Eigen::Vector3d diff_in_pos = (state.head<3>() - previous_state.head<3>());

    // Set the velocity
    state.tail<3>() = diff_in_pos / timestep;
  };

  for (const auto& constraint : constraints_)
  {
    std::visit(update_velocity, constraint);
  }
}

} // namespace constraints