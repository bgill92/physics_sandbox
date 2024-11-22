#include "constraints.hpp"
#include "utils.hpp"

#include <algorithm>
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

  // Calculate the vectors from the center of the Object to the constraint location in the local frame
  const auto theta_1 = state_1(physics::STATE_THETA_IDX);
  const Eigen::Rotation2Dd local_to_global_1(theta_1);
  // const Eigen::Vector2d center_to_constraint_1 = (local_to_global_1.inverse()) *
  // (constraint.getFirstConstraintLocationLocal() - state_1.head<2>());

  const Eigen::Vector2d center_to_constraint_1 = constraint.getFirstConstraintLocationLocal();

  const auto theta_2 = state_2(physics::STATE_THETA_IDX);
  const Eigen::Rotation2Dd local_to_global_2(theta_2);
  // const Eigen::Vector2d center_to_constraint_2 = (local_to_global_2.inverse()) *
  // (constraint.getSecondConstraintLocationLocal() - state_2.head<2>());

  const Eigen::Vector2d center_to_constraint_2 = constraint.getSecondConstraintLocationLocal();

  // Get the constraints attachment points in the global frame
  const Eigen::Vector2d con_att_pnt_1 = state_1.head<2>() + local_to_global_1 * center_to_constraint_1;
  const Eigen::Vector2d con_att_pnt_2 = state_2.head<2>() + local_to_global_2 * center_to_constraint_2;

  // Calculate the normal vector
  const Eigen::Vector2d normal = (con_att_pnt_2 - con_att_pnt_1).normalized();

  // The Constraint equation
  const auto C = (con_att_pnt_2 - con_att_pnt_1).norm() - constraint.getDistance();

  // The derivative of the rotation matrices
  const Eigen::Matrix<double, 2, 2> rot_D_1{ { -std::sin(theta_1), -std::cos(theta_1) },
                                             { std::cos(theta_1), -std::sin(theta_1) } };

  const Eigen::Matrix<double, 2, 2> rot_D_2{ { -std::sin(theta_2), -std::cos(theta_2) },
                                             { std::cos(theta_2), -std::sin(theta_2) } };

  // Get the masses and inertias
  const auto get_mass_lambda = [&](auto& object) -> double { return object.getDynamics().getMass(); };
  const auto get_inertia_lambda = [&](auto& object) -> double { return object.getInertia(); };
  const auto mass_1 = std::visit(get_mass_lambda, objects_.at(constraint.getObjectIdx()));
  const auto mass_2 = std::visit(get_mass_lambda, objects_.at(constraint.getSecondObjectIdx()));
  const auto inertia_1 = std::visit(get_inertia_lambda, objects_.at(constraint.getObjectIdx()));
  const auto inertia_2 = std::visit(get_inertia_lambda, objects_.at(constraint.getSecondObjectIdx()));

  // Calculate the effective masses
  const auto inverse_mass_1 = 1.0 / (mass_1);
  const auto inverse_mass_2 = 1.0 / (mass_2);

  const auto R_prime_r_1 = rot_D_1 * center_to_constraint_1;
  const auto dC_dTheta_1 = normal.dot(R_prime_r_1);

  const auto R_prime_r_2 = rot_D_2 * center_to_constraint_2;
  const auto dC_dTheta_2 = (-normal).dot(R_prime_r_2);

  const auto inverse_inertial_mass_1 = (1.0 / inertia_1) * (dC_dTheta_1 * dC_dTheta_1);
  const auto inverse_inertial_mass_2 = (1.0 / inertia_2) * (dC_dTheta_2 * dC_dTheta_2);

  const auto w = inverse_mass_1 + inverse_mass_2 + inverse_inertial_mass_1 + inverse_inertial_mass_2;

  // Calculate the lagrange multiplier
  const auto lambda = -C / w;

  // Apply the corrections
  // I had to switch the signs for this to work, why?
  const Eigen::Vector2d delta_p_1 = -lambda * inverse_mass_1 * normal;
  const Eigen::Vector2d delta_p_2 = lambda * inverse_mass_2 * normal;

  const auto delta_theta_1 = lambda * (1 / inertia_1) * dC_dTheta_1;
  const auto delta_theta_2 = lambda * (1 / inertia_2) * dC_dTheta_2;

  state_1.head<2>() += delta_p_1;
  state_1(physics::STATE_THETA_IDX) -= delta_theta_1;

  // Had to subtract here too
  state_2.head<2>() += delta_p_2;
  state_2(physics::STATE_THETA_IDX) -= delta_theta_2;
}

void Constrain::operator()(const LinearConstraint& constraint)
{
  // Get the state
  physics::State& state = current_states_.at(constraint.getObjectIdx()).get();

  const auto start = constraint.getStart();
  const auto end = constraint.getEnd();

  const auto new_pos = utils::closestPointInLine(start.head<2>(), end.head<2>(), state.head<2>());

  // Constrain the position of the object to the closest to the line
  state.head<2>() = new_pos;
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

// TODO: Make the VelocityUpdater a templated function, and make the DistanceConstraint a template specialization
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

void VelocityUpdater::operator()(const LinearConstraint& constraint)
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