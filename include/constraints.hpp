#pragma once

#include <functional>

#include "common.hpp"
#include "particle.hpp"
#include "rectangle.hpp"
#include "dynamics.hpp"

namespace constraints
{

struct ConstraintsBase
{
  virtual ~ConstraintsBase() = default;

  virtual size_t getObjectIdx() const = 0;
};

/**
 * @brief      A constraint which constrains an object along a circular path
 */
struct CircleConstraint : public ConstraintsBase
{
  CircleConstraint() = delete;

  CircleConstraint(const size_t object_idx, const physics::State& center_of_constraint, const double radius)
    : object_idx_{ object_idx }, center_of_constraint_{ center_of_constraint }, radius_{ radius }
  {
  }

  physics::State getCenterOfConstraint() const
  {
    return center_of_constraint_;
  }

  double getRadius() const
  {
    return radius_;
  }

  size_t getObjectIdx() const override
  {
    return object_idx_;
  }

private:
  size_t object_idx_;
  physics::State center_of_constraint_;
  double radius_;
};

/**
 * @brief      Constrain two objects to be a certain distance apart
 */
struct DistanceConstraint : public ConstraintsBase
{
  DistanceConstraint() = delete;

  DistanceConstraint(const size_t first_object_idx, const size_t second_object_idx, const double distance)
    : first_object_idx_{ first_object_idx }, second_object_idx_{ second_object_idx }, distance_{ distance }
  {
  }

  double getDistance() const
  {
    return distance_;
  }

  size_t getObjectIdx() const override
  {
    return first_object_idx_;
  }

  size_t getSecondObjectIdx() const
  {
    return second_object_idx_;
  }

private:
  size_t first_object_idx_;
  size_t second_object_idx_;
  double distance_;
};

struct LinearConstraint : public ConstraintsBase
{
  LinearConstraint() = delete;

  LinearConstraint(const size_t object_idx, const physics::State& start_point, const physics::State& end_point)
    : object_idx_{ object_idx }, start_point_{ start_point }, end_point_{ end_point }
  {
  }

  size_t getObjectIdx() const override
  {
    return object_idx_;
  }

  physics::State getStart() const
  {
    return start_point_;
  }

  physics::State getEnd() const
  {
    return end_point_;
  }

private:
  size_t object_idx_;
  physics::State start_point_;
  physics::State end_point_;
};

using Constraint = std::variant<CircleConstraint, DistanceConstraint, LinearConstraint>;

struct Constrain
{
  Constrain() = delete;
  Constrain(std::vector<Object>& objects, std::vector<std::reference_wrapper<physics::State>>& states)
    : objects_{ objects }, current_states_{ states }
  {
  }

  void operator()(const CircleConstraint& constraint);

  void operator()(const DistanceConstraint& constraint);

  void operator()(const LinearConstraint& constraint);

private:
  std::vector<Object>& objects_;
  std::vector<std::reference_wrapper<physics::State>>& current_states_;
};

struct VelocityUpdater
{
  VelocityUpdater() = delete;
  VelocityUpdater(std::vector<physics::State>& previous_states,
                  std::vector<std::reference_wrapper<physics::State>>& current_states, const double timestep)
    : previous_states_{ previous_states }, current_states_{ current_states }, timestep_{ timestep } {};

  void operator()(const CircleConstraint& constraint);

  void operator()(const DistanceConstraint& constraint);

  void operator()(const LinearConstraint& constraint);

private:
  std::vector<physics::State>& previous_states_;
  std::vector<std::reference_wrapper<physics::State>>& current_states_;
  double timestep_;
};

struct ConstraintsManager
{
  ConstraintsManager() = delete;
  ConstraintsManager(std::vector<Object>& objects, std::vector<Constraint>& constraints);

  void evaluateConstraints();

  void storePreviousState(const size_t idx, const physics::State& previous_state)
  {
    previous_states_.at(idx) = previous_state;
  }

  /**
   * @brief      Performs a velocity correction to the object after a constraint has been satisfied. Necessary for
   * particle based dynamics.
   *
   * @param[in]  timestep        The timestep over which the object was subject to the constraint
   * @param      object          The object
   * @param[in]  previous_state  The previous state of the object
   *
   * @tparam     T               A class which satisfies the hasDynamicsAndConstraint concept
   */
  void updateVelocityAfterConstraint(const double timestep);

  std::vector<physics::State>& getPreviousStates()
  {
    return previous_states_;
  }

private:
  std::vector<Object>& objects_;
  std::vector<Constraint>& constraints_;
  std::vector<physics::State> previous_states_;
  std::vector<std::reference_wrapper<physics::State>> current_states_;
};

}  // namespace constraints
