#pragma once

#include "common.hpp"

namespace physics
{

/**
 * @brief      A constraint which constrains an object along a circular path
 */
struct CircleConstraint
{
  CircleConstraint() = delete;

  CircleConstraint(const physics::State& center_of_constraint, const double radius)
    : center_of_constraint_{ center_of_constraint }, radius_{ radius }
  {
  }

  physics::State getCenterOfConstraint()
  {
    return center_of_constraint_;
  }

  double getRadius()
  {
    return radius_;
  }

private:
  physics::State center_of_constraint_;
  double radius_;
};

using Constraint = std::variant<CircleConstraint>;

}  // namespace physics
