#include "dynamics.hpp"

#include <iostream>

namespace dynamics
{
physics::State PointMass::stateDerivative(const physics::State& derivative_at_state, const double timestep) const
{
  return A * (this->state_ + derivative_at_state * timestep) + B * timestep * this->force_;
}
}  // namespace dynamics
