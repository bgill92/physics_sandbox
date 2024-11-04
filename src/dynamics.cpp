#include "dynamics.hpp"

#include <iostream>

namespace dynamics
{
physics::State PointMass::stateDerivative(const physics::State& state, const double timestep) const
{
  return A * state + B * (this->force_ / getMass());
}
}  // namespace dynamics
