#include "dynamics.hpp"

#include <iostream>

namespace dynamics
{
physics::stateVector PointMass::stateDerivative(const physics::stateVector& derivative_at_state, const double timestep) const override
{
  return A * (this->getState().getStateVector() + derivative_at_state * timestep) + B * timestep * this->getState().getForceVector();
}
}  // namespace dynamics
