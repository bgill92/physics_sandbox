#include "dynamics.hpp"

#include <iostream>

namespace dynamics
{
// physics::stateVector PointMass::derivativeAtState(const physics::stateVector& current_state, const
// physics::stateVector& derivative, const physics::commandVector& forces, const double timestep) const
physics::stateVector derivativeAtState(const physics::stateVector& derivative, const double timestep) const;
{
  return A * (current_state + derivative * timestep) + B * forces;
}
}  // namespace dynamics
