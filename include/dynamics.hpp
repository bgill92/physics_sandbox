#pragma once

#include <Eigen/Dense>

#include "common.hpp"

namespace dynamics
{

struct DynamicsBase
{

  virtual ~DynamicsBase() = default;

  virtual physics::stateVector stateDerivative(const physics::stateVector& derivative_at_state, const double timestep) const = 0;
}

struct PointMass final : public DynamicsBase
{
  PointMass() = delete;

  PointMass(const double mass) : state_{ mass } {};

  PointMass(const double mass, const physics::stateVector& state)
    : state_{ mass, state } {};


  physics::State& getState()
  {
    return state_;
  }

  const physics::State& getState() const
  {
    return state_;
  }

  // Overwritten function
  physics::stateVector stateDerivative(const physics::stateVector& derivative_at_state, const double timestep) const override;

private:
  physics::State state_;

  // A and B matrices for simple linear Newtonian motion
  // The A matrix in this case just gets the velocities of the state
  physics::AMatrix A{ { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 },  //
                      { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },  //
                      { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },  //
                      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  //
                      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  //
                      { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  // The B matrix gets the acceleration due to the forces
  physics::BMatrix B{ { 0.0, 0.0, 0.0 },  //
                      { 0.0, 0.0, 0.0 },  //
                      { 0.0, 0.0, 0.0 },  //
                      { 1, 0.0, 0.0 },    //
                      { 0.0, 1, 0.0 },    //
                      { 0.0, 0.0, 1 } };  //
};

}  // namespace dynamics
