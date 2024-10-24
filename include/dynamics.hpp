#pragma once

#include <Eigen/Dense>

#include "common.hpp"

namespace dynamics
{

struct DynamicsBase
{

  virtual ~DynamicsBase() = default;

  virtual physics::State stateDerivative(const physics::State& derivative_at_state, const double timestep) const = 0;

  virtual physics::State& getState() = 0;

  virtual physics::Force& getForce() = 0;

  virtual double getMass() const = 0;

  virtual void setState(const physics::State&) = 0;

  virtual void setForce(const physics::Force&) = 0;

  virtual void clearForce() = 0;

};

struct PointMass final : public DynamicsBase
{
  PointMass() = delete;

  PointMass(const double mass) : mass_{ mass } { state_.setZero(); force_.setZero();};

  PointMass(const double mass, const physics::State& state)
    : mass_{ mass }, state_{ state } { force_.setZero(); };

  // Overwritten functions
  physics::State stateDerivative(const physics::State& derivative_at_state, const double timestep) const override;

  physics::State& getState() override {return state_;};

  physics::Force& getForce() override {return force_;};

  double getMass() const override {return mass_;};

  void setState(const physics::State& state) override {state_ = state;};

  void setForce(const physics::Force& force) override {force_ = force;};

  void clearForce() override {force_.setZero();};

private:
  double mass_;
  physics::State state_;
  physics::Force force_;

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
