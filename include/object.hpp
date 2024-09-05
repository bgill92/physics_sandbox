#pragma once

#include "physics.hpp"
#include <Eigen/Dense>

class Object
{
public:
  Object() = delete;

  Object(const double mass, const physics::stateVector& state) : mass_{ mass }, state_{ state } {};

  // Getters

  double getMass()
  {
    return mass_;
  }

  Eigen::Vector3d getForces()
  {
    return force_accumulator_;
  };

  const physics::stateVector getState() const
  {
    return state_;
  };

  double getState(const size_t idx) const
  {
    return state_(idx);
  };

  // Setters

  void setState(const physics::stateVector& state)
  {
    state_ = state;
  };

  void setState(const size_t idx, const double value)
  {
    state_(idx) = value;
  };

  void addState(const physics::stateVector& state)
  {
    state_ += state;
  };

  void addForces(const Eigen::Vector3d& forces)
  {
    force_accumulator_ += forces;
  };

  void clearForces()
  {
    this->force_accumulator_ = { 0, 0, 0 };
  };

private:
  double mass_;
  physics::stateVector state_;
  Eigen::Vector3d force_accumulator_ = { 0, 0, 0 };
};

void applyGravity(Object& object);