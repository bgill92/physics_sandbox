#pragma once

#include "physics.hpp"
#include "object.hpp"

class Particle : public Object
{
public:
  Particle() = delete;

  Particle(const double radius, const double mass, const physics::stateVector& state)
    : Object{ mass, state }, radius_{ radius } {};

  double getRadius()
  {
    return radius_;
  };

private:
  double radius_;
};