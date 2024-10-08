#include "object.hpp"

void applyGravity(Object& object)
{
  // Force of gravity is mass * acceleration due to gravity, duh
  object.addForces({ 0, -90.81 * object.getMass(), 0 });
}