#include "particle.hpp"

void applyGravity(Object& object)
{
	object.addForces({0, -9.81, 0};);
}