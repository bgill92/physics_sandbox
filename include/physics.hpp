#pragma once

#include <Eigen/Dense>

#include "common.hpp"

class Object;
class Particle;
class DrawableParticle;

namespace physics
{

stateVector update(const AMatrix& A, const BMatrix& B, const stateVector& state, const commandVector& command,
                   const double timestep);

void updateObject(Object& object, const double timestep);

void collisionCheckWall(Particle& particle, const double WINDOW_HEIGHT, const double WINDOW_WIDTH);

void collisionCheckOtherParticles(DrawableParticle& drawable_particle_1, DrawableParticle& drawable_particle_2);

};  // namespace physics
