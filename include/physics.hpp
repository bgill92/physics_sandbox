#pragma once

#include <Eigen/Dense>

class Object;
class Particle;
class DrawableParticle;

namespace physics
{

constexpr size_t STATE_VECTOR_SIZE{ 6 };

constexpr size_t STATE_VECTOR_X_POS_IDX = 0;  // x position
constexpr size_t STATE_VECTOR_Y_POS_IDX = 1;  // y position
constexpr size_t STATE_VECTOR_Z_POS_IDX = 2;  // z position

constexpr size_t STATE_VECTOR_X_LIN_VEL_IDX = 3;  // x linear velocity
constexpr size_t STATE_VECTOR_Y_LIN_VEL_IDX = 4;  // y linear velocity
constexpr size_t STATE_VECTOR_Z_LIN_VEL_IDX = 5;  // z linear velocity

constexpr size_t COMMAND_VECTOR_SIZE{ 3 };

constexpr size_t COMMAND_VECTOR_X_LIN_VEL_IDX = 0;  // x linear velocity
constexpr size_t COMMAND_VECTOR_Y_LIN_VEL_IDX = 1;  // y linear velocity
constexpr size_t COMMAND_VECTOR_Z_LIN_VEL_IDX = 2;  // z linear velocity

constexpr size_t COMMAND_VECTOR_X_ACCEL_IDX = 3;  // x acceleration
constexpr size_t COMMAND_VECTOR_Y_ACCEL_IDX = 4;  // y acceleration
constexpr size_t COMMAND_VECTOR_Z_ACCEL_IDX = 5;  // z acceleration

// State is x, y, z, vx, vy, vz
using stateVector = Eigen::Matrix<double, STATE_VECTOR_SIZE, 1>;

// State is vx, vy, vz, ax, ay, az
using commandVector = Eigen::Matrix<double, COMMAND_VECTOR_SIZE, 1>;

using AMatrix = Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE>;

using BMatrix = Eigen::Matrix<double, STATE_VECTOR_SIZE, COMMAND_VECTOR_SIZE>;

stateVector update(const AMatrix& A, const BMatrix& B, const stateVector& state, const commandVector& command,
                   const double timestep);

void updateObject(Object& object, const double timestep);

void collisionCheckWall(Particle& particle, const double WINDOW_HEIGHT, const double WINDOW_WIDTH);

void collisionCheckOtherParticles(DrawableParticle& drawable_particle_1, DrawableParticle& drawable_particle_2);

};  // namespace physics
