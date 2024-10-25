#pragma once

#include <variant>

#include <Eigen/Dense>

class Particle;

using Object = std::variant<Particle>;

namespace physics
{

constexpr size_t STATE_SIZE{ 6 };

constexpr size_t STATE_X_POS_IDX = 0;  // x position
constexpr size_t STATE_Y_POS_IDX = 1;  // y position
constexpr size_t STATE_THETA_IDX = 2;  // angle

constexpr size_t STATE_X_LIN_VEL_IDX = 3;  // x linear velocity
constexpr size_t STATE_Y_LIN_VEL_IDX = 4;  // y linear velocity
constexpr size_t STATE_ANG_VEL_IDX = 5;  // angular velocity

constexpr size_t FORCE_SIZE{ 3 };

constexpr size_t FORCE_X_IDX = 0;  // x Force
constexpr size_t FORCE_Y_IDX = 1;  // y Force
constexpr size_t FORCE_TAU_IDX = 2;  // Torque

// State is x, y, theta, vx, vy, omega_z
using State = Eigen::Matrix<double, STATE_SIZE, 1>;

// command is Fx, Fy, tau
using Force = Eigen::Matrix<double, FORCE_SIZE, 1>;

using AMatrix = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

using BMatrix = Eigen::Matrix<double, STATE_SIZE, FORCE_SIZE>;

}  // namespace physics

struct Config
{
	unsigned int window_height = 1000;
	unsigned int window_width = 1000;
	double timestep_physics = 0.1;
	int num_particles = 100;
	double particle_COR = 1.0;
	double pixels_to_meters_ratio = 100;
};
