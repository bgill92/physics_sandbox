#pragma once

#include <Eigen/Dense>

namespace physics
{

constexpr size_t STATE_VECTOR_SIZE{ 6 };

constexpr size_t STATE_VECTOR_X_POS_IDX = 0;  // x position
constexpr size_t STATE_VECTOR_Y_POS_IDX = 1;  // y position
constexpr size_t STATE_VECTOR_THETA_IDX = 2;  // angle

constexpr size_t STATE_VECTOR_X_LIN_VEL_IDX = 3;  // x linear velocity
constexpr size_t STATE_VECTOR_Y_LIN_VEL_IDX = 4;  // y linear velocity
constexpr size_t STATE_VECTOR_ANG_VEL_IDX = 5;  // angular velocity

constexpr size_t COMMAND_VECTOR_SIZE{ 3 };

constexpr size_t COMMAND_VECTOR_X_LIN_VEL_IDX = 0;  // x linear velocity
constexpr size_t COMMAND_VECTOR_Y_LIN_VEL_IDX = 1;  // y linear velocity
constexpr size_t COMMAND_VECTOR_ANG_VEL_IDX = 2;  // z linear velocity

constexpr size_t COMMAND_VECTOR_X_ACCEL_IDX = 3;  // x acceleration
constexpr size_t COMMAND_VECTOR_Y_ACCEL_IDX = 4;  // y acceleration
constexpr size_t COMMAND_VECTOR_ANG_ACCEL_IDX = 5;  // angular acceleration

// State is x, y, theta, vx, vy, omega_z
using stateVector = Eigen::Matrix<double, STATE_VECTOR_SIZE, 1>;

// command is Fx, Fy, tau
using commandVector = Eigen::Matrix<double, COMMAND_VECTOR_SIZE, 1>;

using AMatrix = Eigen::Matrix<double, STATE_VECTOR_SIZE, STATE_VECTOR_SIZE>;

using BMatrix = Eigen::Matrix<double, STATE_VECTOR_SIZE, COMMAND_VECTOR_SIZE>;

class State
{
public:
  State() = delete;
  explicit State(const double mass) : mass_{ mass }
  {
  }
  State(const double mass, const stateVector& state) : mass_{ mass }, state_{ state }
  {
  }

  // Getters
  double getMass() const
  {
    return mass_;
  }

  const stateVector getStateVector() const
  {
    return state_;
  };

  Eigen::Vector3d& getForceVector()
  {
    return force_accumulator_;
  };

  const Eigen::Vector3d& getForceVector() const
  {
    return force_accumulator_;
  };

  double getStateVectorAtIdx(const size_t idx) const
  {
    return state_(idx);
  };

  // Setters
  void setStateVector(const stateVector& state)
  {
    state_ = state;
  };

  void setStateVectorAtIdx(const size_t idx, const double value)
  {
    state_(idx) = value;
  };

  void addStateVector(const stateVector& state)
  {
    state_ += state;
  };

  void setForceVector(const Eigen::Vector3d& forces)
  {
    force_accumulator_ = forces;
  };

  void addForceVector(const Eigen::Vector3d& forces)
  {
    force_accumulator_ += forces;
  };

  void clearForceVector()
  {
    this->force_accumulator_.setZero();
  };

private:
  double mass_ = 1;
  stateVector state_{ 0, 0, 0, 0, 0, 0 };
  physics::commandVector force_accumulator_{ 0, 0, 0 };
};

}  // namespace physics