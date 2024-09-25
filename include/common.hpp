#pragma once

#include <Eigen/Dense>

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

	class State
	{
	public:
		State() = delete;
	  State(const double mass) : mass_{mass}{}
	  State(const double mass, const stateVector& state): mass_{mass}, state_{state}{}

	  // Getters
	  double getMass() const
	  {
	    return mass_;
	  }

	  const stateVector getState() const
	  {
	    return state_;
	  };

	  Eigen::Vector3d& getForces()
	  {
	    return force_accumulator_;
	  };	  

	  double getState(const size_t idx) const
	  {
	    return state_(idx);
	  };

	  // Setters
	  void setState(const stateVector& state)
	  {
	    state_ = state;
	  };

	  void setState(const size_t idx, const double value)
	  {
	    state_(idx) = value;
	  };

	  void addState(const stateVector& state)
	  {
	    state_ += state;
	  };

	  void setForces(const Eigen::Vector3d& forces)
	  {
	    force_accumulator_ = forces;
	  };

	  void addForces(const Eigen::Vector3d& forces)
	  {
	    force_accumulator_ += forces;
	  };

	  void clearForces()
	  {
	    this->force_accumulator_.setZero();
	  };

	private:
    double mass_ = 1;	  
	  stateVector state_ {0, 0, 0, 0, 0, 0};
    physics::commandVector force_accumulator_ {0, 0, 0};
	};

}