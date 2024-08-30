#pragma once

#include <Eigen/Dense>
#include "object.hpp"

class Object;

namespace physics
{

	constexpr size_t STATE_VECTOR_SIZE {6};

  constexpr size_t STATE_VECTOR_X_POS_IDX = 0;  // x position
  constexpr size_t STATE_VECTOR_Y_POS_IDX = 1;  // y position
  constexpr size_t STATE_VECTOR_Z_POS_IDX = 2;  // z position
  
  constexpr size_t STATE_VECTOR_X_LIN_VEL_IDX = 7;  // x linear velocity
  constexpr size_t STATE_VECTOR_Y_LIN_VEL_IDX = 8;  // y linear velocity
  constexpr size_t STATE_VECTOR_Z_LIN_VEL_IDX = 9;  // z linear velocity

	// State is x, y, z, vx, vy, vz
	using stateVector = VectorNd<STATE_VECTOR_SIZE>;

	void update(Object& object, const double timestep);

	void applyGravity(Object& object);

};


