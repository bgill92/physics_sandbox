#pragma once

#include "common.hpp"

namespace integrator
{

	template <typename T>
	T RK4(const T& value, const double timestep, T (*derivative_func)(const T&, const double));

};