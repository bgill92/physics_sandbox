#pragma once

#include <functional>
#include <iostream>

#include "common.hpp"

namespace integrator
{

	template<typename T>
	concept zeroable = requires (T a)
	{
	  a.setZero();
	};

	template <zeroable T>
	T RK4(const T& value, const double timestep, std::function<T(const T&, const double)> derivative_func)
	{
	  const auto bound_derivative_func = [&value, &timestep, &derivative_func](const T& delta_value) {
	    return derivative_func(value + delta_value, timestep);
	  };

	  // k1 is the slope at the beginning of the time step
	  // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
	  // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the
	  // midpoint. Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate
	  // of the slope at the endpoint.
	  const T k1 = bound_derivative_func(T().setZero());
	  const T k2 = bound_derivative_func(k1 * timestep * 0.5);
	  const T k3 = bound_derivative_func(k2 * timestep * 0.5);
	  const T k4 = bound_derivative_func(k3 * timestep);

	  return (1.0 / 6.0) * timestep * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
	}		

};