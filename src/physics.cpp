#include "physics.hpp"
#include "object.hpp"

namespace physics
{

	void update(Object& object, const double timestep)
	{
	  // Update is Runge-Kutta 4
	  // Essentially we calculate the derivative at four points, take a weighted average of the derivatives,
	  // and find the next state using that weighted derivative average
	  const auto evaluateDerivative = [&object](const stateVector& derivative, const double timestep)
	  {
	      stateVector state;
	      // Calculate the next state given the derivative
	      state.segment(0, 3) = object.getState().segment(0, 3) + derivative.segment(0, 3) * timestep;
	      // [dx, dy, dz] = [ dx, dy, dz] + [ddx, ddy, ddz] * timestep
	      state.segment(3, 3) = object.getState().segment(3, 3) + derivative.segment(3, 3) * timestep;

	      // Calculate the derivative at the next state
	      stateVector new_derivative;
	      new_derivative.segment(0, 3) = state.segment(3, 3);
	      new_derivative.segment(3, 3) = object.getForces()/object.getMass();

	      return new_derivative;
	  };

	  // k1 is the slope at the beginning of the time step
	  // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
	  // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the midpoint.
	  // Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate of the slope at the endpoint.
	  const auto k1 = evaluateDerivative(stateVector(), 0.0);
	  const auto k2 = evaluateDerivative(k1, timestep/2);
	  const auto k3 = evaluateDerivative(k2, timestep/2);
	  const auto k4 = evaluateDerivative(k3, timestep);

	  object.addState((1.0/6.0) * timestep * (k1 + 2.0*k2 + 2.0*k3 + k4));
	}

}