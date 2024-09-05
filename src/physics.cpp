#include "physics.hpp"
#include "object.hpp"

#include <iostream>

namespace physics
{
	stateVector update(const AMatrix& A, const BMatrix& B, const stateVector& state, const commandVector& command, const double timestep)
	{
	  // Update is Runge-Kutta 4
	  // Essentially we calculate the derivative at four points, take a weighted average of the derivatives,
	  // and find the next state using that weighted derivative average
	  const auto evaluateDerivative = [&timestep](const AMatrix& A, const BMatrix& B, const stateVector& state, const stateVector& derivative, const commandVector& command)
	  {
	  	// Calculate the next state given the derivative, and then calculate the derivative at the next state
	  	return A*(state + derivative*timestep) + B*command;
	  };

	  // k1 is the slope at the beginning of the time step
	  // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
	  // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the midpoint.
	  // Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate of the slope at the endpoint.
	  const auto k1 = evaluateDerivative(A, B, state, stateVector(), command);
	  const auto k2 = evaluateDerivative(A, B, state, k1*timestep*0.5, command);
	  const auto k3 = evaluateDerivative(A, B, state, k2*timestep*0.5, command);
	  const auto k4 = evaluateDerivative(A, B, state, k3*timestep, command);

	  return (1.0/6.0) * timestep * (k1 + 2.0*k2 + 2.0*k3 + k4);
	}

	std::pair<AMatrix, BMatrix> generateAandBMatrices(const double timestep)
	{
		// A and B matrices for simple linear Newtonian motion
		// The A matrix in this case just gets the velocities of the state
	  AMatrix A {
	  						 {0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
								 {0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
								 {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
								 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
								 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
								 {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
								};

		// The B matrix gets the acceleration due to the forces
		BMatrix B {
								 {			0.0, 			0.0, 			0.0},
								 {			0.0, 			0.0, 			0.0},
								 {			0.0, 			0.0, 			0.0},
								 {timestep,      0.0, 			0.0},
								 {     0.0, timestep, 			0.0},
								 {     0.0, 			0.0, timestep}
								};

		return {A, B};

	}

	void updateObject(Object& object, const double timestep)
	{
		const auto AandB = generateAandBMatrices(timestep);

		object.addState(update(AandB.first, AandB.second, object.getState(), object.getForces()/object.getMass(), timestep));
	}
}