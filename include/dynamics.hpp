#pragma once

#include <Eigen/Dense>

#include "common.hpp"

namespace dynamics
{

	struct PointMass
	{
		PointMass() = delete;

		PointMass(const double mass, const double timestep) : state_{mass}, timestep_{timestep} {B *= timestep;};
		PointMass(const double mass, const double timestep, const physics::stateVector& state) : state_{mass, state}, timestep_{timestep} {B *= timestep;};

    physics::stateVector derivativeAtState(const physics::stateVector& derivative, const double timestep) const;

    physics::State& getState() {return state_;}

    private:
    physics::State state_;
    double timestep_;

		// A and B matrices for simple linear Newtonian motion
	  // The A matrix in this case just gets the velocities of the state
	  physics::AMatrix A{ { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 }, //
	  					 { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 }, //
	  					 { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 }, //
	             { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, //
	             { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, //
	             { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

	  // The B matrix gets the acceleration due to the forces
		physics::BMatrix B{ { 0.0, 0.0, 0.0 }, //
           { 0.0, 0.0, 0.0 }, //
           { 0.0, 0.0, 0.0 }, //
           { 1, 0.0, 0.0 }, //
           { 0.0, 1, 0.0 }, //
    	     { 0.0, 0.0, 1 } }; //

	};

} // namespace dynamics
