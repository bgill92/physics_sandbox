#pragma once

#include "physics.hpp"
#include <Eigen/Dense>

class Object
{
public:

	Object() = delete;

	Object(const double mass, const physics::stateVector& state) : mass_{mass}, state_{state} {};

	// Getters

	double getMass() {return mass_;}

	Eigen::Vector3d getForces() {return force_accumulator_;};	

	const physics::stateVector getState() {return state_;} const;

	// Setters

	void setState(const physics::stateVector& state) {state_ = state;};

	void addState(const physics::stateVector& state) {state_ += state;};

	void addForces(const Eigen::Vector3d& forces) {force_accumulator_ += forces;};

	void clearForces() {this->force_accumulator_ = {0, 0, 0};};	

private:

	double mass_;
	physics::stateVector state_;
  Eigen::Vector3d force_accumulator_ = {0, 0, 0};

};