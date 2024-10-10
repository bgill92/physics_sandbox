#pragma once

#include "common.hpp"
#include "dynamics.hpp"

/**
 * @brief ObjectBase class which mostly has to do with getting and setting state at the moment
 */
class ObjectBase
{
public:

	// A Base class should have a virtual destructor per https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c35-a-base-class-destructor-should-be-either-public-and-virtual-or-protected-and-non-virtual
	virtual void ~ObjectBase() = default;

	// Getters
	virtual physics::State& getState() = 0;

	virtual physics::stateVector& getStateVector() = 0;

	virtual dynamics::DynamicsBase& getDynamics() = 0;

	virtual double getMass() = 0;

	virtual void draw() = 0;

	// virtual physics::commandVector& getForces() = 0;

	// virtual physics::commandVector& getAcceleration() = 0;


	// virtual double getMass() const = 0;

	// // Setters
	// virtual void setState(const physics::stateVector& state) = 0;

	// virtual void setForces(const physics::commandVector& forces) = 0;

}