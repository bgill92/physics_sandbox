#include "physics.hpp"

#include <iostream>
#include <cmath>

Particle::Particle(uint const id, Eigen::Vector3d const& position, Eigen::Vector3d const& velocity, std::array<uint, 3> const& color) {
	id_ = id;
	state_.position = position;
	state_.linear_velocity = velocity;
	color_ = color;
}

void World::spawn_particle(uint const id, Eigen::Vector3d const& position, Eigen::Vector3d const& velocity, std::array<uint, 3> const& color) {
	particles_.emplace_back(id, position, velocity, color);
}

void World::run() {

	// Run Dynamics

}

void World::step() {

	// Step through the dynamics
	// Iterate over forces, velocities, and speed over one time step
	
	std::vector<std::size_t> screenWidthAndHeightAndZ {kScreenWidth, kScreenHeight, 0};
	for (auto& obj:particles_) {
		// std::cout << count << "\n";

		// Calculate force at current time step
		// obj->F = obj->F + acc_world*obj->_m;

		// Calculate velocity at current timestep
		// obj->vel = obj->vel + (obj->F/obj->_m)*dt;

		// Calculate position at current timestep
		// obj->pos = obj->pos + (obj->vel*dt) + (0.5*(obj->F/obj->_m)*(dt*dt));

		// std::cout << "pre\n";		
		// std::cout << obj.get_state().position << "\n";
		// std::cout << "\n";
		for (size_t idx = 0; idx < obj.get_state().position.size(); idx++)
		{
			if (obj.get_state().position[idx] + obj.get_state().linear_velocity[idx]*dt > screenWidthAndHeightAndZ[idx])
			{
				obj.get_state().linear_velocity[idx] *= -1;	
			}
			if (obj.get_state().position[idx] + obj.get_state().linear_velocity[idx]*dt < 0)
			{
				obj.get_state().linear_velocity[idx] *= -1;	
			}
		}

		// Update position
		obj.get_state().position = obj.get_state().position + obj.get_state().linear_velocity*dt;

	}

}