#pragma once

#include <vector>
#include <memory>
#include <iostream>
#include <array>

#include "common.hpp"
#include "ini.hpp"

class Particle {
public:
	Particle(uint const id, Eigen::Vector3d const& position, Eigen::Vector3d const& velocity, std::array<uint, 3> const& color);
	common::State view_state() const {return state_;};
	common::State& get_state() {return state_;};
	std::array<uint, 3> view_color() const {return color_;};

private:
	double mass_ {1.0};
	common::State state_;
	std::array<uint, 3> color_;
	uint id_;
};

void find_collisions(std::vector<Particle> particles, float dt);

class World {
public:

	void run();

	void step();

	void spawn_particle(uint const id, Eigen::Vector3d const& position, Eigen::Vector3d const& velocity, std::array<uint, 3> const& color = {255, 255, 255});

	std::vector<Particle> const& get_particles() const {return particles_;}

private:
	std::vector<Particle> particles_; // Vector of particles
	float dt{1.0/(float)kPhysicsFreq}; // Time resolution
};