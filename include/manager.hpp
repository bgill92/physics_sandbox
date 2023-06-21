#pragma once

#include <memory>
#include <thread>
#include <cstdlib>
#include <random>

#include "graphics.hpp"
#include "physics.hpp"
#include "ini.hpp"

class Manager {
public:

	Manager();

	void run();

	void initialize();

	void spawn_random_particles(size_t num_particles);

private:

	// Controls window and graphics and rendering
	std::unique_ptr<Graphics> graphics_;

	// World object
	std::unique_ptr<World> world_;

};