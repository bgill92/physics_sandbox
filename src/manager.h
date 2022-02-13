#ifndef MANAGER_H
#define MANAGER_H

#include <memory>
#include <thread>
#include "graphics.h"
#include "physics.h"
#include "ini.h"

class Manager {

	public:

		Manager();

		void Run();

		void initialize();


	private:

		// Controls window and graphics and rendering
		std::unique_ptr<Graphics> _graphics;

		// World object
		std::unique_ptr<World> _world;

};

#endif