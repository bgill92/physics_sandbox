#ifndef MANAGER_H
#define MANAGER_H

#include <memory>
#include "graphics.h"
#include "ini.h"

class Manager {

	public:

		Manager();

		void Run();

		void initialize();

	private:

		// Controls window and graphics and rendering
		std::unique_ptr<Graphics> _graphics;

};

#endif