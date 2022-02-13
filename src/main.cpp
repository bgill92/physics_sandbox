#include <iostream>
#include <memory>
#include "physics.h"
#include "graphics.h"
#include "manager.h"

int main() {	

	// Create a sandbox manager
	auto man = std::make_unique<Manager>();

	// Initialize graphics/physics/etc.
	man->initialize();

	// Run the sandbox manager
	man->Run();

	return 0;

}