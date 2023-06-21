#include <iostream>
#include <memory>
// #include "physics.hpp"
#include "graphics.hpp"
#include "manager.hpp"

int main() {	

	// Create a sandbox manager
	auto man = std::make_unique<Manager>();

	// Initialize graphics/physics/etc.
	man->initialize();

	// Run the sandbox manager
	man->run();

	return 0;

}