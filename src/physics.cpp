#include "physics.h"

void World::Run() {

	// Run Dynamics

}

void World::Step() {
		
}

void World::putBlock(double x, double y, int width, int height, double m) {
	this->_objects.emplace_back(x,y,width,height, m);
}