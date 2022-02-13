#include "physics.h"

#include <iostream>
#include <cmath>

twoD operator+(const twoD &lhs, const twoD &rhs) {
	return twoD{lhs.x + rhs.x, lhs.y + rhs.y};
}

twoD operator*(const twoD &lhs, const float &num) {
	return twoD{lhs.x*num,lhs.y*num};
}

twoD operator*(const float &num, const twoD &rhs) {
	return twoD{rhs.x*num,rhs.y*num};
}

twoD operator/(const twoD &lhs, const float &num) {
	return twoD{lhs.x/num,lhs.y/num};
}
	
std::ostream& operator<<(std::ostream& os, const twoD& vec){
	os << "(" << vec.x << "," << vec.y << ")\n";
	return os;
}

void World::Run() {

	// Run Dynamics

}


void World::Step() {

	// Step through the dynamics
	twoD acc_world{0,-9.81}; // gravity is -9.81 m/s^2

	// Iterate over forces, velocities, and speed over one time step
	for (auto &obj:_objects) {

		// Calculate force at current time step
		obj->F = obj->F + acc_world*obj->_m;

		// Calculate velocity at current timestep
		obj->vel = obj->vel + (obj->F/obj->_m)*dt;

		if (std::abs(obj->vel.y) > std::abs(obj->_termSpeed)) {
			obj->vel.y = obj->_termSpeed;
		}		

		// Calculate position at current timestep
		obj->pos = obj->pos + (obj->vel*dt) + (0.5*(obj->F/obj->_m)*(dt*dt));	

		obj->F = twoD{0,0};

	}

	// Check for collision and resolve;
	for (auto &obj:_objects) {
		obj->checkCollision();
		if (obj->isColliding()) {
			obj->resolveCollision();
		}
	}

	// // Resolve collision
	// for (auto &obj:_objects) {
	// 	obj->checkCollision();
	// }	

}

void World::putBlock(double x, double y, int width, int height, double m) {
	// this->_objects.emplace_back(x,y,width,height, m);
	// std::cout << x << ", " << y << ", " << width << ", " << height << ", " << m << "\n";
	this->_objects.push_back(std::make_unique<Box>(x,y,width,height, m));
}