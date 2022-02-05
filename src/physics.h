#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>

struct twoD {
	twoD(double x, double y) : x(x), y(y);
	double x; // Value in x direction
	double y; // Value in y direction
}


class Object {

	public:

		Object(double x, double y, double m): pos(x,y), _m(m) {};

		// Position
		twoD pos;
		// Velocity
		twoD vel;		
		// Acceleration
		twoD acc;
		// Force
		twoD F;
		// Mass
		double _m;

	private:

};

class Box : public Object {
	public:
		Box(double x, double y, double width, double height, double m):
		Object(x,y), _width(width), _height(height);
	private:
		int _width;
		int _height;
		float _termSpeed{10}; // Fastest speed of the object 
};


class Surface {

	public:

	private:

};

class World {

	public:

		void Run();

		void Step();

		void putBlock(double x, double y, int width, int height, double m);

	private: 		

		std::vector<Object> _objects; // Vector of objects
		std::vector<Surface> _surfaces; // Vector of surfaces
		float dt{1/kPhysicsFreq}; // Time resolution

};




#endif