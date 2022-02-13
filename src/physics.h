#ifndef PHYSICS_H
#define PHYSICS_H

#include <vector>
#include <memory>

#include <iostream>

#include "ini.h"

struct twoD {
	twoD(double x = 0, double y = 0) : x(x), y(y) {};
	double x; // Value in x direction
	double y; // Value in y direction

	// friend twoD operator+(const twoD &lhs, const twoD &rhs);

	// twoD operator+ (const twoD &vec) const {return twoD{x+vec.x,y+vec.y};};

};

enum class ObjType { Box, Circle };

class Object {

	public:

		Object(double x, double y, double m): pos{x,y}, _m(m) {};

		// Position
		twoD pos;
		// Velocity
		twoD vel;		
		// Force
		twoD F;
		// Mass
		double _m;
		float _termSpeed{-50}; // Fastest speed of the object 

		void setCollision(bool val) {_isColliding = val;};

		virtual void checkCollision(){};

		bool isColliding(){return _isColliding;};		

		virtual void resolveCollision(){};

		virtual twoD getPos() const {};

		virtual ObjType getObjType() const {};

		// virtual Object* getThis(){return this;};

	private:

		bool _isColliding{false};

};

class Box : public Object {
	public:
		Box(double x, double y, double width, double height, double m = 1):
		Object(x,y,m), _width(width), _height(height) {};
		// Object(x,y,m), _width(width), _height(height) { std::cout << this->pos.x << " " << this->pos.y << " " << this->_m << " " << this->_width << " " << this->_height << "\n"; };
		virtual void checkCollision() { setCollision((this->pos.y) - this->_height <= 0);};
		virtual void resolveCollision(){ this->pos.y = this->_height;};
		virtual twoD getPos() const {return this->pos;};
		virtual ObjType getObjType() const {return ObjType::Box;};
		int getWidth() const {return _width;};
		int getHeight() const {return _height;};
		// Box* getThis() override {return this;};
	private:
		int _width;
		int _height;
};


// class Surface {

// 	public:

// 	private:

// };

class World {

	public:

		void Run();

		void Step();

		void putBlock(double x, double y, int width, int height, double m);

		std::vector<std::unique_ptr<Object>> * getObjectsPtr(){return &_objects;};

	private: 		

		std::vector<std::unique_ptr<Object>> _objects; // Vector of pointers to objects
		// std::vector<Surface> _surfaces; // Vector of surfaces
		float dt{1.0/(float)kPhysicsFreq}; // Time resolution

};

#endif