// #pragma once

// #include <memory>

// #include "common.hpp"
// #include "dynamics.hpp"
// #include "graphics.hpp"

// /**
//  * @brief ObjectBase class which mostly has to do with getting and setting state at the moment
//  */
// class ObjectBase
// {
// public:

// 	// A Base class should have a virtual destructor per https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#c35-a-base-class-destructor-should-be-either-public-and-virtual-or-protected-and-non-virtual
// 	virtual ~ObjectBase() = default;

// 	// Getters
// 	virtual dynamics::DynamicsBase& getDynamics() = 0;

//   virtual graphics::GraphicsBase& getGraphics() = 0;

//   // Collision check against the walls of the environment
//   virtual void collisionCheckWall(const double WINDOW_HEIGHT, const double WINDOW_WIDTH) = 0;

// };