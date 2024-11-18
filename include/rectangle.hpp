#pragma once

#include <array>
#include <cmath>

#include "common.hpp"
#include "dynamics.hpp"
#include "rectangle_graphics.hpp"

class Rectangle
{
public:
  Rectangle() = delete;

  Rectangle(const double length, const double height, const double mass, const physics::State& state,
            const sf::Color& color);

  dynamics::PointMass& getDynamics()
  {
    return point_mass_;
  }

  graphics::RectangleGraphics& getGraphics()
  {
    return graphics_;
  }

  double getInertia() const
  {
    return inertia_;
  }

  double getLength() const
  {
    return length_;
  }

  double getHeight() const
  {
    return height_;
  }

  // Calculates the vertices of the rectangle in the global frame
  std::array<Vertex, 4> calculateVerticesGlobal();

private:
  double length_;
  double height_;
  double inertia_;
  dynamics::PointMass point_mass_;
  graphics::RectangleGraphics graphics_;
  std::array<Vertex, 4> vertices_local_;
};
