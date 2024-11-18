#include "rectangle.hpp"

#include <Eigen/Geometry>

Rectangle::Rectangle(const double length, const double height, const double mass, const physics::State& state,
                     const sf::Color& color)
  : length_{ length }
  , height_{ height }
  , inertia_{ (1.0 / 12.0) * mass * (length * length + height * height) }
  , point_mass_{ mass, state }
  , graphics_{ length_, height_, color }
{
  // top left
  vertices_local_[0] = { -length_ / 2, height_ / 2 };
  // top right
  vertices_local_[1] = { length_ / 2, height_ / 2 };
  // bottom right
  vertices_local_[2] = { length_ / 2, -height_ / 2 };
  // bottom left
  vertices_local_[3] = { -length_ / 2, -height_ / 2 };
};

std::array<Vertex, 4> Rectangle::calculateVerticesGlobal()
{
  // Get the state (which is center of mass)
  auto& state = this->getDynamics().getState();

  // Get the angle of the rectangle
  const auto angle = state(physics::STATE_THETA_IDX);

  // Make a rotation matrix out of the angle
  const Eigen::Rotation2Dd rot(angle);

  std::array<Vertex, 4> vertices;

  // Rotate all of he vertices based off of the angle and translate them based on state
  for (size_t idx = 0; idx < vertices_local_.size(); idx++)
  {
    vertices[idx] = rot * vertices_local_[idx] + state.head<2>();
  }

  return vertices;
}