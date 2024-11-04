#pragma once

#include <SFML/Graphics.hpp>

#include "common.hpp"

class Particle;

namespace graphics
{

/**
 * @brief      Contains the necessary information for drawing a circle using SFML
 */
struct CircleGraphics
{
  CircleGraphics() = delete;
  CircleGraphics(const double radius, const sf::Color& color = sf::Color::Red) : radius_{ radius }
  {
    circle_ = sf::CircleShape(radius);
    circle_.setOrigin(radius, radius);
    circle_.setFillColor(color);
  }

  sf::CircleShape& getShape()
  {
    return circle_;
  };

  /**
   * @brief      Sets the draw position of the circle
   *
   * @param[in]  state          The state of the object
   * @param[in]  WINDOW_HEIGHT  The window height
   */
  void setDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT)
  {
    circle_.setPosition(convertToDrawPosition(state, WINDOW_HEIGHT));
  }

private:
  double radius_;
  sf::CircleShape circle_;

  /**
   * @brief      Converts the y position from physics space to draw space
   *
   * @param[in]  state          The state
   * @param[in]  WINDOW_HEIGHT  The window height
   *
   * @return     A SFML vector of x and y position that it uses for drawing
   */
  sf::Vector2f convertToDrawPosition(const physics::State& state, const unsigned int WINDOW_HEIGHT);
};

/**
 * @brief      A struct which is intended to draw an object using std::visit
 */
struct Draw
{
  Draw() = delete;
  Draw(const Config& config, sf::RenderWindow& window) : config_{ config }, window_{ window }
  {
  }

  /**
   * @brief      Draws a particle
   *
   * @param      particle  The particle
   */
  void operator()(Particle& p);

private:
  const Config& config_;
  sf::RenderWindow& window_;
};

/**
 * @brief      The manager for drawing objects
 */
struct DrawerManager
{
  DrawerManager() = delete;
  DrawerManager(const Config& config, std::vector<Object>& objects, sf::RenderWindow& window)
    : objects_{ objects }, drawer_{ config, window }
  {
  }

  /**
   * @brief      Draws an object from the objects vector that the DrawerManager holds
   *
   * @param[in]  idx   The index
   */
  void drawObject(const size_t idx);

private:
  std::vector<Object>& objects_;
  Draw drawer_;
};

}  // namespace graphics