#pragma once

#include <SFML/Graphics.hpp>

#include "common.hpp"
#include "constraints.hpp"

namespace graphics
{

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

struct DrawConstraint
{
  DrawConstraint() = delete;
  DrawConstraint(const Config& config, sf::RenderWindow& window, std::vector<Object>& objects,
                 std::vector<constraints::Constraint>& constraints)
    : config_{ config }, window_{ window }, objects_{ objects }, constraints_{ constraints }
  {
  }

  void operator()(constraints::CircleConstraint& constraint);

  void operator()(constraints::DistanceConstraint& constraint);

private:
  const Config& config_;
  sf::RenderWindow& window_;
  std::vector<Object>& objects_;
  std::vector<constraints::Constraint>& constraints_;
};

/**
 * @brief      The manager for drawing objects
 */
struct DrawerManager
{
  DrawerManager() = delete;
  DrawerManager(const Config& config, std::vector<Object>& objects, std::vector<constraints::Constraint>& constraints,
                sf::RenderWindow& window)
    : objects_{ objects }
    , constraints_{ constraints }
    , drawer_{ config, window }
    , constraint_drawer_{ config, window, objects_, constraints_ }
  {
  }

  /**
   * @brief      Draws an object from the objects vector that the DrawerManager holds
   *
   * @param[in]  idx   The index
   */
  void drawObject(const size_t idx);

  void drawConstraints();

private:
  std::vector<Object>& objects_;
  std::vector<constraints::Constraint>& constraints_;
  Draw drawer_;
  DrawConstraint constraint_drawer_;
};

}  // namespace graphics