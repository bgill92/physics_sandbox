#include "input.hpp"

#include "particle.hpp"
#include "rectangle.hpp"

#include <SFML/Window/Keyboard.hpp>

#include <variant>
#include <iostream>

namespace input
{

// TODO: Put this into control functionality
void InputProcessor::processInput()
{
  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
  {
    object_force_ = physics::Force{ -100, 0, 0 };
  }
  else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
  {
    object_force_ = physics::Force{ 100, 0, 0 };
  }
  else
  {
    object_force_ = physics::Force{ 0, 0, 0 };
  }
}

}  // namespace input