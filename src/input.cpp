#include "input.hpp"

#include "particle.hpp"
#include "rectangle.hpp"

#include <SFML/Window/Keyboard.hpp>

#include <variant>
#include <iostream>

namespace input
{

void InputProcessor::processInput()
{
  // std::cout << "object_force" << object_force << "\n";

  if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
  {
    object_force_ = physics::Force{ -100, 0, 0 };
    std::cout << "Pressing Left\n";
  }
  else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
  {
    object_force_ = physics::Force{ 100, 0, 0 };
    std::cout << "Pressing Right\n";
  }
  else
  {
    object_force_ = physics::Force{ 0, 0, 0 };
  }
}

}  // namespace input