#pragma once

#include "common.hpp"

#include <vector>

namespace input
{

struct InputProcessor
{
  InputProcessor() = delete;
  InputProcessor(std::vector<Object>& objects, const size_t object_idx) : objects_{ objects }, object_idx_{ object_idx }
  {
  }

  void processInput();

  physics::Force getObjectForce() const
  {
    return object_force_;
  }

  size_t getObjectIdx() const
  {
    return object_idx_;
  }

private:
  std::vector<Object>& objects_;
  size_t object_idx_;
  physics::Force object_force_;
};

}  // namespace input