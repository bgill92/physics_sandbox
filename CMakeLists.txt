cmake_minimum_required(VERSION 3.16)
project(physics_sandbox)
set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

include(FetchContent)

find_package(Eigen3 REQUIRED)
find_package(SFML COMPONENTS graphics REQUIRED)

#FetchContent_Declare(
#  yaml-cpp
#  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
#  GIT_TAG 84459a7f982ea4d10e943237b2e9c71afdab6a45)
#
#FetchContent_MakeAvailable(yaml-cpp)

# Add the executable
add_executable(physics_sandbox src/main.cpp src/object.cpp src/particle.cpp src/physics.cpp)

target_include_directories(physics_sandbox PRIVATE include)
target_link_libraries(physics_sandbox sfml-graphics pthread Eigen3::Eigen)

#target_link_libraries(physics_sandbox sfml-graphics pthread Eigen3::Eigen yaml-cpp::yaml-cpp)
#target_precompile_headers(physics_sandbox PRIVATE <rclcpp/rclcpp.hpp>)