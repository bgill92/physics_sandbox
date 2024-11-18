cmake_minimum_required(VERSION 3.22)
project(physics_sandbox)
set(CMAKE_CXX_STANDARD 23)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

include(FetchContent)

find_package(Eigen3 REQUIRED)
find_package(SFML COMPONENTS graphics REQUIRED)

add_library(json INTERFACE)
add_library(nlohmann::json ALIAS json)
target_include_directories(json INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/nlohmann>)
target_compile_features(json INTERFACE cxx_std_23)

add_library(physics_engine src/common_graphics.cpp src/constraints.cpp src/dynamics.cpp src/graphics.cpp src/particle.cpp src/physics.cpp src/rectangle.cpp src/simulator.cpp)
target_include_directories(physics_engine PRIVATE include)
target_link_libraries(physics_engine PRIVATE sfml-graphics pthread Eigen3::Eigen)

# Add the executables
add_executable(physics_sandbox src/physics_sandbox.cpp)
target_include_directories(physics_sandbox PRIVATE include)
target_link_libraries(physics_sandbox PRIVATE physics_engine sfml-graphics pthread Eigen3::Eigen nlohmann::json)

add_executable(constraints_example src/constraints_example.cpp)
target_include_directories(constraints_example PRIVATE include)
target_link_libraries(constraints_example PRIVATE physics_engine sfml-graphics pthread Eigen3::Eigen nlohmann::json)

add_executable(double_pendulum src/double_pendulum.cpp)
target_include_directories(double_pendulum PRIVATE include)
target_link_libraries(double_pendulum PRIVATE physics_engine sfml-graphics pthread Eigen3::Eigen nlohmann::json)

add_executable(n_pendulum src/n_pendulum.cpp)
target_include_directories(n_pendulum PRIVATE include)
target_link_libraries(n_pendulum PRIVATE physics_engine sfml-graphics pthread Eigen3::Eigen nlohmann::json)

add_executable(linear_constraint_example src/linear_constraint_example.cpp)
target_include_directories(linear_constraint_example PRIVATE include)
target_link_libraries(linear_constraint_example PRIVATE physics_engine sfml-graphics pthread Eigen3::Eigen nlohmann::json)

add_executable(rectangle_example src/rectangle_example.cpp)
target_include_directories(rectangle_example PRIVATE include)
target_link_libraries(rectangle_example PRIVATE physics_engine sfml-graphics pthread Eigen3::Eigen nlohmann::json)

find_package(GTest REQUIRED)
include_directories(include ${GTEST_INCLUDE_DIRS})

add_executable(test_dynamics test/test_dynamics.cpp)
target_link_libraries(test_dynamics PRIVATE ${GTEST_LIBRARIES} physics_engine pthread Eigen3::Eigen)
