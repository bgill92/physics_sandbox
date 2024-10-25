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

add_library(json INTERFACE)
add_library(nlohmann::json ALIAS json)
target_include_directories(json INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/nlohmann>)
target_compile_features(json INTERFACE cxx_std_23)

# Add the executable
add_executable(physics_sandbox src/dynamics.cpp src/graphics.cpp src/physics_sandbox.cpp src/particle.cpp src/physics.cpp)

target_include_directories(physics_sandbox PRIVATE include)
target_link_libraries(physics_sandbox PRIVATE sfml-graphics pthread Eigen3::Eigen nlohmann::json)

add_executable(constraints_example src/dynamics.cpp src/graphics.cpp src/constraints_example.cpp src/particle.cpp src/physics.cpp)

target_include_directories(constraints_example PRIVATE include)
target_link_libraries(constraints_example PRIVATE sfml-graphics pthread Eigen3::Eigen nlohmann::json)
