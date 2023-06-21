cmake_minimum_required(VERSION 3.16)
project(physics_sandbox)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(Eigen3 REQUIRED)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS} include)

# Add the executable
add_executable(physics_sandbox src/main.cpp src/manager.cpp src/graphics.cpp src/physics.cpp)

target_link_libraries(physics_sandbox ${SDL2_LIBRARIES} pthread Eigen3::Eigen)