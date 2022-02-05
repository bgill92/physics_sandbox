cmake_minimum_required(VERSION 3.16)

add_definitions(-std=c++17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS} -pthread")

# Set project name
project(PhysicsSandbox)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS} src)

# Add the executable
add_executable(PhysicsSandbox src/main.cpp src/graphics.cpp src/manager.cpp)

target_link_libraries(PhysicsSandbox ${SDL2_LIBRARIES})
target_link_libraries(PhysicsSandbox pthread)