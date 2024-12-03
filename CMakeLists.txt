cmake_minimum_required(VERSION 3.22)
project(physics_sandbox)
set(CMAKE_CXX_STANDARD 23)

# Compiler warnings for GCC and Clang
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SFML COMPONENTS graphics window REQUIRED)
find_package(GTest REQUIRED)

# Add interface library for nlohmann::json
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE ${PROJECT_SOURCE_DIR}/third_party/nlohmann)
target_compile_features(nlohmann_json INTERFACE cxx_std_23)

# Add interface library for tl::expected
add_library(tl_expected INTERFACE)
target_include_directories(tl_expected INTERFACE ${PROJECT_SOURCE_DIR}/third_party/tl)
target_compile_features(tl_expected INTERFACE cxx_std_20)

# Add the physics_engine library
add_library(physics_engine
    src/common_graphics.cpp
    src/constraints.cpp
    src/control.cpp
    src/dynamics.cpp
    src/graphics.cpp
    src/input.cpp
    src/particle.cpp
    src/physics.cpp
    src/rectangle.cpp
    src/simulator.cpp
    src/sensor.cpp
    src/utils.cpp
    src/utils_control.cpp
)
target_include_directories(physics_engine PUBLIC include)
target_link_libraries(physics_engine PUBLIC
    sfml-graphics
    sfml-window
    Threads::Threads
    Eigen3::Eigen
    nlohmann_json
    tl_expected
)
target_compile_features(physics_engine PUBLIC cxx_std_23)

# List of executables
set(EXECUTABLES
    physics_sandbox
    constraints_example
    double_pendulum
    n_pendulum
    linear_constraint_example
    rectangle_example
    inverted_pendulum
)

# Add executables and link to physics_engine
foreach(exec ${EXECUTABLES})
    add_executable(${exec} src/${exec}.cpp)
    target_link_libraries(${exec} PRIVATE physics_engine)
endforeach()

# Enable testing
enable_testing()

# Add test executable
add_executable(test_dynamics test/test_dynamics.cpp)
target_link_libraries(test_dynamics PRIVATE physics_engine GTest::GTest GTest::Main)
