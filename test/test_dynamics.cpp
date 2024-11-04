#include <gtest/gtest.h>

#include "common.hpp"
#include "integrator.hpp"
#include "dynamics.hpp"

TEST(Dynamics, trivial_tests)
{
  double mass = 12.34;

  dynamics::PointMass dynamics{ mass };

  EXPECT_EQ(dynamics.getMass(), mass);
}

TEST(Dynamics, test_pointmass_dynamics_no_force)
{
  // GIVEN a pointmass dynamics of a certain mass and at a certain state and with no force applied to it
  double mass = 1;
  physics::State state{ 1, 0, 0, 1, 0, 0 };
  double timestep = 0.1;
  dynamics::PointMass dynamics{ mass, state };

  EXPECT_EQ(dynamics.getState(), state);

  const auto derivative_func = [&dynamics](const physics::State& derivative_at_state,
                                           const double timestep) -> physics::State {
    return dynamics.stateDerivative(derivative_at_state, timestep);
  };

  const auto result = integrator::RK4<physics::State>(dynamics.getState(), timestep, derivative_func);

  // THEN the addition to the current state should be the expected value
  physics::State expected_result{ 0.1, 0, 0, 0, 0, 0 };
  EXPECT_EQ(result, expected_result);
}

TEST(Dynamics, test_pointmass_dynamics_yes_force)
{
  // GIVEN a pointmass dynamics of a certain mass and at a certain state and with a force applied to it
  double mass = 2;
  physics::State state{ 1, 0, 0, 1, 0, 0 };
  double timestep = 0.1;

  physics::Force force{ 10, -10, 0 };

  dynamics::PointMass dynamics{ mass, state };

  EXPECT_EQ(dynamics.getState(), state);

  dynamics.setForce(force);

  EXPECT_EQ(dynamics.getForce(), force);

  // WHEN The state derivative is defined and the dynamics are simulated via RK4
  const auto derivative_func = [&dynamics](const physics::State& derivative_at_state,
                                           const double timestep) -> physics::State {
    return dynamics.stateDerivative(derivative_at_state, timestep);
  };

  const auto result = integrator::RK4<physics::State>(dynamics.getState(), timestep, derivative_func);

  // THEN the addition to the current state should be the expected value
  physics::State expected_result{ 0.125, -0.025, 0, 0.5, -0.5, 0 };
  EXPECT_EQ(result, expected_result);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}