#include <gtest/gtest.h>
#include <ros/ros.h>

#include "local_planner/cubic_velocity_time_profile.h"

TEST(CubicVelocityTimeProfile, boundaryConditions)
{
  const double step = 1.5;
  const double epsilon = 1e-10;

  for (int i = 0; i < 100; i++)
  {
    double v_i = i * step;

    for (int j = 0; j < 100; j++)
    {
      if (i == 0)
      {
        continue;
      }

      double v_f = j * step;

      for (int k = 1; k < 100; k++)
      {
        double s_f = k * step;

        CubicVelocityTimeProfile profile(v_i, v_f, s_f);
        EXPECT_NEAR(profile.getVelocityAtTime(0.0), v_i, epsilon);
        EXPECT_NEAR(profile.getFinalVelocity(), v_f, epsilon);
        EXPECT_NEAR(profile.getAccelerationAtTime(0), 0.0, epsilon);
        EXPECT_NEAR(profile.getFinalAcceleration(), 0.0, epsilon);
        EXPECT_NEAR(profile.getFinalDisplacement(), s_f, epsilon);
      }
    }
  }
}

TEST(CubicVelocityTimeProfile, timeAtDisplacement)
{
  const double step = 1.5;

  for (int i = 0; i < 100; i++)
  {
    double v_i = i * step;

    for (int j = 0; j < 100; j++)
    {
      if (i == 0)
      {
        continue;
      }

      double v_f = j * step;

      for (int k = 1; k < 100; k++)
      {
        double s_f = k * step;

        CubicVelocityTimeProfile profile(v_i, v_f, s_f);
        EXPECT_EQ(profile.getTimeAtDisplacement(0.0), 0.0);
        EXPECT_EQ(profile.getTimeAtDisplacement(s_f), profile.getEndTime());
        double t_mid = profile.getTimeAtDisplacement(s_f / 2);
        EXPECT_GT(t_mid, 0.0);
        EXPECT_LT(t_mid, profile.getEndTime());
      }
    }
  }
}

TEST(CubicVelocityTimeProfile, invalidArgs)
{
  EXPECT_THROW(CubicVelocityTimeProfile profile(0.0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(CubicVelocityTimeProfile profile(1.0, 1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(CubicVelocityTimeProfile profile(0.0, 0.0, 0.0), std::invalid_argument);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
