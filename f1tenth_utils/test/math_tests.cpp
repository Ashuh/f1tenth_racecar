#include <gtest/gtest.h>

#include "f1tenth_utils/math.h"

TEST(DistanceTest, handlesZero)
{
  EXPECT_DOUBLE_EQ(calculateDistance(0, 0, 0, 0), 0);
  EXPECT_DOUBLE_EQ(calculateDistance(1, 1, 1, 1), 0);
}

TEST(DistanceTest, handlesNW)
{
  EXPECT_DOUBLE_EQ(calculateDistance(1, 1, 2, 2), sqrt(2));
}

TEST(DistanceTest, handlesNE)
{
  EXPECT_DOUBLE_EQ(calculateDistance(1, 1, 2, 0), sqrt(2));
}

TEST(DistanceTest, handlesSW)
{
  EXPECT_DOUBLE_EQ(calculateDistance(1, 1, 0, 2), sqrt(2));
}

TEST(DistanceTest, handlesSE)
{
  EXPECT_DOUBLE_EQ(calculateDistance(1, 1, 0, 0), sqrt(2));
}

TEST(CurvatureTest, handlesStraightLine)
{
  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.position.y = 1;
  pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.x = 2;
  p.y = 1;

  EXPECT_DOUBLE_EQ(calculateCurvature(pose, p), 0);
}

TEST(CurvatureTest, handlesLeftArc)
{
  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.position.y = 1;
  pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.x = 2;
  p.y = 2;

  EXPECT_GT(calculateCurvature(pose, p), 0);
}

TEST(CurvatureTest, handlesRightArc)
{
  geometry_msgs::Pose pose;
  pose.position.x = 1;
  pose.position.y = 1;
  pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.x = 2;
  p.y = 0;

  EXPECT_LT(calculateCurvature(pose, p), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
