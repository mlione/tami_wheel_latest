#include <gtest/gtest.h>

#include <limits>

#include "wheel_perception/dwa_controller/DWAPlanner.hpp"

namespace {
using wheel_control::dwa::DWAPlanner;
using wheel_control::dwa::ObstaclePoint;
using wheel_control::dwa::Pose2D;
using wheel_control::dwa::RoadModel;
using wheel_control::dwa::Velocity;

TEST(DWAPlanner, SelectsForwardTrajectoryOnClearRoad) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.weight_obstacle = 0.0;
  DWAPlanner planner(config);
  RoadModel road{true, true, 1.0, 3.0, 1.0, 0.0};

  const auto result = planner.plan(Pose2D{}, Velocity{}, road, {}, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.best.command.linear, 0.0);
  EXPECT_NEAR(result.best.command.angular, 0.0, config.angular_resolution);
}

TEST(DWAPlanner, NeverCrossesRoadBoundary) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  DWAPlanner planner(config);
  RoadModel narrow_road{true, true, 0.55, 1.1, 0.55, 0.0};

  const auto result = planner.plan(Pose2D{}, Velocity{}, narrow_road, {}, 0.0);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.best.inside_road);
}

TEST(DWAPlanner, RejectsTrajectoriesBlockedAcrossTheRoad) {
  DWAPlanner::Config config;
  config.max_acceleration = 0.5;
  config.max_angular_acceleration = 10.0;
  config.robot_radius = 0.4;
  DWAPlanner planner(config);
  RoadModel road{true, true, 1.0, 3.0, 1.0, 0.0};
  std::vector<ObstaclePoint> wall;
  for (double y = -1.5; y <= 1.5; y += 0.1) wall.push_back({0.65, y});

  const auto result = planner.plan(Pose2D{}, Velocity{0.8, 0.0}, road, wall, 0.0);

  EXPECT_FALSE(result.valid);
}

TEST(DWAPlanner, RejectsInvalidOdometryState) {
  DWAPlanner planner(DWAPlanner::Config{});
  Pose2D invalid_state;
  invalid_state.yaw = std::numeric_limits<double>::quiet_NaN();

  const auto result = planner.plan(invalid_state, Velocity{}, RoadModel{}, {}, 0.0);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.candidates.empty());
}
}  // namespace
