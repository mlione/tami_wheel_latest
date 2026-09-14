#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "wheel_perception/dwa_controller/DWAPlanner.hpp"

namespace {
using wheel_control::dwa::DWAPlanner;
using wheel_control::dwa::MotionHistory;
using wheel_control::dwa::ObstaclePoint;
using wheel_control::dwa::Pose2D;
using wheel_control::dwa::RoadModel;
using wheel_control::dwa::Trajectory;
using wheel_control::dwa::Velocity;

TEST(DWAPlanner, SelectsForwardTrajectoryOnClearRoad) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.weight_obstacle = 0.0;
  DWAPlanner planner(config);
  RoadModel road{true, true, 1.0, 3.0, 1.0, 0.0};

  const auto result = planner.plan(Pose2D{}, Velocity{}, road, {}, MotionHistory{}, 0.1);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.best.command.linear, 0.0);
  EXPECT_NEAR(result.best.command.angular, 0.0, config.angular_resolution);
}

TEST(DWAPlanner, EmptyCloudProducesStableRoadTrackingPath) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  DWAPlanner planner(config);
  RoadModel straight_road{true, true, 1.0, 3.0, 1.0, 0.0};

  const auto first =
      planner.plan(Pose2D{}, Velocity{}, straight_road, {}, MotionHistory{}, 0.1);
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.best.collision_free);
  EXPECT_TRUE(first.best.inside_road);
  EXPECT_GT(first.best.command.linear, 0.0);
  EXPECT_NEAR(first.best.command.angular, 0.0, config.angular_resolution);

  MotionHistory history;
  history.previous_command = first.best.command;
  history.valid = true;
  const auto second =
      planner.plan(Pose2D{}, first.best.command, straight_road, {}, history, 0.1);
  ASSERT_TRUE(second.valid);
  EXPECT_GT(second.best.command.linear, 0.0);
  EXPECT_NEAR(second.best.command.angular, first.best.command.angular,
              config.angular_resolution);
}

TEST(DWAPlanner, NeverCrossesRoadBoundary) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  DWAPlanner planner(config);
  RoadModel narrow_road{true, true, 0.55, 1.1, 0.55, 0.0};

  const auto result =
      planner.plan(Pose2D{}, Velocity{}, narrow_road, {}, MotionHistory{}, 0.1);

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

  const auto result = planner.plan(Pose2D{}, Velocity{0.8, 0.0}, road, wall,
                                   MotionHistory{}, 0.1);

  EXPECT_FALSE(result.valid);
}

TEST(DWAPlanner, RejectsInvalidOdometryState) {
  DWAPlanner planner(DWAPlanner::Config{});
  Pose2D invalid_state;
  invalid_state.yaw = std::numeric_limits<double>::quiet_NaN();

  const auto result = planner.plan(invalid_state, Velocity{}, RoadModel{}, {},
                                   MotionHistory{}, 0.1);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(DWAPlanner, DoesNotGenerateInPlaceTurningCandidates) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.minimum_turning_velocity = 0.001;
  DWAPlanner planner(config);
  RoadModel curved_road{true, true, 1.0, 3.0, 1.0, 0.6};

  const auto result =
      planner.plan(Pose2D{}, Velocity{}, curved_road, {}, MotionHistory{}, 0.1);

  ASSERT_TRUE(result.valid);
  for (const auto& candidate : result.candidates) {
    EXPECT_FALSE(candidate.command.linear < config.minimum_turning_velocity &&
                 std::abs(candidate.command.angular) > 1e-6);
  }
}

TEST(DWAPlanner, StartsStraightFromRestWithRealisticAccelerationLimit) {
  DWAPlanner::Config config;
  config.max_acceleration = 0.5;
  config.max_angular_acceleration = 1.5;
  config.minimum_turning_velocity = 0.001;
  DWAPlanner planner(config);
  RoadModel straight_road{true, true, 1.0, 3.0, 1.0, 0.0};

  const auto result =
      planner.plan(Pose2D{}, Velocity{}, straight_road, {}, MotionHistory{}, 0.05);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.best.command.linear, 0.0);
  EXPECT_NEAR(result.best.command.angular, 0.0, 1e-9);

  // From rest at 20 Hz the reachable speed is only 0.025 m/s. These rolling
  // turn candidates must still exist so an obstacle cannot trap the planner
  // in a stop -> no-path -> stop loop.
  const bool has_low_speed_turn = std::any_of(
      result.candidates.begin(), result.candidates.end(),
      [&config](const Trajectory& candidate) {
        return candidate.command.linear > config.minimum_turning_velocity &&
               candidate.command.linear < 0.10 &&
               std::abs(candidate.command.angular) > 1e-6;
      });
  EXPECT_TRUE(has_low_speed_turn);
}

TEST(DWAPlanner, SelectsLowSpeedRollingTurnFromRest) {
  DWAPlanner::Config config;
  config.max_acceleration = 0.5;
  config.max_angular_acceleration = 1.5;
  config.minimum_turning_velocity = 0.001;
  config.weight_heading = 10.0;
  config.weight_obstacle = 0.0;
  config.weight_velocity = 0.0;
  config.weight_road = 0.0;
  config.weight_smooth = 0.0;
  DWAPlanner planner(config);
  RoadModel left_turning_road{true, true, 1.0, 3.0, 1.0, 0.6};

  const auto result =
      planner.plan(Pose2D{}, Velocity{}, left_turning_road, {}, MotionHistory{}, 0.05);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.best.command.linear, config.minimum_turning_velocity);
  EXPECT_LT(result.best.command.linear, 0.10);
  EXPECT_GT(result.best.command.angular, 0.0);
}

TEST(DWAPlanner, RespectsMinimumTurningRadius) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.minimum_turning_velocity = 0.001;
  config.minimum_turning_radius = 0.6;
  DWAPlanner planner(config);

  const auto result = planner.plan(Pose2D{}, Velocity{}, RoadModel{}, {},
                                   MotionHistory{}, 0.1);

  ASSERT_TRUE(result.valid);
  for (const auto& candidate : result.candidates) {
    if (std::abs(candidate.command.angular) <= 1e-6) continue;
    EXPECT_GE(std::abs(candidate.command.linear / candidate.command.angular) + 1e-6,
              config.minimum_turning_radius);
  }
}

TEST(DWAPlanner, KeepsSafePreviousTrajectoryWhenScoresAreClose) {
  DWAPlanner::Config config;
  config.max_acceleration = 1.0;
  config.max_angular_acceleration = 2.0;
  config.score_switch_margin = 100.0;
  config.relative_switch_margin = 0.0;
  DWAPlanner planner(config);
  RoadModel road{true, true, 1.0, 3.0, 1.0, 0.0};
  MotionHistory history;
  history.previous_command = {0.5, 0.0};
  history.valid = true;

  const auto result =
      planner.plan(Pose2D{}, history.previous_command, road, {}, history, 0.1);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.best.command.linear, history.previous_command.linear, 1e-6);
  EXPECT_NEAR(result.best.command.angular, history.previous_command.angular, 1e-6);
}

TEST(DWAPlanner, RespectsAccelerationLimitsRelativeToCommandHistory) {
  DWAPlanner::Config config;
  config.max_acceleration = 0.5;
  config.max_angular_acceleration = 1.0;
  config.enable_trajectory_hold = false;
  DWAPlanner planner(config);
  RoadModel road{true, true, 1.0, 3.0, 1.0, 0.0};
  MotionHistory history;
  history.previous_command = {0.5, 0.2};
  history.valid = true;
  constexpr double dt = 0.1;

  const auto result =
      planner.plan(Pose2D{}, history.previous_command, road, {}, history, dt);

  ASSERT_TRUE(result.valid);
  EXPECT_LE(std::abs(result.best.command.linear - history.previous_command.linear),
            config.max_acceleration * dt + 1e-6);
  EXPECT_LE(std::abs(result.best.command.angular - history.previous_command.angular),
            config.max_angular_acceleration * dt + 1e-6);
}
}  // namespace
