#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "wheel_perception/dwa_controller/DWAPlanner.hpp"
#include "wheel_perception/core/road_geometry.hpp"

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
  RoadModel narrow_road{true, true, 1.0, 2.0, 1.0, 0.0};

  const auto result =
      planner.plan(Pose2D{}, Velocity{}, narrow_road, {}, MotionHistory{}, 0.1);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.best.inside_road);
}

TEST(DWAPlanner, RejectsRoadTooNarrowForFootprintAndMargins) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.robot_radius = 0.45;
  config.road_margin = 0.15;
  DWAPlanner planner(config);
  // The 1.1 m road is narrower than two times the required 0.60 m clearance.
  RoadModel narrow_road{true, true, 0.55, 1.1, 0.55, 0.0};

  const auto result =
      planner.plan(Pose2D{}, Velocity{}, narrow_road, {}, MotionHistory{}, 0.1);

  EXPECT_FALSE(result.valid);
}

TEST(DWAPlanner, RejectsTrajectoriesBlockedAcrossTheRoad) {
  DWAPlanner::Config config;
  config.max_acceleration = 0.5;
  config.max_angular_acceleration = 10.0;
  config.robot_radius = 0.4;
  config.enable_stop_preference = true;
  config.stop_penalty = 100.0;
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

  // The BLE actuator cannot execute the raw 0.025 m/s dynamic-window sample.
  // The planner crosses that deadzone at the first executable speed instead.
  const bool has_executable_start = std::any_of(
      result.candidates.begin(), result.candidates.end(),
      [&config](const Trajectory& candidate) {
        return std::abs(candidate.command.linear - config.minimum_moving_velocity) < 1e-6;
      });
  EXPECT_TRUE(has_executable_start);
  for (const auto& candidate : result.candidates) {
    EXPECT_FALSE(candidate.command.linear > 1e-6 &&
                 candidate.command.linear < config.minimum_moving_velocity - 1e-6);
  }
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
  EXPECT_GE(result.best.command.linear, config.minimum_moving_velocity);
  EXPECT_GT(result.best.command.angular, 0.0);
}

TEST(DWAPlanner, RejectsCommandsOutsideBleGearEnvelope) {
  DWAPlanner::Config config;
  DWAPlanner planner(config);

  EXPECT_TRUE(planner.isHardwareFeasible({0.0, 0.0}));
  EXPECT_FALSE(planner.isHardwareFeasible({0.0, 0.1}));
  EXPECT_FALSE(planner.isHardwareFeasible({0.10, 0.0}));
  EXPECT_TRUE(planner.isHardwareFeasible({0.20, 0.30}));
  EXPECT_FALSE(planner.isHardwareFeasible({0.20, 0.31}));
  EXPECT_TRUE(planner.isHardwareFeasible({0.50, 0.60}));
  EXPECT_FALSE(planner.isHardwareFeasible({0.50, 0.61}));
  EXPECT_TRUE(planner.isHardwareFeasible({0.80, 0.90}));
  EXPECT_FALSE(planner.isHardwareFeasible({0.80, 0.91}));
}

TEST(DWAPlanner, KeepsStopCandidateAtMinimumExecutableSpeed) {
  DWAPlanner::Config config;
  config.max_acceleration = 0.5;
  DWAPlanner planner(config);
  MotionHistory history;
  history.previous_command = {config.minimum_moving_velocity, 0.0};
  history.valid = true;

  const auto result = planner.plan(Pose2D{}, history.previous_command, RoadModel{}, {},
                                   history, 0.1);

  const auto stop = std::find_if(
      result.candidates.begin(), result.candidates.end(), [](const Trajectory& candidate) {
        return std::abs(candidate.command.linear) <= 1e-6 &&
               std::abs(candidate.command.angular) <= 1e-6;
      });
  ASSERT_NE(stop, result.candidates.end());
  EXPECT_TRUE(stop->dynamic_feasible);
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
DWAPlanner::Config stopPreferenceTestConfig() {
  DWAPlanner::Config config;
  config.max_velocity = 0.8;
  config.prediction_time = 3.0;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.minimum_turning_radius = 0.1;
  config.weight_heading = 0.0;
  config.weight_velocity = 0.0;
  config.weight_road = 0.0;
  config.weight_smooth = 0.0;
  config.enable_stop_preference = true;
  config.stop_penalty = 6.0;
  config.minimum_moving_clearance = 0.10;
  config.minimum_clearance_gain = 0.05;
  return config;
}

TEST(DWAPlanner, PrefersOnlyExecutableTurnSaferThanStraightOverStop) {
  const DWAPlanner::Config config = stopPreferenceTestConfig();
  const std::vector<ObstaclePoint> obstacles{{1.5, 0.0}};
  DWAPlanner::Config baseline_config = config;
  baseline_config.enable_stop_preference = false;

  const auto baseline = DWAPlanner(baseline_config).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 0.1);
  DWAPlanner planner(config);
  const auto preferred = planner.plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 0.1);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(preferred.valid);
  EXPECT_NEAR(baseline.best.command.linear, 0.0, 1e-9);
  EXPECT_GT(preferred.best.command.linear, config.minimum_turning_velocity);
  EXPECT_GT(std::abs(preferred.best.command.angular), 1e-6);
  EXPECT_TRUE(planner.isHardwareFeasible(preferred.best.command));
  EXPECT_GE(preferred.best.minimum_clearance, config.minimum_moving_clearance);
  const auto straight = std::find_if(
      preferred.candidates.begin(), preferred.candidates.end(), [&](const Trajectory& candidate) {
        return std::abs(candidate.command.linear - preferred.best.command.linear) <= 1e-6 &&
               std::abs(candidate.command.angular) <= 1e-6;
      });
  ASSERT_NE(straight, preferred.candidates.end());
  EXPECT_GE(preferred.best.minimum_clearance - straight->minimum_clearance,
            config.minimum_clearance_gain);
}

TEST(DWAPlanner, StopPreferenceDoesNotForceMotionWithoutSaferTurn) {
  auto config = stopPreferenceTestConfig();
  config.max_angular_velocity = 0.0;
  config.stop_penalty = 100.0;

  const auto result = DWAPlanner(config).plan(
      Pose2D{}, Velocity{}, RoadModel{}, {{1.2, 0.0}}, MotionHistory{}, 0.1);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.best.command.linear, 0.0, 1e-9);
  const auto stop = std::find_if(result.candidates.begin(), result.candidates.end(),
                                 [](const Trajectory& candidate) {
                                   return std::abs(candidate.command.linear) <= 1e-9;
                                 });
  ASSERT_NE(stop, result.candidates.end());
  EXPECT_DOUBLE_EQ(stop->score, result.best.score);
}

TEST(DWAPlanner, SafeAvoidanceReleasesStationaryTrajectoryHold) {
  auto config = stopPreferenceTestConfig();
  config.score_switch_margin = 100.0;
  MotionHistory history;
  history.valid = true;

  const auto result = DWAPlanner(config).plan(
      Pose2D{}, Velocity{}, RoadModel{}, {{1.5, 0.0}}, history, 0.1);

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.best.command.linear, config.minimum_turning_velocity);
  const auto straight = std::find_if(
      result.candidates.begin(), result.candidates.end(), [&](const Trajectory& candidate) {
        return std::abs(candidate.command.linear - result.best.command.linear) <= 1e-6 &&
               std::abs(candidate.command.angular) <= 1e-6;
      });
  ASSERT_NE(straight, result.candidates.end());
  EXPECT_GE(result.best.minimum_clearance - straight->minimum_clearance,
            config.minimum_clearance_gain);
}

TEST(DWAPlanner, NearObstacleStartRewardsDetourBeforeTerminalClearanceRecovers) {
  auto config = stopPreferenceTestConfig();
  config.max_velocity = 0.30;
  config.weight_obstacle = 0.30;
  const std::vector<ObstaclePoint> obstacles{{1.10, 0.0}};
  auto without_preference = config;
  without_preference.enable_stop_preference = false;

  const auto baseline = DWAPlanner(without_preference).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 0.1);
  const auto preferred = DWAPlanner(config).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 0.1);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(preferred.valid);
  EXPECT_NEAR(baseline.best.command.linear, 0.0, 1e-6);
  EXPECT_GE(preferred.best.command.linear, config.minimum_moving_velocity);
  EXPECT_GT(std::abs(preferred.best.command.angular), 1e-6);
  EXPECT_LT(preferred.best.terminal_clearance, preferred.best.initial_clearance);
  EXPECT_GE(preferred.best.minimum_clearance, config.minimum_moving_clearance);
  const auto straight = std::find_if(
      preferred.candidates.begin(), preferred.candidates.end(), [&](const Trajectory& candidate) {
        return std::abs(candidate.command.linear - preferred.best.command.linear) <= 1e-6 &&
               std::abs(candidate.command.angular) <= 1e-6;
      });
  ASSERT_NE(straight, preferred.candidates.end());
  EXPECT_GE(preferred.best.minimum_clearance - straight->minimum_clearance,
            config.minimum_clearance_gain);
}

TEST(DWAPlanner, DoesNotPenalizeStopWhenTurnsDoNotImproveOnStraight) {
  const auto config = stopPreferenceTestConfig();
  const std::vector<ObstaclePoint> obstacles{{0.0, 0.8}};
  auto without_preference = config;
  without_preference.enable_stop_preference = false;

  const auto baseline = DWAPlanner(without_preference).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 0.1);
  const auto preferred = DWAPlanner(config).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 0.1);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(preferred.valid);
  EXPECT_DOUBLE_EQ(preferred.best.command.linear, baseline.best.command.linear);
  EXPECT_DOUBLE_EQ(preferred.best.score, baseline.best.score);
}

TEST(DWAPlanner, NarrowStartupWindowDoesNotForceAnIneffectiveTurn) {
  auto config = stopPreferenceTestConfig();
  config.max_velocity = 0.30;
  config.max_acceleration = 1.0;
  config.max_angular_acceleration = 1.5;
  const std::vector<ObstaclePoint> obstacles{{1.10, 0.0}};
  auto without_preference = config;
  without_preference.enable_stop_preference = false;

  const auto baseline = DWAPlanner(without_preference).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 1.0 / 17.0);
  const auto preferred = DWAPlanner(config).plan(
      Pose2D{}, Velocity{}, RoadModel{}, obstacles, MotionHistory{}, 1.0 / 17.0);

  ASSERT_TRUE(baseline.valid);
  ASSERT_TRUE(preferred.valid);
  EXPECT_NEAR(baseline.best.command.linear, 0.0, 1e-6);
  EXPECT_NEAR(preferred.best.command.linear, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(preferred.best.score, baseline.best.score);
}

TEST(DWAPlanner, RejectsDisjointMeasuredAndSentYawWindowsBeforeScoring) {
  DWAPlanner::Config config;
  config.max_angular_acceleration = 1.5;
  DWAPlanner planner(config);
  const Velocity measured{0.20, -0.279};
  const Velocity last_sent{0.0, 0.0};

  const auto result = planner.plan(Pose2D{}, measured, RoadModel{}, {},
                                   MotionHistory{}, 0.056, &last_sent);

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.command_window_empty);
  EXPECT_TRUE(result.candidates.empty());
}

TEST(DWAPlanner, IndoorCandidatesFitBothYawWindowsAndOutputLimit) {
  DWAPlanner::Config config;
  config.max_angular_acceleration = 1.5;
  config.minimum_turning_radius = 0.1;
  DWAPlanner planner(config);
  const Velocity measured{0.0, -0.07};
  const Velocity last_sent{0.0, 0.0};
  constexpr double dt = 0.1;

  const auto result = planner.plan(Pose2D{}, measured, RoadModel{}, {},
                                   MotionHistory{}, dt, &last_sent);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.command_window_empty);
  for (const auto& candidate : result.candidates) {
    EXPECT_GE(candidate.command.angular,
              measured.angular - config.max_angular_acceleration * dt - 1e-6);
    EXPECT_LE(candidate.command.angular,
              measured.angular + config.max_angular_acceleration * dt + 1e-6);
    EXPECT_LE(std::abs(candidate.command.angular - last_sent.angular),
              config.max_angular_acceleration * dt + 1e-6);
  }
}

namespace road_geometry = wheel_perception::core::road_geometry;

TEST(RoadGeometry, NormalDistanceIsNotMistakenForLateralIntercept) {
  for (double yaw : {-0.7, 0.0, 0.7}) {
    const double intercept =
        road_geometry::lateralInterceptFromSignedNormalDistance(-0.4, yaw);
    EXPECT_NEAR(intercept, -0.4 / std::cos(yaw), 1e-12);
    EXPECT_NEAR(road_geometry::signedDistanceToLine(0.0, 0.0, yaw, intercept),
                0.4, 1e-12);
  }
}

TEST(RoadGeometry, CameraTranslationAndRotationPreserveTheFittedLine) {
  constexpr double distance = 0.4;
  constexpr double tx = 0.45;
  constexpr double ty = -0.2;
  for (double camera_yaw : {-0.7, 0.0, 0.7}) {
    for (double mounting_yaw : {-0.2, 0.0, 0.2}) {
      const double camera_intercept =
          road_geometry::lateralInterceptFromSignedNormalDistance(-distance, camera_yaw);
      const double c = std::cos(mounting_yaw);
      const double s = std::sin(mounting_yaw);
      const double base_yaw = camera_yaw + mounting_yaw;
      const double base_intercept = road_geometry::lateralInterceptFromPointDirection(
          -s * camera_intercept + tx, c * camera_intercept + ty,
          std::cos(base_yaw), std::sin(base_yaw));
      EXPECT_NEAR(road_geometry::signedDistanceToLine(0.0, 0.0, base_yaw, base_intercept),
                  distance + std::sin(base_yaw) * tx - std::cos(base_yaw) * ty, 1e-12);
      for (double along : {0.0, 1.0, 3.0}) {
        const double camera_x = along * std::cos(camera_yaw);
        const double camera_y = camera_intercept + along * std::sin(camera_yaw);
        EXPECT_NEAR(road_geometry::signedDistanceToLine(
                        c * camera_x - s * camera_y + tx,
                        s * camera_x + c * camera_y + ty, base_yaw, base_intercept),
                    0.0, 1e-12);
      }
    }
  }
}

TEST(RoadGeometry, DegenerateLinesDoNotReturnInventedIntercepts) {
  EXPECT_FALSE(std::isfinite(road_geometry::lateralInterceptFromSignedNormalDistance(
      -0.4, std::acos(-1.0) / 2.0)));
  EXPECT_FALSE(std::isfinite(road_geometry::lateralInterceptFromPointDirection(
      1.0, 2.0, 0.0, 1.0)));
}

TEST(DWAPlanner, RejectsInsufficientPerpendicularRightMarginOnSlantedRoad) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  for (double yaw : {-0.7, 0.7}) {
    // A 0.65 m Y gap is only 0.497 m perpendicular, below the 0.60 m requirement.
    RoadModel road{true, false, 0.65, 0.0, 1.0, yaw};
    EXPECT_FALSE(DWAPlanner(config).plan({}, {}, road, {}, {}, 0.1).valid);
  }
}

TEST(DWAPlanner, RejectsInsufficientPerpendicularLeftMarginOnSlantedRoad) {
  DWAPlanner::Config config;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  for (double yaw : {-0.7, 0.7}) {
    RoadModel road{true, true, 1.5, 2.15, 1.0, yaw};
    EXPECT_FALSE(DWAPlanner(config).plan({}, {}, road, {}, {}, 0.1).valid);
  }
}

TEST(DWAPlanner, AcceptsExactPerpendicularRoadMarginAtRest) {
  DWAPlanner::Config config;
  config.max_velocity = 0.0;
  for (double yaw : {-0.7, 0.0, 0.7}) {
    const double lateral_distance =
        (config.robot_radius + config.road_margin) / std::cos(yaw);
    RoadModel road{true, false, lateral_distance, 0.0, 1.0, yaw};
    EXPECT_TRUE(DWAPlanner(config).plan({}, {}, road, {}, {}, 0.1).valid);
  }
}

TEST(DWAPlanner, CorrectedCameraLineRejectsThePreviouslyUnderMarginedTrajectory) {
  constexpr double yaw = 0.7;
  const double camera_intercept =
      road_geometry::lateralInterceptFromSignedNormalDistance(-0.4, yaw);
  const double base_intercept = road_geometry::lateralInterceptFromPointDirection(
      0.45, camera_intercept - 0.2, std::cos(yaw), std::sin(yaw));
  RoadModel road{true, false, -base_intercept, 0.0, 0.8, yaw};
  MotionHistory history;
  history.valid = true;
  history.previous_command = {0.17, 0.0};
  const auto result = DWAPlanner(DWAPlanner::Config{}).plan(
      {}, history.previous_command, road, {}, history, 0.1);
  const auto straight = std::find_if(
      result.candidates.begin(), result.candidates.end(), [](const Trajectory& candidate) {
        return std::abs(candidate.command.linear - 0.17) < 1e-9 &&
               std::abs(candidate.command.angular) < 1e-9;
      });
  ASSERT_NE(straight, result.candidates.end());
  EXPECT_FALSE(straight->inside_road);
}

TEST(DWAPlanner, AllSlantedRoadCandidatesUsePerpendicularFootprintClearance) {
  DWAPlanner::Config config;
  config.max_velocity = 0.6;
  config.max_acceleration = 10.0;
  config.max_angular_acceleration = 10.0;
  config.minimum_turning_radius = 0.1;
  const double required = config.robot_radius + config.road_margin;
  for (double yaw : {-0.7, 0.7}) {
    RoadModel road{true, true, 1.6, 3.2, 1.0, yaw};
    const auto result = DWAPlanner(config).plan({}, {}, road, {}, {}, 0.1);
    ASSERT_TRUE(result.valid);
    ASSERT_FALSE(result.candidates.empty());
    const double slope = std::tan(yaw);
    const double normalizer = std::sqrt(1.0 + slope * slope);
    for (const auto& candidate : result.candidates) {
      bool inside = true;
      for (const auto& pose : candidate.poses) {
        const double right = (pose.y - slope * pose.x + road.right_distance) / normalizer;
        const double left =
            (road.width - road.right_distance + slope * pose.x - pose.y) / normalizer;
        inside = inside && right + 1e-6 >= required && left + 1e-6 >= required;
      }
      EXPECT_EQ(candidate.inside_road, inside);
    }
  }
}

TEST(DWAPlanner, RejectsNonFiniteOrVerticalRoadGeometry) {
  RoadModel road{true, false, 1.0, 0.0, 1.0, std::acos(-1.0) / 2.0};
  DWAPlanner planner(DWAPlanner::Config{});
  EXPECT_FALSE(planner.plan({}, {}, road, {}, {}, 0.1).valid);
  road.yaw_error = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(planner.plan({}, {}, road, {}, {}, 0.1).valid);
}

TEST(DWAPlanner, FinalLqrCommandCannotCrossRightBoundary) {
  DWAPlanner::Config config;
  config.robot_radius = 0.45;
  config.road_margin = 0.15;
  DWAPlanner planner(config);
  RoadModel road{true, false, 1.0, 0.0, 1.0, 0.0};

  Trajectory checked;
  EXPECT_TRUE(planner.assessCommandSafety({0.6, 0.0}, road, {}, &checked));
  EXPECT_TRUE(checked.inside_road);
  // The pre-LQR straight path is safe, but a rightward yaw correction
  // would put the full wheelchair footprint across the right boundary.
  EXPECT_FALSE(planner.assessCommandSafety({0.6, -0.3}, road, {}, &checked));
  EXPECT_FALSE(checked.inside_road);
}

TEST(DWAPlanner, FinalCommandCannotHitObstacle) {
  DWAPlanner planner(DWAPlanner::Config{});
  RoadModel road{true, false, 1.5, 0.0, 1.0, 0.0};
  Trajectory checked;
  EXPECT_FALSE(planner.assessCommandSafety(
      {0.6, 0.0}, road, {{1.0, 0.0}}, &checked));
  EXPECT_FALSE(checked.collision_free);
}

}  // namespace
