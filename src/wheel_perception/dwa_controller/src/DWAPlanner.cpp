#include "wheel_perception/dwa_controller/DWAPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace wheel_control::dwa {
namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kInvalidRoadPenalty = 1e3;
}  // namespace

DWAPlanner::DWAPlanner(Config config) : config_(std::move(config)) {
  config_.simulation_time_step = std::max(config_.simulation_time_step, 0.01);
  config_.prediction_time = std::max(config_.prediction_time, config_.simulation_time_step);
  config_.velocity_resolution = std::max(config_.velocity_resolution, 0.01);
  config_.angular_resolution = std::max(config_.angular_resolution, 0.01);
  config_.robot_radius = std::max(config_.robot_radius, 0.0);
}

DWAPlanner::Result DWAPlanner::plan(
    const Pose2D& robot_state, const Velocity& current_velocity,
    const RoadModel& road, const std::vector<ObstaclePoint>& obstacles,
    double previous_angular_velocity) const {
  Result result;
  // Planning geometry is local to base_link, but odometry state is still a
  // required safety input: invalid localization must never produce a command.
  if (!std::isfinite(robot_state.x) || !std::isfinite(robot_state.y) ||
      !std::isfinite(robot_state.yaw) || !std::isfinite(current_velocity.linear) ||
      !std::isfinite(current_velocity.angular)) {
    return result;
  }

  const double dt = config_.simulation_time_step;
  const double min_v = std::clamp(current_velocity.linear - config_.max_acceleration * dt,
                                  config_.min_velocity, config_.max_velocity);
  const double max_v = std::clamp(current_velocity.linear + config_.max_acceleration * dt,
                                  config_.min_velocity, config_.max_velocity);
  const double min_w = std::clamp(
      current_velocity.angular - config_.max_angular_acceleration * dt,
      -config_.max_angular_velocity, config_.max_angular_velocity);
  const double max_w = std::clamp(
      current_velocity.angular + config_.max_angular_acceleration * dt,
      -config_.max_angular_velocity, config_.max_angular_velocity);

  for (double v : samples(min_v, max_v, config_.velocity_resolution)) {
    for (double w : samples(min_w, max_w, config_.angular_resolution)) {
      Trajectory trajectory = simulate(v, w);
      evaluate(trajectory, road, obstacles, previous_angular_velocity);
      if (trajectory.collision_free && trajectory.inside_road &&
          (!result.valid || trajectory.score > result.best.score)) {
        result.valid = true;
        result.best = trajectory;
      }
      result.candidates.push_back(std::move(trajectory));
    }
  }
  return result;
}

Trajectory DWAPlanner::simulate(double linear, double angular) const {
  Trajectory trajectory;
  trajectory.command = {linear, angular};
  trajectory.poses.push_back({});

  Pose2D pose;
  for (double time = 0.0; time < config_.prediction_time - kEpsilon;
       time += config_.simulation_time_step) {
    pose.x += linear * std::cos(pose.yaw) * config_.simulation_time_step;
    pose.y += linear * std::sin(pose.yaw) * config_.simulation_time_step;
    pose.yaw = normalizeAngle(pose.yaw + angular * config_.simulation_time_step);
    trajectory.poses.push_back(pose);
  }
  return trajectory;
}

void DWAPlanner::evaluate(Trajectory& trajectory, const RoadModel& road,
                          const std::vector<ObstaclePoint>& obstacles,
                          double previous_angular_velocity) const {
  trajectory.collision_free = true;
  trajectory.inside_road = true;
  double road_cost = 0.0;

  const double road_slope = std::tan(std::clamp(
      road.yaw_error, -1.2, 1.2));
  const double target_offset = road.has_right_edge
      ? road.target_right_distance - road.right_distance
      : 0.0;
  const double right_at_origin = road.has_right_edge ? -road.right_distance : 0.0;
  const double left_at_origin = road.has_width
      ? road.width - road.right_distance
      : std::numeric_limits<double>::infinity();

  for (const auto& pose : trajectory.poses) {
    for (const auto& obstacle : obstacles) {
      if (obstacle.x < -config_.robot_radius ||
          obstacle.x > config_.obstacle_distance_threshold) {
        continue;
      }
      const double clearance = std::hypot(pose.x - obstacle.x, pose.y - obstacle.y) -
                               config_.robot_radius;
      trajectory.minimum_clearance = std::min(trajectory.minimum_clearance, clearance);
      if (clearance <= 0.0) trajectory.collision_free = false;
    }

    if (road.has_right_edge) {
      const double center_y = target_offset + road_slope * pose.x;
      road_cost += std::abs(pose.y - center_y);
      const double right_y = right_at_origin + road_slope * pose.x + config_.road_margin;
      if (pose.y < right_y) {
        trajectory.inside_road = false;
        road_cost += kInvalidRoadPenalty;
      }
      if (road.has_width) {
        const double left_y = left_at_origin + road_slope * pose.x - config_.road_margin;
        if (pose.y > left_y) {
          trajectory.inside_road = false;
          road_cost += kInvalidRoadPenalty;
        }
      }
    }
  }

  if (!std::isfinite(trajectory.minimum_clearance)) {
    trajectory.minimum_clearance = config_.obstacle_distance_threshold;
  }
  road_cost /= std::max<std::size_t>(1, trajectory.poses.size());

  const auto& endpoint = trajectory.poses.back();
  const double heading_cost = std::cos(normalizeAngle(endpoint.yaw - road.yaw_error));
  const double obstacle_cost = 1.0 / std::max(trajectory.minimum_clearance, 0.01);
  const double velocity_cost = config_.max_velocity - trajectory.command.linear;
  const double smooth_cost = std::abs(trajectory.command.angular - previous_angular_velocity);

  trajectory.score = config_.weight_heading * heading_cost -
                     config_.weight_obstacle * obstacle_cost -
                     config_.weight_velocity * velocity_cost -
                     config_.weight_road * road_cost -
                     config_.weight_smooth * smooth_cost;
  if (!trajectory.collision_free || !trajectory.inside_road) {
    trajectory.score = -std::numeric_limits<double>::infinity();
  }
}

std::vector<double> DWAPlanner::samples(double lower, double upper,
                                        double resolution) const {
  if (lower > upper) std::swap(lower, upper);
  std::vector<double> values;
  for (double value = lower; value <= upper + kEpsilon; value += resolution) {
    values.push_back(std::min(value, upper));
  }
  if (values.empty() || std::abs(values.back() - upper) > kEpsilon) values.push_back(upper);
  return values;
}

double DWAPlanner::normalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace wheel_control::dwa
