#include "wheel_perception/dwa_controller/DWAPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <map>
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
  config_.minimum_turning_velocity = std::max(config_.minimum_turning_velocity, 0.0);
  config_.minimum_turning_radius = std::max(config_.minimum_turning_radius, 0.0);
  config_.max_jerk = std::max(config_.max_jerk, kEpsilon);
  config_.max_angular_jerk = std::max(config_.max_angular_jerk, kEpsilon);
  config_.stop_penalty = std::max(config_.stop_penalty, 0.0);
  config_.minimum_moving_clearance = std::max(config_.minimum_moving_clearance, 0.0);
  config_.minimum_clearance_gain = std::max(config_.minimum_clearance_gain, 0.0);
  config_.minimum_moving_velocity = std::max(config_.minimum_moving_velocity, 0.0);
  config_.gear_0001_selection_threshold = std::max(
      config_.gear_0001_selection_threshold, config_.minimum_moving_velocity);
  config_.gear_0003_selection_threshold = std::max(
      config_.gear_0003_selection_threshold, config_.gear_0001_selection_threshold);
  config_.gear_0001_maximum_angular_velocity = std::max(
      config_.gear_0001_maximum_angular_velocity, 0.0);
  config_.gear_0003_maximum_angular_velocity = std::max(
      config_.gear_0003_maximum_angular_velocity, 0.0);
  config_.gear_0005_maximum_angular_velocity = std::max(
      config_.gear_0005_maximum_angular_velocity, 0.0);
}

DWAPlanner::Result DWAPlanner::plan(
    const Pose2D& robot_state, const Velocity& current_velocity,
    const RoadModel& road, const std::vector<ObstaclePoint>& obstacles,
    const MotionHistory& history, double control_dt) const {
  Result result;
  // Planning geometry is local to base_link, but odometry state is still a
  // required safety input: invalid localization must never produce a command.
  if (!std::isfinite(robot_state.x) || !std::isfinite(robot_state.y) ||
      !std::isfinite(robot_state.yaw) || !std::isfinite(current_velocity.linear) ||
      !std::isfinite(current_velocity.angular)) {
    return result;
  }

  // The reachable velocity window must follow the real controller period.
  // Clamp long scheduling gaps so that one delayed callback cannot cause a
  // large command jump.
  const double dt = std::clamp(control_dt, 0.01, 0.25);
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

  auto linear_samples = samples(min_v, max_v, config_.velocity_resolution);
  auto angular_samples = samples(min_w, max_w, config_.angular_resolution);
  // Sampling from a negative lower bound with a coarse resolution does not
  // necessarily land exactly on zero. Always retain a straight candidate,
  // including during a low-speed start.
  if (min_w <= 0.0 && max_w >= 0.0) {
    angular_samples.push_back(0.0);
  }
  if (history.valid) {
    linear_samples.push_back(std::clamp(history.previous_command.linear, min_v, max_v));
    angular_samples.push_back(std::clamp(history.previous_command.angular, min_w, max_w));
  }
  // The BLE joystick has a non-zero forward deadzone. The physical actuator
  // jumps from stop to its minimum executable speed, so expose exactly that
  // boundary as a candidate when acceleration away from rest is requested.
  if (config_.enable_hardware_constraints &&
      current_velocity.linear < config_.minimum_moving_velocity &&
      max_v > kEpsilon && config_.minimum_moving_velocity <= config_.max_velocity) {
    linear_samples.push_back(config_.minimum_moving_velocity);
  }
  if (config_.enable_hardware_constraints &&
      current_velocity.linear <= config_.minimum_moving_velocity + kEpsilon) {
    linear_samples.push_back(0.0);
  }
  const auto sort_and_unique = [](std::vector<double>& values) {
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(), [](double lhs, double rhs) {
      return std::abs(lhs - rhs) <= kEpsilon;
    }), values.end());
  };
  sort_and_unique(linear_samples);
  sort_and_unique(angular_samples);

  for (double v : linear_samples) {
    auto angular_samples_for_velocity = angular_samples;
    // Add both curvature-limit boundaries explicitly. This keeps low-speed
    // left/right sampling symmetric even when angular_resolution is coarser
    // than v / minimum_turning_radius.
    if (config_.minimum_turning_radius > kEpsilon &&
        v >= config_.minimum_turning_velocity) {
      const double radius_limited_w = std::abs(v) / config_.minimum_turning_radius;
      for (double boundary_w : {-radius_limited_w, radius_limited_w}) {
        if (boundary_w >= min_w - kEpsilon && boundary_w <= max_w + kEpsilon) {
          angular_samples_for_velocity.push_back(std::clamp(boundary_w, min_w, max_w));
        }
      }
      sort_and_unique(angular_samples_for_velocity);
    }

    for (double w : angular_samples_for_velocity) {
      // Low-speed rolling arcs are valid and are needed to escape a blocked
      // startup. Only reject effectively stationary turning so this mode still
      // has no in-place rotation state.
      if (v < config_.minimum_turning_velocity && std::abs(w) > kEpsilon) {
        continue;
      }
      if (config_.minimum_turning_radius > kEpsilon && std::abs(w) > kEpsilon &&
          std::abs(v / w) + kEpsilon < config_.minimum_turning_radius) {
        continue;
      }
      if (!isHardwareFeasible({v, w})) {
        continue;
      }
      Trajectory trajectory = simulate(v, w);
      evaluate(trajectory, road, obstacles, history, dt);
      if (trajectory.collision_free && trajectory.inside_road && trajectory.dynamic_feasible &&
          (!result.valid || trajectory.score > result.best.score)) {
        result.valid = true;
        result.best = trajectory;
      }
      result.candidates.push_back(std::move(trajectory));
    }
  }

  const bool stop_preference_applied = applyConditionalStopPenalty(result);

  // Candidate hysteresis: keep the previous safe command unless a new path is
  // meaningfully better or provides materially more obstacle clearance.
  if (result.valid && history.valid && config_.enable_trajectory_hold) {
    const Trajectory* incumbent = nullptr;
    double incumbent_distance = std::numeric_limits<double>::infinity();
    const double v_scale = std::max(config_.velocity_resolution, 0.01);
    const double w_scale = std::max(config_.angular_resolution, 0.01);
    for (const auto& candidate : result.candidates) {
      if (!candidate.collision_free || !candidate.inside_road ||
          !candidate.dynamic_feasible) {
        continue;
      }
      const double command_distance = std::hypot(
          (candidate.command.linear - history.previous_command.linear) / v_scale,
          (candidate.command.angular - history.previous_command.angular) / w_scale);
      if (command_distance < incumbent_distance) {
        incumbent_distance = command_distance;
        incumbent = &candidate;
      }
    }

    if (incumbent != nullptr &&
        incumbent->minimum_clearance >= config_.hold_minimum_clearance) {
      const double improvement = result.best.score - incumbent->score;
      const double required_improvement = config_.score_switch_margin +
          config_.relative_switch_margin * std::max(1.0, std::abs(incumbent->score));
      const double clearance_gain =
          result.best.minimum_clearance - incumbent->minimum_clearance;
      const bool leaving_deliberate_stop = stop_preference_applied &&
          incumbent->command.linear <= config_.minimum_turning_velocity + kEpsilon &&
          result.best.command.linear > config_.minimum_turning_velocity + kEpsilon;
      if (!leaving_deliberate_stop && improvement < required_improvement &&
          clearance_gain < config_.clearance_switch_margin) {
        result.best = *incumbent;
      }
    }
  }
  return result;
}

bool DWAPlanner::applyConditionalStopPenalty(Result& result) const {
  if (!result.valid || !config_.enable_stop_preference || config_.stop_penalty <= 0.0) {
    return false;
  }

  // Reuse the already evaluated straight trajectory at each sampled speed.
  // A colliding straight trajectory still provides a valid clearance baseline.
  // If zero yaw rate is outside the dynamic window, do not force motion.
  std::map<double, double> straight_clearance_by_speed;
  for (const auto& candidate : result.candidates) {
    if (std::abs(candidate.command.angular) <= kEpsilon) {
      straight_clearance_by_speed[candidate.command.linear] =
          candidate.minimum_clearance;
    }
  }

  const auto is_feasible = [](const Trajectory& candidate) {
    return candidate.collision_free && candidate.inside_road &&
           candidate.dynamic_feasible && std::isfinite(candidate.score);
  };
  const auto is_safe_avoidance = [&](const Trajectory& candidate) {
    const auto straight = straight_clearance_by_speed.find(candidate.command.linear);
    return is_feasible(candidate) &&
           candidate.command.linear > config_.minimum_turning_velocity + kEpsilon &&
           std::abs(candidate.command.angular) > kEpsilon &&
           candidate.minimum_clearance >= config_.minimum_moving_clearance &&
           straight != straight_clearance_by_speed.end() &&
           candidate.minimum_clearance - straight->second >=
               config_.minimum_clearance_gain + kEpsilon;
  };
  const bool has_safe_avoidance = std::any_of(
      result.candidates.begin(), result.candidates.end(), is_safe_avoidance);
  if (!has_safe_avoidance) {
    return false;
  }

  // Penalize stopping and other non-improving motion equally, so a slow
  // approach cannot win merely because the stationary option was penalized.
  // Hard collision, road, acceleration and BLE constraints remain unchanged.
  result.valid = false;
  for (auto& candidate : result.candidates) {
    if (!is_feasible(candidate)) continue;
    if (!is_safe_avoidance(candidate)) {
      candidate.score -= config_.stop_penalty;
    }
    if (!result.valid || candidate.score > result.best.score) {
      result.valid = true;
      result.best = candidate;
    }
  }
  return is_safe_avoidance(result.best);
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
                          const MotionHistory& history, double control_dt) const {
  trajectory.collision_free = true;
  trajectory.inside_road = true;
  trajectory.dynamic_feasible = true;
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
  // Road edges constrain the complete wheelchair footprint, not only the
  // base_link centre. Keep the circular footprint plus the configured extra
  // safety margin inside both detected boundaries.
  const double road_boundary_clearance = config_.robot_radius + config_.road_margin;

  for (std::size_t pose_index = 0; pose_index < trajectory.poses.size(); ++pose_index) {
    const auto& pose = trajectory.poses[pose_index];
    for (const auto& obstacle : obstacles) {
      if (obstacle.x < -config_.robot_radius ||
          obstacle.x > config_.obstacle_distance_threshold) {
        continue;
      }
      const double clearance = std::hypot(pose.x - obstacle.x, pose.y - obstacle.y) -
                               config_.robot_radius;
      trajectory.minimum_clearance = std::min(trajectory.minimum_clearance, clearance);
      if (pose_index == 0) {
        trajectory.initial_clearance = std::min(trajectory.initial_clearance, clearance);
      }
      if (pose_index + 1 == trajectory.poses.size()) {
        trajectory.terminal_clearance = std::min(trajectory.terminal_clearance, clearance);
      }
      if (clearance <= 0.0) trajectory.collision_free = false;
    }

    if (road.has_right_edge) {
      const double center_y = target_offset + road_slope * pose.x;
      road_cost += std::abs(pose.y - center_y);
      const double right_y =
          right_at_origin + road_slope * pose.x + road_boundary_clearance;
      if (pose.y < right_y) {
        trajectory.inside_road = false;
        road_cost += kInvalidRoadPenalty;
      }
      if (road.has_width) {
        const double left_y =
            left_at_origin + road_slope * pose.x - road_boundary_clearance;
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
  if (!std::isfinite(trajectory.initial_clearance)) {
    trajectory.initial_clearance = config_.obstacle_distance_threshold;
  }
  if (!std::isfinite(trajectory.terminal_clearance)) {
    trajectory.terminal_clearance = config_.obstacle_distance_threshold;
  }
  road_cost /= std::max<std::size_t>(1, trajectory.poses.size());

  const auto& endpoint = trajectory.poses.back();
  const double heading_cost = std::cos(normalizeAngle(endpoint.yaw - road.yaw_error));
  const double obstacle_cost = 1.0 / std::max(trajectory.minimum_clearance, 0.01);
  const double velocity_cost = config_.max_velocity - trajectory.command.linear;
  double smooth_cost = 0.0;
  if (history.valid) {
    const double dt = std::max(control_dt, 0.01);
    const double delta_v = trajectory.command.linear - history.previous_command.linear;
    const double delta_w = trajectory.command.angular - history.previous_command.angular;
    const double linear_acceleration = delta_v / dt;
    const double angular_acceleration = delta_w / dt;
    const double linear_jerk =
        (linear_acceleration - history.previous_linear_acceleration) / dt;
    const double angular_jerk =
        (angular_acceleration - history.previous_angular_acceleration) / dt;

    const bool starts_across_actuator_deadzone = config_.enable_hardware_constraints &&
        std::abs(history.previous_command.linear) <= kEpsilon &&
        std::abs(trajectory.command.linear - config_.minimum_moving_velocity) <= kEpsilon;
    const bool stops_across_actuator_deadzone = config_.enable_hardware_constraints &&
        history.previous_command.linear <= config_.minimum_moving_velocity + kEpsilon &&
        std::abs(trajectory.command.linear) <= kEpsilon;
    trajectory.dynamic_feasible =
        (std::abs(linear_acceleration) <= config_.max_acceleration + kEpsilon ||
         starts_across_actuator_deadzone || stops_across_actuator_deadzone) &&
        std::abs(angular_acceleration) <= config_.max_angular_acceleration + kEpsilon;

    // Normalize each term so the weights remain meaningful when the callback
    // rate or actuator limits change. Jerk is intentionally a soft cost: an
    // emergency stop must never be blocked by a comfort constraint.
    smooth_cost =
        config_.weight_delta_velocity *
            std::abs(delta_v) / std::max(config_.max_acceleration * dt, 0.01) +
        config_.weight_delta_angular *
            std::abs(delta_w) / std::max(config_.max_angular_acceleration * dt, 0.01) +
        config_.weight_acceleration *
            std::abs(linear_acceleration) / std::max(config_.max_acceleration, 0.01) +
        config_.weight_angular_acceleration *
            std::abs(angular_acceleration) /
                std::max(config_.max_angular_acceleration, 0.01) +
        config_.weight_jerk * std::abs(linear_jerk) / config_.max_jerk +
        config_.weight_angular_jerk *
            std::abs(angular_jerk) / config_.max_angular_jerk;
  }

  trajectory.score = config_.weight_heading * heading_cost -
                     config_.weight_obstacle * obstacle_cost -
                     config_.weight_velocity * velocity_cost -
                     config_.weight_road * road_cost -
                     config_.weight_smooth * smooth_cost;
  if (!trajectory.collision_free || !trajectory.inside_road ||
      !trajectory.dynamic_feasible) {
    trajectory.score = -std::numeric_limits<double>::infinity();
  }
}

bool DWAPlanner::isHardwareFeasible(const Velocity& command) const {
  if (!config_.enable_hardware_constraints) return true;
  if (!std::isfinite(command.linear) || !std::isfinite(command.angular)) return false;
  if (std::abs(command.linear) <= kEpsilon) {
    return std::abs(command.angular) <= kEpsilon;
  }
  if (command.linear < config_.minimum_moving_velocity - kEpsilon) return false;
  return std::abs(command.angular) <=
      maximumHardwareAngularVelocity(command.linear) + kEpsilon;
}

double DWAPlanner::maximumHardwareAngularVelocity(double linear_velocity) const {
  if (!config_.enable_hardware_constraints || linear_velocity <= kEpsilon) {
    return config_.enable_hardware_constraints ? 0.0 : config_.max_angular_velocity;
  }
  double limit = config_.gear_0005_maximum_angular_velocity;
  if (linear_velocity < config_.gear_0001_selection_threshold) {
    limit = config_.gear_0001_maximum_angular_velocity;
  } else if (linear_velocity < config_.gear_0003_selection_threshold) {
    limit = config_.gear_0003_maximum_angular_velocity;
  }
  return std::min(limit, config_.max_angular_velocity);
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
