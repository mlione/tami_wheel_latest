#pragma once

#include <cstddef>
#include <limits>
#include <vector>

namespace wheel_control::dwa {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Velocity {
  double linear{0.0};
  double angular{0.0};
};

struct ObstaclePoint {
  double x{0.0};
  double y{0.0};
};

struct RoadModel {
  bool has_right_edge{false};
  bool has_width{false};
  double right_distance{0.0};
  double width{0.0};
  double target_right_distance{0.0};
  double yaw_error{0.0};
};

struct Trajectory {
  Velocity command;
  std::vector<Pose2D> poses;
  double score{-std::numeric_limits<double>::infinity()};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  bool collision_free{false};
  bool inside_road{false};
};

class DWAPlanner {
 public:
  struct Config {
    double max_velocity{1.0};
    double min_velocity{0.0};
    double max_angular_velocity{1.0};
    double max_acceleration{0.5};
    double max_angular_acceleration{1.5};
    double prediction_time{2.5};
    double simulation_time_step{0.1};
    double velocity_resolution{0.1};
    double angular_resolution{0.1};
    double weight_heading{2.0};
    double weight_obstacle{3.0};
    double weight_velocity{1.0};
    double weight_road{3.0};
    double weight_smooth{2.0};
    double obstacle_distance_threshold{2.5};
    double robot_radius{0.45};
    double road_margin{0.15};
  };

  struct Result {
    bool valid{false};
    Trajectory best;
    std::vector<Trajectory> candidates;
  };

  explicit DWAPlanner(Config config);

  Result plan(const Pose2D& robot_state,
              const Velocity& current_velocity,
              const RoadModel& road,
              const std::vector<ObstaclePoint>& obstacles,
              double previous_angular_velocity) const;

 private:
  Trajectory simulate(double linear, double angular) const;
  void evaluate(Trajectory& trajectory,
                const RoadModel& road,
                const std::vector<ObstaclePoint>& obstacles,
                double previous_angular_velocity) const;
  std::vector<double> samples(double lower, double upper, double resolution) const;
  static double normalizeAngle(double angle);

  Config config_;
};

}  // namespace wheel_control::dwa
