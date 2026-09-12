#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wheel_msgs/msg/perception_output.hpp"
#include "wheel_perception/core/lqr_controller.hpp"
#include "wheel_perception/dwa_controller/DWAPlanner.hpp"

namespace wheel_control {
namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

}  // namespace

class ControllerNode : public rclcpp::Node {
 public:
  explicit ControllerNode(const rclcpp::NodeOptions& options)
      : Node("controller_node", options) {
    declareParameters();
    configureControllers();

    sub_perception_ = create_subscription<wheel_msgs::msg::PerceptionOutput>(
        "perception/output", 10,
        std::bind(&ControllerNode::perceptionCallback, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1));
    sub_dataset_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/zed/odom", rclcpp::SensorDataQoS(),
        std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1));
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "zed/point_cloud", rclcpp::SensorDataQoS(),
        std::bind(&ControllerNode::cloudCallback, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_dwa_cmd_ = create_publisher<geometry_msgs::msg::Twist>("dwa/planner_cmd", 10);
    pub_local_trajectory_ = create_publisher<nav_msgs::msg::Path>("dwa/local_trajectory", 10);
    pub_best_path_ = create_publisher<nav_msgs::msg::Path>("dwa/best_path", 10);
    pub_candidate_paths_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("dwa/candidate_paths", 10);

    last_perception_time_ = now();
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControllerNode::watchdogCallback, this));

    RCLCPP_INFO(get_logger(),
                "Hierarchical controller ready: ObstacleFusion -> DWA -> LQR -> safety -> cmd_vel");
  }

 private:
  void declareParameters() {
    declare_parameter("lqr.gain", 60.0);
    declare_parameter("lqr.q_pos", 10.0);
    declare_parameter("lqr.q_ang", 10.0);
    declare_parameter("lqr.q_integral", 0.0);
    declare_parameter("lqr.integral_limit", 1.5);
    declare_parameter("lqr.k_w", 10.0);
    declare_parameter("lqr.model_v", 0.5);
    declare_parameter("lqr.aim_dist", 0.65);
    declare_parameter("lqr.lookahead_time", 0.6);
    declare_parameter("dynamic_aim.enabled", true);
    declare_parameter("logic.base_vel", 1.0);
    declare_parameter("logic.stop_dist", 1.0);
    declare_parameter("logic.narrow_road_width", 3.0);
    declare_parameter("logic.pass_clearance", 0.6);
    declare_parameter("logic.recover_time", 2.0);

    declare_parameter("dwa.max_velocity", 1.0);
    declare_parameter("dwa.min_velocity", 0.0);
    declare_parameter("dwa.max_angular_velocity", 1.0);
    declare_parameter("dwa.max_acceleration", 0.5);
    declare_parameter("dwa.max_angular_acceleration", 1.5);
    declare_parameter("dwa.prediction_time", 2.5);
    declare_parameter("dwa.simulation_time_step", 0.1);
    declare_parameter("dwa.velocity_resolution", 0.1);
    declare_parameter("dwa.angular_resolution", 0.1);
    declare_parameter("dwa.weight_heading", 2.0);
    declare_parameter("dwa.weight_obstacle", 3.0);
    declare_parameter("dwa.weight_velocity", 1.0);
    declare_parameter("dwa.weight_road", 3.0);
    declare_parameter("dwa.weight_smooth", 2.0);
    declare_parameter("dwa.obstacle_distance_threshold", 2.5);
    declare_parameter("dwa.robot_radius", 0.45);
    declare_parameter("dwa.road_margin", 0.15);
    declare_parameter("dwa.max_obstacle_points", 2500);
    declare_parameter("dwa.visualization.max_candidates", 80);

    declare_parameter("safety.emergency_stop_distance", 0.8);
    declare_parameter("safety.perception_timeout", 0.5);
    declare_parameter("safety.stop_on_no_path", true);
  }

  void configureControllers() {
    LqrController::Config lqr_config;
    lqr_config.q_pos = get_parameter("lqr.q_pos").as_double();
    lqr_config.q_ang = get_parameter("lqr.q_ang").as_double();
    lqr_config.q_integral = get_parameter("lqr.q_integral").as_double();
    lqr_config.integral_limit = get_parameter("lqr.integral_limit").as_double();
    lqr_config.lqr_gain = get_parameter("lqr.gain").as_double();
    lqr_config.k_w = get_parameter("lqr.k_w").as_double();
    lqr_config.model_v = get_parameter("lqr.model_v").as_double();
    lqr_config.dt = get_parameter("dwa.simulation_time_step").as_double();
    lqr_ = std::make_unique<LqrController>(lqr_config);

    dwa::DWAPlanner::Config dwa_config;
    dwa_config.max_velocity = get_parameter("dwa.max_velocity").as_double();
    dwa_config.min_velocity = get_parameter("dwa.min_velocity").as_double();
    dwa_config.max_angular_velocity = get_parameter("dwa.max_angular_velocity").as_double();
    dwa_config.max_acceleration = get_parameter("dwa.max_acceleration").as_double();
    dwa_config.max_angular_acceleration =
        get_parameter("dwa.max_angular_acceleration").as_double();
    dwa_config.prediction_time = get_parameter("dwa.prediction_time").as_double();
    dwa_config.simulation_time_step = get_parameter("dwa.simulation_time_step").as_double();
    dwa_config.velocity_resolution = get_parameter("dwa.velocity_resolution").as_double();
    dwa_config.angular_resolution = get_parameter("dwa.angular_resolution").as_double();
    dwa_config.weight_heading = get_parameter("dwa.weight_heading").as_double();
    dwa_config.weight_obstacle = get_parameter("dwa.weight_obstacle").as_double();
    dwa_config.weight_velocity = get_parameter("dwa.weight_velocity").as_double();
    dwa_config.weight_road = get_parameter("dwa.weight_road").as_double();
    dwa_config.weight_smooth = get_parameter("dwa.weight_smooth").as_double();
    dwa_config.obstacle_distance_threshold =
        get_parameter("dwa.obstacle_distance_threshold").as_double();
    dwa_config.robot_radius = get_parameter("dwa.robot_radius").as_double();
    dwa_config.road_margin = get_parameter("dwa.road_margin").as_double();
    max_angular_velocity_ = dwa_config.max_angular_velocity;
    simulation_dt_ = dwa_config.simulation_time_step;
    max_angular_acceleration_ = dwa_config.max_angular_acceleration;
    dwa_ = std::make_unique<dwa::DWAPlanner>(dwa_config);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message) {
    dwa::Pose2D next_state;
    next_state.x = message->pose.pose.position.x;
    next_state.y = message->pose.pose.position.y;
    const auto& q = message->pose.pose.orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, next_state.yaw);

    const double measured_v = message->twist.twist.linear.x;
    const double measured_w = message->twist.twist.angular.z;
    if (std::isfinite(measured_v) && std::isfinite(measured_w) &&
        (std::abs(measured_v) > 1e-3 || std::abs(measured_w) > 1e-3)) {
      measured_velocity_ = {measured_v, measured_w};
      has_velocity_feedback_ = true;
    } else {
      const rclcpp::Time stamp(message->header.stamp);
      if (last_odom_stamp_.nanoseconds() > 0 && stamp > last_odom_stamp_) {
        const double dt = (stamp - last_odom_stamp_).seconds();
        if (dt > 0.001 && dt < 1.0) {
          const double dx = next_state.x - robot_state_.x;
          const double dy = next_state.y - robot_state_.y;
          const double linear =
              (dx * std::cos(robot_state_.yaw) + dy * std::sin(robot_state_.yaw)) / dt;
          const double angular = std::atan2(std::sin(next_state.yaw - robot_state_.yaw),
                                            std::cos(next_state.yaw - robot_state_.yaw)) /
                                 dt;
          measured_velocity_ = {linear, angular};
          has_velocity_feedback_ = true;
        }
      }
      last_odom_stamp_ = stamp;
    }
    robot_state_ = next_state;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    std::vector<dwa::ObstaclePoint> points;
    if (message->width == 0 || message->height == 0) return;

    const std::size_t maximum = static_cast<std::size_t>(
        std::max<int64_t>(1, get_parameter("dwa.max_obstacle_points").as_int()));
    const std::size_t total = static_cast<std::size_t>(message->width) * message->height;
    const std::size_t stride = std::max<std::size_t>(1, total / maximum);
    points.reserve(std::min(total, maximum));

    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    for (std::size_t index = 0; index < total; ++index, ++x, ++y) {
      if (index % stride != 0 || !std::isfinite(*x) || !std::isfinite(*y)) continue;
      points.push_back({*x, *y});
      if (points.size() >= maximum) break;
    }
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_ = std::move(points);
  }

  void perceptionCallback(const wheel_msgs::msg::PerceptionOutput::SharedPtr message) {
    last_perception_time_ = now();
    watchdog_stopped_ = false;
    const double emergency_distance =
        get_parameter("safety.emergency_stop_distance").as_double();
    if (message->min_distance > 0.01 && message->min_distance < emergency_distance) {
      publishStop("emergency obstacle distance");
      return;
    }

    std::vector<dwa::ObstaclePoint> obstacles;
    {
      std::lock_guard<std::mutex> lock(obstacle_mutex_);
      obstacles = obstacles_;
    }

    dwa::RoadModel road;
    road.has_right_edge = message->has_road_edge;
    road.has_width = message->has_road_width;
    road.right_distance = message->right_distance;
    road.width = message->road_width;
    road.target_right_distance =
        get_parameter("dynamic_aim.enabled").as_bool() &&
                message->target_right_distance > 0.01
            ? message->target_right_distance
            : get_parameter("lqr.aim_dist").as_double();
    // FusionNode already publishes this value in radians.
    road.yaw_error = message->road_yaw_error;

    const dwa::Velocity planning_velocity =
        has_velocity_feedback_ ? measured_velocity_ : last_planner_velocity_;
    const auto result = dwa_->plan(robot_state_, planning_velocity, road, obstacles,
                                   last_planner_velocity_.angular);
    publishVisualization(result);

    if (!result.valid && get_parameter("safety.stop_on_no_path").as_bool()) {
      publishStop("DWA found no collision-free road-valid path");
      return;
    }
    if (!result.valid) return;

    geometry_msgs::msg::Twist planner_command;
    planner_command.linear.x = result.best.command.linear;
    planner_command.angular.z = result.best.command.angular;
    pub_dwa_cmd_->publish(planner_command);

    const auto& reference = selectLookahead(result.best);
    const double lateral_error = -reference.y;
    const double heading_error = -reference.yaw;
    double tracked_w = lqr_->computeTracking(lateral_error, heading_error,
                                             result.best.command.angular);
    tracked_w = std::clamp(tracked_w, -max_angular_velocity_, max_angular_velocity_);
    const double max_delta_w = max_angular_acceleration_ * simulation_dt_;
    tracked_w = std::clamp(tracked_w, last_tracked_angular_ - max_delta_w,
                           last_tracked_angular_ + max_delta_w);

    geometry_msgs::msg::Twist command;
    command.linear.x = result.best.command.linear;
    command.angular.z = lqr_->toActuatorCommand(tracked_w);
    pub_cmd_->publish(command);
    last_planner_velocity_ = {result.best.command.linear, tracked_w};
    last_tracked_angular_ = tracked_w;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "DWA+LQR: v=%.2f w_ref=%.2f w_track=%.2f clearance=%.2f score=%.2f obstacles=%zu",
        command.linear.x, result.best.command.angular, tracked_w,
        result.best.minimum_clearance, result.best.score, obstacles.size());
  }

  const dwa::Pose2D& selectLookahead(const dwa::Trajectory& trajectory) const {
    const double lookahead_time = get_parameter("lqr.lookahead_time").as_double();
    const auto index = static_cast<std::size_t>(std::clamp(
        std::lround(lookahead_time / std::max(simulation_dt_, 0.01)), 0L,
        static_cast<long>(trajectory.poses.size() - 1)));
    return trajectory.poses[index];
  }

  void publishStop(const std::string& reason) {
    geometry_msgs::msg::Twist stop;
    pub_cmd_->publish(stop);
    pub_dwa_cmd_->publish(stop);
    last_planner_velocity_ = {};
    last_tracked_angular_ = 0.0;
    lqr_->reset();
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "base_link";
    pub_local_trajectory_->publish(empty_path);
    pub_best_path_->publish(empty_path);
    visualization_msgs::msg::MarkerArray clear_markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(clear);
    pub_candidate_paths_->publish(clear_markers);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Safety stop: %s", reason.c_str());
  }

  void watchdogCallback() {
    const double timeout = get_parameter("safety.perception_timeout").as_double();
    if (!watchdog_stopped_ && timeout > 0.0 &&
        (now() - last_perception_time_).seconds() > timeout) {
      watchdog_stopped_ = true;
      publishStop("perception timeout");
    }
  }

  nav_msgs::msg::Path toPath(const dwa::Trajectory& trajectory) const {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "base_link";
    path.poses.reserve(trajectory.poses.size());
    for (const auto& pose : trajectory.poses) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = path.header;
      stamped.pose.position.x = pose.x;
      stamped.pose.position.y = pose.y;
      stamped.pose.orientation = yawToQuaternion(pose.yaw);
      path.poses.push_back(std::move(stamped));
    }
    return path;
  }

  void publishVisualization(const dwa::DWAPlanner::Result& result) {
    if (result.valid) {
      const auto path = toPath(result.best);
      pub_local_trajectory_->publish(path);
      pub_best_path_->publish(path);
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    const auto max_candidates = static_cast<std::size_t>(std::max<int64_t>(
        1, get_parameter("dwa.visualization.max_candidates").as_int()));
    const std::size_t count = std::min(max_candidates, result.candidates.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto& candidate = result.candidates[index];
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = "base_link";
      marker.ns = "dwa_candidates";
      marker.id = static_cast<int>(index);
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.015;
      marker.color.a = 0.45F;
      const bool valid = candidate.collision_free && candidate.inside_road;
      marker.color.g = valid ? 0.8F : 0.0F;
      marker.color.r = valid ? 0.1F : 0.8F;
      for (const auto& pose : candidate.poses) {
        geometry_msgs::msg::Point point;
        point.x = pose.x;
        point.y = pose.y;
        marker.points.push_back(point);
      }
      markers.markers.push_back(std::move(marker));
    }
    pub_candidate_paths_->publish(markers);
  }

  std::unique_ptr<LqrController> lqr_;
  std::unique_ptr<dwa::DWAPlanner> dwa_;
  rclcpp::Subscription<wheel_msgs::msg::PerceptionOutput>::SharedPtr sub_perception_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dataset_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_dwa_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_trajectory_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_best_path_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_candidate_paths_;

  std::mutex obstacle_mutex_;
  std::vector<dwa::ObstaclePoint> obstacles_;
  dwa::Pose2D robot_state_;
  dwa::Velocity measured_velocity_;
  dwa::Velocity last_planner_velocity_;
  bool has_velocity_feedback_{false};
  double max_angular_velocity_{1.0};
  double max_angular_acceleration_{1.5};
  double simulation_dt_{0.1};
  double last_tracked_angular_{0.0};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_perception_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  bool watchdog_stopped_{false};
};

}  // namespace wheel_control

RCLCPP_COMPONENTS_REGISTER_NODE(wheel_control::ControllerNode)
