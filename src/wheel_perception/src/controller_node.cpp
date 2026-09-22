#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
#include <Eigen/Core>
#include <Eigen/Geometry>

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
    if (dataset_mode_) {
      sub_dataset_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/zed/odom", rclcpp::SensorDataQoS(),
          std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1));
    } else {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10,
          std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1));
    }
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "zed/point_cloud", rclcpp::SensorDataQoS(),
        std::bind(&ControllerNode::cloudCallback, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_dwa_cmd_ = create_publisher<geometry_msgs::msg::Twist>("dwa/planner_cmd", 10);
    pub_local_trajectory_ = create_publisher<nav_msgs::msg::Path>("dwa/local_trajectory", 10);
    pub_best_path_ = create_publisher<nav_msgs::msg::Path>("dwa/best_path", 10);
    pub_best_path_marker_ =
        create_publisher<visualization_msgs::msg::Marker>("dwa/best_path_marker", 10);
    pub_candidate_paths_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("dwa/candidate_paths", 10);
    pub_dwa_obstacle_cloud_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("dwa/obstacle_cloud", 10);

    last_perception_time_ = now();
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControllerNode::watchdogCallback, this));

    RCLCPP_INFO(get_logger(),
                "Hierarchical controller ready: ObstacleFusion -> DWA -> LQR -> safety -> cmd_vel");
  }

 private:
  void declareParameters() {
    // Shared with FusionNode. It selects only the source of velocity feedback;
    // all DWA/LQR/safety parameters remain common to both operating modes.
    declare_parameter("zed.use_dataset_mode", true);
    declare_parameter("zed.odometry.base_frame", "base_link");
    declare_parameter("zed.odometry.extrinsic.translation_x", 0.0);
    declare_parameter("zed.odometry.extrinsic.translation_y", 0.0);
    declare_parameter("zed.odometry.extrinsic.translation_z", 0.0);
    declare_parameter("zed.odometry.extrinsic.roll", 0.0);
    declare_parameter("zed.odometry.extrinsic.pitch", 0.0);
    declare_parameter("zed.odometry.extrinsic.yaw", 0.0);
    declare_parameter("lqr.q_pos", 10.0);
    declare_parameter("lqr.q_ang", 10.0);
    declare_parameter("lqr.q_integral", 0.0);
    declare_parameter("lqr.integral_limit", 1.5);
    declare_parameter("lqr.k_w", 10.0);
    declare_parameter("lqr.model_v", 0.5);
    declare_parameter("lqr.aim_dist", 0.65);
    declare_parameter("lqr.lookahead_time", 0.6);
    declare_parameter("lqr.max_dwa_tracking_correction", 0.15);
    declare_parameter("lqr.max_cruise_angular_velocity", 0.30);
    declare_parameter("dynamic_aim.enabled", true);
    declare_parameter("logic.base_vel", 1.0);
    declare_parameter("logic.stop_dist", 1.0);
    declare_parameter("logic.narrow_road_width", 3.0);
    declare_parameter("logic.pass_clearance", 0.6);

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
    declare_parameter("dwa.weight_smooth", 0.10);
    declare_parameter("dwa.smooth.weight_delta_v", 1.5);
    declare_parameter("dwa.smooth.weight_delta_w", 2.0);
    declare_parameter("dwa.smooth.weight_acceleration", 0.8);
    declare_parameter("dwa.smooth.weight_angular_acceleration", 1.0);
    declare_parameter("dwa.smooth.weight_jerk", 1.2);
    declare_parameter("dwa.smooth.weight_angular_jerk", 1.5);
    declare_parameter("dwa.smooth.max_jerk", 1.0);
    declare_parameter("dwa.smooth.max_angular_jerk", 3.0);
    declare_parameter("dwa.minimum_turning_velocity", 0.001);
    declare_parameter("dwa.minimum_turning_radius", 0.6);
    declare_parameter("dwa.cruise_velocity", 0.6);
    declare_parameter("dwa.activation.max_x", 2.5);
    declare_parameter("dwa.activation.min_y", -0.9);
    declare_parameter("dwa.activation.max_y", 0.9);
    declare_parameter("dwa.activation.minimum_points", 1);
    declare_parameter("dwa.activation.clear_frames", 5);
    declare_parameter("dwa.trajectory_hold.enabled", true);
    declare_parameter("dwa.trajectory_hold.score_switch_margin", 0.10);
    declare_parameter("dwa.trajectory_hold.relative_switch_margin", 0.05);
    declare_parameter("dwa.trajectory_hold.minimum_clearance", 0.20);
    declare_parameter("dwa.trajectory_hold.clearance_switch_margin", 0.30);
    declare_parameter("dwa.obstacle_distance_threshold", 2.5);
    declare_parameter("dwa.robot_radius", 0.45);
    declare_parameter("dwa.road_margin", 0.15);
    declare_parameter("dwa.hardware_constraints.enabled", true);
    declare_parameter("dwa.hardware_constraints.minimum_moving_velocity", 0.15);
    declare_parameter("dwa.hardware_constraints.gear_0001_threshold", 0.35);
    declare_parameter("dwa.hardware_constraints.gear_0003_threshold", 0.70);
    declare_parameter("dwa.hardware_constraints.gear_0001_max_angular", 0.30);
    declare_parameter("dwa.hardware_constraints.gear_0003_max_angular", 0.60);
    declare_parameter("dwa.hardware_constraints.gear_0005_max_angular", 0.90);
    declare_parameter("dwa.max_obstacle_points", 2500);
    // DWA is a 2-D base planner. Only project points inside the wheelchair's
    // collision-height band and local planning rectangle; canopy/high points
    // remain visible in /zed/point_cloud but must not bend the base path.
    declare_parameter("dwa.obstacle_roi.min_x", 0.3);
    declare_parameter("dwa.obstacle_roi.max_x", 2.5);
    declare_parameter("dwa.obstacle_roi.min_y", -2.0);
    declare_parameter("dwa.obstacle_roi.max_y", 2.0);
    declare_parameter("dwa.obstacle_roi.min_z", -0.2);
    declare_parameter("dwa.obstacle_roi.max_z", 1.0);
    declare_parameter("dwa.visualization.max_candidates", 80);

    declare_parameter("safety.emergency_stop_distance", 0.8);
    declare_parameter("safety.perception_timeout", 0.5);
    declare_parameter("safety.stop_on_no_path", true);
    declare_parameter("velocity_feedback.timeout", 0.3);
    declare_parameter("velocity_feedback.stop_on_timeout", false);
    declare_parameter("velocity_feedback.accept_zero_twist", true);
  }

  void configureControllers() {
    dataset_mode_ = get_parameter("zed.use_dataset_mode").as_bool();
    base_frame_id_ = get_parameter("zed.odometry.base_frame").as_string();
    camera_translation_in_base_ = Eigen::Vector3d(
        get_parameter("zed.odometry.extrinsic.translation_x").as_double(),
        get_parameter("zed.odometry.extrinsic.translation_y").as_double(),
        get_parameter("zed.odometry.extrinsic.translation_z").as_double());
    const double camera_roll =
        get_parameter("zed.odometry.extrinsic.roll").as_double();
    const double camera_pitch =
        get_parameter("zed.odometry.extrinsic.pitch").as_double();
    const double camera_yaw =
        get_parameter("zed.odometry.extrinsic.yaw").as_double();
    camera_rotation_in_base_ =
        (Eigen::AngleAxisd(camera_yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(camera_pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(camera_roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

    LqrController::Config lqr_config;
    lqr_config.q_pos = get_parameter("lqr.q_pos").as_double();
    lqr_config.q_ang = get_parameter("lqr.q_ang").as_double();
    lqr_config.q_integral = get_parameter("lqr.q_integral").as_double();
    lqr_config.integral_limit = get_parameter("lqr.integral_limit").as_double();
    lqr_config.k_w = get_parameter("lqr.k_w").as_double();
    lqr_config.model_v = get_parameter("lqr.model_v").as_double();
    lqr_ = std::make_unique<LqrController>(lqr_config);
    max_dwa_tracking_correction_ = std::max(
        0.0, get_parameter("lqr.max_dwa_tracking_correction").as_double());
    max_cruise_angular_velocity_ = std::max(
        0.0, get_parameter("lqr.max_cruise_angular_velocity").as_double());

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
    dwa_config.weight_delta_velocity =
        get_parameter("dwa.smooth.weight_delta_v").as_double();
    dwa_config.weight_delta_angular =
        get_parameter("dwa.smooth.weight_delta_w").as_double();
    dwa_config.weight_acceleration =
        get_parameter("dwa.smooth.weight_acceleration").as_double();
    dwa_config.weight_angular_acceleration =
        get_parameter("dwa.smooth.weight_angular_acceleration").as_double();
    dwa_config.weight_jerk = get_parameter("dwa.smooth.weight_jerk").as_double();
    dwa_config.weight_angular_jerk =
        get_parameter("dwa.smooth.weight_angular_jerk").as_double();
    dwa_config.max_jerk = get_parameter("dwa.smooth.max_jerk").as_double();
    dwa_config.max_angular_jerk =
        get_parameter("dwa.smooth.max_angular_jerk").as_double();
    dwa_config.minimum_turning_velocity =
        get_parameter("dwa.minimum_turning_velocity").as_double();
    dwa_config.minimum_turning_radius =
        get_parameter("dwa.minimum_turning_radius").as_double();
    dwa_config.enable_trajectory_hold =
        get_parameter("dwa.trajectory_hold.enabled").as_bool();
    dwa_config.score_switch_margin =
        get_parameter("dwa.trajectory_hold.score_switch_margin").as_double();
    dwa_config.relative_switch_margin =
        get_parameter("dwa.trajectory_hold.relative_switch_margin").as_double();
    dwa_config.hold_minimum_clearance =
        get_parameter("dwa.trajectory_hold.minimum_clearance").as_double();
    dwa_config.clearance_switch_margin =
        get_parameter("dwa.trajectory_hold.clearance_switch_margin").as_double();
    dwa_config.obstacle_distance_threshold =
        get_parameter("dwa.obstacle_distance_threshold").as_double();
    dwa_config.robot_radius = get_parameter("dwa.robot_radius").as_double();
    dwa_config.road_margin = get_parameter("dwa.road_margin").as_double();
    dwa_config.enable_hardware_constraints =
        get_parameter("dwa.hardware_constraints.enabled").as_bool();
    dwa_config.minimum_moving_velocity =
        get_parameter("dwa.hardware_constraints.minimum_moving_velocity").as_double();
    dwa_config.gear_0001_selection_threshold =
        get_parameter("dwa.hardware_constraints.gear_0001_threshold").as_double();
    dwa_config.gear_0003_selection_threshold =
        get_parameter("dwa.hardware_constraints.gear_0003_threshold").as_double();
    dwa_config.gear_0001_maximum_angular_velocity =
        get_parameter("dwa.hardware_constraints.gear_0001_max_angular").as_double();
    dwa_config.gear_0003_maximum_angular_velocity =
        get_parameter("dwa.hardware_constraints.gear_0003_max_angular").as_double();
    dwa_config.gear_0005_maximum_angular_velocity =
        get_parameter("dwa.hardware_constraints.gear_0005_max_angular").as_double();
    max_angular_velocity_ = dwa_config.max_angular_velocity;
    simulation_dt_ = dwa_config.simulation_time_step;
    max_angular_acceleration_ = dwa_config.max_angular_acceleration;
    max_linear_acceleration_ = dwa_config.max_acceleration;
    minimum_turning_velocity_ = dwa_config.minimum_turning_velocity;
    minimum_turning_radius_ = dwa_config.minimum_turning_radius;
    minimum_moving_velocity_ = dwa_config.enable_hardware_constraints
        ? dwa_config.minimum_moving_velocity : 0.0;
    prediction_time_ = dwa_config.prediction_time;
    cruise_velocity_ = std::clamp(
        get_parameter("dwa.cruise_velocity").as_double(),
        dwa_config.min_velocity, dwa_config.max_velocity);
    dwa_activation_max_x_ =
        get_parameter("dwa.activation.max_x").as_double();
    dwa_activation_min_y_ =
        get_parameter("dwa.activation.min_y").as_double();
    dwa_activation_max_y_ =
        get_parameter("dwa.activation.max_y").as_double();
    dwa_activation_minimum_points_ = static_cast<std::size_t>(std::max<int64_t>(
        1, get_parameter("dwa.activation.minimum_points").as_int()));
    dwa_activation_clear_frames_ = static_cast<std::size_t>(std::max<int64_t>(
        1, get_parameter("dwa.activation.clear_frames").as_int()));
    dwa_ = std::make_unique<dwa::DWAPlanner>(dwa_config);

    RCLCPP_INFO(get_logger(), "DWA velocity source: %s",
                dataset_mode_ ? "last controller output (dataset mode)"
                              : "WORLD-pose-difference /odom twist (real-wheelchair mode)");
    RCLCPP_INFO(
        get_logger(),
        "DWA camera-to-base extrinsic xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
        camera_translation_in_base_.x(), camera_translation_in_base_.y(),
        camera_translation_in_base_.z(), camera_roll, camera_pitch, camera_yaw);
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

    // Dataset odometry belongs to the historical drive, not to the commands
    // generated by this replay. Keep its pose for visualization/state input,
    // but never feed its velocity (or a velocity reconstructed at replay FPS)
    // into the new controller's dynamic window.
    if (dataset_mode_) {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      robot_state_ = next_state;
      return;
    }

    // In real-wheelchair mode FusionNode has already converted consecutive
    // ZED WORLD camera poses into base-link body velocity.  The controller
    // consumes that result directly; it never consumes raw SDK Pose.twist.
    const double measured_v = message->twist.twist.linear.x;
    const double measured_w = message->twist.twist.angular.z;
    const bool accept_zero_twist =
        get_parameter("velocity_feedback.accept_zero_twist").as_bool();
    const bool direct_twist_valid = std::isfinite(measured_v) &&
        std::isfinite(measured_w) &&
        (accept_zero_twist || std::abs(measured_v) > 1e-3 ||
         std::abs(measured_w) > 1e-3);
    const rclcpp::Time stamp(message->header.stamp);
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    if (direct_twist_valid) {
      measured_velocity_ = {measured_v, measured_w};
      has_velocity_feedback_ = true;
      last_velocity_feedback_time_ = now();
    } else {
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
          last_velocity_feedback_time_ = now();
        }
      }
    }
    last_odom_stamp_ = stamp;
    robot_state_ = next_state;
  }

  bool velocityFeedbackFreshLocked(const rclcpp::Time& reference_time) const {
    if (!has_velocity_feedback_ || last_velocity_feedback_time_.nanoseconds() == 0) {
      return false;
    }
    const double timeout = get_parameter("velocity_feedback.timeout").as_double();
    return timeout > 0.0 && reference_time >= last_velocity_feedback_time_ &&
           (reference_time - last_velocity_feedback_time_).seconds() <= timeout;
  }

  struct PlanningMotionSnapshot {
    dwa::Pose2D pose;
    dwa::Velocity velocity;
    bool feedback_fresh{false};
  };

  PlanningMotionSnapshot planningMotionSnapshot(
      const rclcpp::Time& reference_time) {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    PlanningMotionSnapshot snapshot;
    snapshot.pose = robot_state_;
    snapshot.feedback_fresh = !dataset_mode_ &&
        velocityFeedbackFreshLocked(reference_time);
    snapshot.velocity = dataset_mode_ || !snapshot.feedback_fresh
        ? last_output_velocity_ : measured_velocity_;
    return snapshot;
  }

  Eigen::Vector3d transformCameraPointToBase(
      double x, double y, double z) const {
    return camera_rotation_in_base_ * Eigen::Vector3d(x, y, z) +
           camera_translation_in_base_;
  }

  dwa::RoadModel makeBaseRoadModel(
      const wheel_msgs::msg::PerceptionOutput& message) const {
    dwa::RoadModel road;
    road.has_right_edge = message.has_road_edge;
    road.has_width = message.has_road_width;
    road.right_distance = message.right_distance;
    road.width = message.road_width;
    road.target_right_distance =
        get_parameter("dynamic_aim.enabled").as_bool() &&
                message.target_right_distance > 0.01
            ? message.target_right_distance
            : get_parameter("lqr.aim_dist").as_double();
    // FusionNode publishes road geometry in base_link after applying the same
    // camera extrinsic used by odometry and obstacle points.
    road.yaw_error = message.road_yaw_error;
    return road;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (message->width == 0 || message->height == 0) return;

    const std::size_t maximum = static_cast<std::size_t>(
        std::max<int64_t>(1, get_parameter("dwa.max_obstacle_points").as_int()));
    const std::size_t total = static_cast<std::size_t>(message->width) * message->height;

    const double min_x = get_parameter("dwa.obstacle_roi.min_x").as_double();
    const double max_x = get_parameter("dwa.obstacle_roi.max_x").as_double();
    const double min_y = get_parameter("dwa.obstacle_roi.min_y").as_double();
    const double max_y = get_parameter("dwa.obstacle_roi.max_y").as_double();
    const double min_z = get_parameter("dwa.obstacle_roi.min_z").as_double();
    const double max_z = get_parameter("dwa.obstacle_roi.max_z").as_double();

    // Filter before downsampling. Otherwise a dense tree canopy can consume
    // most of the sampling budget and hide relevant low obstacles.
    std::vector<std::array<float, 3>> eligible;
    eligible.reserve(std::min<std::size_t>(total, maximum * 4));

    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
    for (std::size_t index = 0; index < total; ++index, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
      const Eigen::Vector3d base_point = transformCameraPointToBase(*x, *y, *z);
      if (base_point.x() < min_x || base_point.x() > max_x ||
          base_point.y() < min_y || base_point.y() > max_y ||
          base_point.z() < min_z || base_point.z() > max_z) {
        continue;
      }
      eligible.push_back({static_cast<float>(base_point.x()),
                          static_cast<float>(base_point.y()),
                          static_cast<float>(base_point.z())});
    }

    std::vector<dwa::ObstaclePoint> points;
    points.reserve(std::min(eligible.size(), maximum));
    if (!eligible.empty()) {
      const double stride = std::max(1.0, static_cast<double>(eligible.size()) /
                                              static_cast<double>(maximum));
      for (double index = 0.0; static_cast<std::size_t>(index) < eligible.size() &&
                               points.size() < maximum;
           index += stride) {
        const auto& point = eligible[static_cast<std::size_t>(index)];
        points.push_back({point[0], point[1]});
      }
    }

    if (pub_dwa_obstacle_cloud_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = message->header;
      cloud.header.frame_id = base_frame_id_;
      cloud.height = 1;
      cloud.width = static_cast<std::uint32_t>(points.size());
      cloud.is_dense = true;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(points.size());
      sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
      for (const auto& point : points) {
        *out_x = static_cast<float>(point.x);
        *out_y = static_cast<float>(point.y);
        // Publish the 2-D footprint projection actually consumed by DWA.
        *out_z = 0.0F;
        ++out_x;
        ++out_y;
        ++out_z;
      }
      pub_dwa_obstacle_cloud_->publish(std::move(cloud));
    }

    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles_ = std::move(points);
  }

  void perceptionCallback(const wheel_msgs::msg::PerceptionOutput::SharedPtr message) {
    const rclcpp::Time callback_time = now();
    double control_dt = simulation_dt_;
    if (last_control_time_.nanoseconds() > 0 && callback_time > last_control_time_) {
      control_dt = std::clamp((callback_time - last_control_time_).seconds(), 0.01, 0.25);
    }
    last_control_time_ = callback_time;
    last_perception_time_ = callback_time;
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

    const dwa::RoadModel road = makeBaseRoadModel(*message);

    const auto motion = planningMotionSnapshot(callback_time);
    if (!dataset_mode_ && !motion.feedback_fresh) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Velocity feedback unavailable or stale; using last controller output");
      if (get_parameter("velocity_feedback.stop_on_timeout").as_bool()) {
        publishStop("velocity feedback timeout");
        return;
      }
    }
    const dwa::Velocity planning_velocity = motion.velocity;
    const bool use_dwa = updateDwaActivation(obstacles);
    dwa::DWAPlanner::Result result;
    if (use_dwa) {
      result = dwa_->plan(motion.pose, planning_velocity, road, obstacles,
                          motion_history_, control_dt);
    } else {
      result.valid = true;
      result.best = makeCruiseTrajectory(road, planning_velocity, control_dt);
    }
    publishVisualization(result, use_dwa);

    if (use_dwa && !result.valid &&
        get_parameter("safety.stop_on_no_path").as_bool()) {
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
    const double reference_w = result.best.command.angular;
    const double raw_tracked_w = lqr_->computeTracking(
        lateral_error, heading_error, reference_w, control_dt);
    // DWA limits the feedback correction around its checked reference command.
    // Cruise has its own steering limit because its reference angular velocity
    // is zero. Both modes still pass through the actuator/rate limits below.
    double tracked_w = use_dwa
        ? reference_w + std::clamp(
            raw_tracked_w - reference_w,
            -max_dwa_tracking_correction_, max_dwa_tracking_correction_)
        : std::clamp(raw_tracked_w,
                     -max_cruise_angular_velocity_, max_cruise_angular_velocity_);
    tracked_w = std::clamp(tracked_w, -max_angular_velocity_, max_angular_velocity_);
    const double max_delta_w = max_angular_acceleration_ * control_dt;
    tracked_w = std::clamp(tracked_w, last_tracked_angular_ - max_delta_w,
                           last_tracked_angular_ + max_delta_w);
    if (result.best.command.linear < minimum_turning_velocity_) {
      tracked_w = 0.0;
    } else if (minimum_turning_radius_ > 1e-6) {
      const double curvature_limited_w =
          std::abs(result.best.command.linear) / minimum_turning_radius_;
      tracked_w = std::clamp(tracked_w, -curvature_limited_w, curvature_limited_w);
    }
    // LQR is downstream of DWA and can otherwise push a valid planner command
    // outside the BLE wheelchair's executable envelope.
    const double hardware_limited_w =
        dwa_->maximumHardwareAngularVelocity(result.best.command.linear);
    tracked_w = std::clamp(tracked_w, -hardware_limited_w, hardware_limited_w);

    geometry_msgs::msg::Twist command;
    command.linear.x = result.best.command.linear;
    command.angular.z = tracked_w;
    pub_cmd_->publish(command);

    if (use_dwa) {
      const dwa::Velocity previous_dwa_command = motion_history_.valid
          ? motion_history_.previous_command
          : planning_velocity;
      motion_history_.previous_linear_acceleration =
          (result.best.command.linear - previous_dwa_command.linear) / control_dt;
      motion_history_.previous_angular_acceleration =
          (result.best.command.angular - previous_dwa_command.angular) / control_dt;
      motion_history_.previous_command = result.best.command;
      motion_history_.valid = true;
    } else {
      // A later DWA activation starts from the actual last controller output,
      // not from stale DWA state left over from a previous obstacle.
      motion_history_ = {};
    }
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      last_output_velocity_ = {result.best.command.linear, tracked_w};
    }
    last_tracked_angular_ = tracked_w;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "%s+LQR: v=%.2f w_ref=%.2f w_track=%.2f velocity_source=%s "
        "lat_err=%.2f heading_err=%.3f right=%.2f road_yaw=%.3f edge=%s "
        "clearance=%.2f score=%.2f obstacles=%zu",
        use_dwa ? "DWA" : "CRUISE", command.linear.x,
        result.best.command.angular, tracked_w,
        dataset_mode_ ? "controller_output(dataset)"
                      : (motion.feedback_fresh ? "zed_odom" : "controller_output(fallback)"),
        lateral_error, heading_error,
        road.right_distance, road.yaw_error, road.has_right_edge ? "true" : "false",
        result.best.minimum_clearance, result.best.score, obstacles.size());
  }

  bool updateDwaActivation(const std::vector<dwa::ObstaclePoint>& obstacles) {
    const std::size_t points_in_activation_area = static_cast<std::size_t>(std::count_if(
        obstacles.begin(), obstacles.end(), [this](const dwa::ObstaclePoint& point) {
          return point.x > 0.0 && point.x <= dwa_activation_max_x_ &&
                 point.y >= dwa_activation_min_y_ && point.y <= dwa_activation_max_y_;
        }));
    const bool obstacle_detected =
        points_in_activation_area >= dwa_activation_minimum_points_;
    const bool was_active = dwa_active_;

    if (obstacle_detected) {
      dwa_active_ = true;
      dwa_clear_frame_count_ = 0;
    } else if (dwa_active_) {
      ++dwa_clear_frame_count_;
      if (dwa_clear_frame_count_ >= dwa_activation_clear_frames_) {
        dwa_active_ = false;
        dwa_clear_frame_count_ = 0;
      }
    }

    if (dwa_active_ != was_active) {
      RCLCPP_INFO(get_logger(), "Local planner mode: %s (activation points=%zu)",
                  dwa_active_ ? "DWA obstacle avoidance" : "road cruise",
                  points_in_activation_area);
      motion_history_ = {};
      lqr_->reset();
    }
    return dwa_active_;
  }

  dwa::Trajectory makeCruiseTrajectory(const dwa::RoadModel& road,
                                       const dwa::Velocity& current_velocity,
                                       double control_dt) const {
    dwa::Trajectory trajectory;
    const double dt = std::clamp(control_dt, 0.01, 0.25);
    const double minimum_velocity = std::max(0.0,
        current_velocity.linear - max_linear_acceleration_ * dt);
    const double maximum_velocity = std::max(
        minimum_velocity, current_velocity.linear + max_linear_acceleration_ * dt);
    trajectory.command.linear =
        std::clamp(cruise_velocity_, minimum_velocity, maximum_velocity);
    if (trajectory.command.linear > 0.0 &&
        trajectory.command.linear < minimum_moving_velocity_) {
      // Cross the BLE forward deadzone atomically. Values below this boundary
      // are rejected by the bridge and would leave the wheelchair stationary.
      trajectory.command.linear = minimum_moving_velocity_;
    }
    trajectory.command.angular = 0.0;
    trajectory.collision_free = true;
    trajectory.inside_road = true;
    trajectory.dynamic_feasible = true;
    trajectory.minimum_clearance = dwa_activation_max_x_;
    trajectory.score = 0.0;

    const double target_offset = road.has_right_edge
        ? road.target_right_distance - road.right_distance
        : 0.0;
    const double road_yaw = road.has_right_edge
        ? std::clamp(road.yaw_error, -1.2, 1.2)
        : 0.0;
    const double road_slope = std::tan(road_yaw);
    for (double time = 0.0; time <= prediction_time_ + 1e-6;
         time += simulation_dt_) {
      const double x = trajectory.command.linear * time;
      trajectory.poses.push_back({x, target_offset + road_slope * x, road_yaw});
    }
    return trajectory;
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
    motion_history_ = {};
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      last_output_velocity_ = {};
    }
    last_tracked_angular_ = 0.0;
    lqr_->reset();
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "base_link";
    pub_local_trajectory_->publish(empty_path);
    pub_best_path_->publish(empty_path);
    visualization_msgs::msg::Marker clear_best_path;
    clear_best_path.header.stamp = now();
    clear_best_path.header.frame_id = "base_link";
    clear_best_path.ns = "selected_local_path";
    clear_best_path.id = 0;
    clear_best_path.action = visualization_msgs::msg::Marker::DELETE;
    pub_best_path_marker_->publish(clear_best_path);
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

  void publishVisualization(const dwa::DWAPlanner::Result& result, bool use_dwa) {
    if (result.valid) {
      const auto path = toPath(result.best);
      pub_local_trajectory_->publish(path);
      pub_best_path_->publish(path);

      visualization_msgs::msg::Marker selected_path;
      selected_path.header = path.header;
      selected_path.ns = "selected_local_path";
      selected_path.id = 0;
      selected_path.action = visualization_msgs::msg::Marker::ADD;
      selected_path.pose.orientation.w = 1.0;
      selected_path.color.a = 1.0F;
      if (use_dwa) {
        // Blue is reserved for the selected DWA avoidance path. Candidate
        // trajectories already use green/red for valid/invalid samples.
        selected_path.color.r = 0.10F;
        selected_path.color.g = 0.35F;
        selected_path.color.b = 1.00F;
      } else {
        // Green identifies the deterministic road-following cruise path.
        selected_path.color.r = 0.00F;
        selected_path.color.g = 1.00F;
        selected_path.color.b = 0.00F;
      }

      const bool selected_stop =
          use_dwa &&
          std::abs(result.best.command.linear) < minimum_turning_velocity_ &&
          std::abs(result.best.command.angular) < 1e-6;
      if (selected_stop) {
        // A valid DWA stop trajectory contains repeated poses at the origin,
        // so a LINE_STRIP is visually indistinguishable from no path. Render
        // it as a blue disc to make the deliberate stop decision explicit.
        selected_path.type = visualization_msgs::msg::Marker::SPHERE;
        selected_path.pose.position.z = 0.04;
        selected_path.scale.x = 0.18;
        selected_path.scale.y = 0.18;
        selected_path.scale.z = 0.08;
      } else {
        selected_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        selected_path.scale.x = 0.06;
        selected_path.points.reserve(result.best.poses.size());
        for (const auto& pose : result.best.poses) {
          geometry_msgs::msg::Point point;
          point.x = pose.x;
          point.y = pose.y;
          point.z = 0.03;
          selected_path.points.push_back(point);
        }
      }
      pub_best_path_marker_->publish(selected_path);
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
      const bool valid = candidate.collision_free && candidate.inside_road &&
                         candidate.dynamic_feasible;
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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_best_path_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_candidate_paths_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dwa_obstacle_cloud_;

  std::mutex obstacle_mutex_;
  std::mutex velocity_mutex_;
  std::vector<dwa::ObstaclePoint> obstacles_;
  dwa::Pose2D robot_state_;
  dwa::Velocity measured_velocity_;
  dwa::MotionHistory motion_history_;
  dwa::Velocity last_output_velocity_;
  bool has_velocity_feedback_{false};
  bool dataset_mode_{true};
  std::string base_frame_id_{"base_link"};
  Eigen::Vector3d camera_translation_in_base_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d camera_rotation_in_base_{Eigen::Matrix3d::Identity()};
  double max_angular_velocity_{1.0};
  double max_linear_acceleration_{0.5};
  double max_angular_acceleration_{1.5};
  double max_dwa_tracking_correction_{0.15};
  double max_cruise_angular_velocity_{0.30};
  double minimum_turning_velocity_{0.001};
  double minimum_turning_radius_{0.6};
  double minimum_moving_velocity_{0.15};
  double simulation_dt_{0.1};
  double prediction_time_{2.5};
  double cruise_velocity_{0.6};
  double dwa_activation_max_x_{2.5};
  double dwa_activation_min_y_{-0.9};
  double dwa_activation_max_y_{0.9};
  std::size_t dwa_activation_minimum_points_{1};
  std::size_t dwa_activation_clear_frames_{5};
  std::size_t dwa_clear_frame_count_{0};
  bool dwa_active_{false};
  double last_tracked_angular_{0.0};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_feedback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_perception_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  bool watchdog_stopped_{false};
};

}  // namespace wheel_control

RCLCPP_COMPONENTS_REGISTER_NODE(wheel_control::ControllerNode)
