#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
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
  enum class IndoorTestMode { Off, Visualize, Drive };

  explicit ControllerNode(const rclcpp::NodeOptions& options)
      : Node("controller_node", options) {
    declareParameters();
    configureControllers();

    // Planning stays in the default mutually exclusive group. Sensor updates
    // and the watchdog must remain schedulable while DWA is evaluating paths.
    odom_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions odom_options;
    odom_options.callback_group = odom_group_;
    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_group_;

    sub_perception_ = create_subscription<wheel_msgs::msg::PerceptionOutput>(
        "perception/output", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&ControllerNode::perceptionCallback, this, std::placeholders::_1));
    if (dataset_mode_) {
      sub_dataset_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/zed/odom", rclcpp::SensorDataQoS(),
          std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1),
          odom_options);
    } else {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/odom", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          std::bind(&ControllerNode::odomCallback, this, std::placeholders::_1),
          odom_options);
    }
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "zed/point_cloud", rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&ControllerNode::cloudCallback, this, std::placeholders::_1),
        cloud_options);

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_dwa_cmd_ = create_publisher<geometry_msgs::msg::Twist>("dwa/planner_cmd", 10);
    pub_indoor_test_cmd_ =
        create_publisher<geometry_msgs::msg::Twist>("dwa/indoor_test_cmd", 10);
    pub_local_trajectory_ = create_publisher<nav_msgs::msg::Path>("dwa/local_trajectory", 10);
    pub_best_path_ = create_publisher<nav_msgs::msg::Path>("dwa/best_path", 10);
    pub_best_path_marker_ =
        create_publisher<visualization_msgs::msg::Marker>("dwa/best_path_marker", 10);
    pub_candidate_paths_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("dwa/candidate_paths", 10);
    pub_dwa_obstacle_cloud_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("dwa/obstacle_cloud", 10);

    last_perception_receive_ns_.store(now().nanoseconds());
    watchdog_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControllerNode::watchdogCallback, this), watchdog_group_);

    RCLCPP_INFO(get_logger(), "Controller ready: indoor mode=%s%s",
                indoor_test_mode_ == IndoorTestMode::Off ? "off" :
                (indoor_test_mode_ == IndoorTestMode::Drive ? "drive" : "visualize"),
                indoor_test_mode_ == IndoorTestMode::Visualize
                    ? " (cmd_vel held at zero)" : "");
  }

 private:
  struct ObstacleFrame {
    std::int64_t stamp_ns{0};
    std::vector<dwa::ObstaclePoint> points;
  };
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
    declare_parameter("dwa.stop_preference.enabled", false);
    declare_parameter("dwa.stop_preference.penalty", 0.0);
    declare_parameter("dwa.stop_preference.minimum_moving_clearance", 0.10);
    declare_parameter("dwa.stop_preference.minimum_clearance_gain", 0.05);
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
    declare_parameter("dwa.indoor_test.mode", "off");
    declare_parameter("dwa.indoor_test.allow_motion", false);
    declare_parameter("dwa.indoor_test.max_velocity", 0.30);
    declare_parameter("dwa.indoor_test.preview_velocity", 0.30);

    declare_parameter("safety.emergency_stop_distance", 0.8);
    declare_parameter("safety.perception_timeout", 0.5);
    declare_parameter("safety.max_sensor_age", 0.25);
    declare_parameter("safety.stop_on_no_path", true);
    declare_parameter("velocity_feedback.timeout", 0.3);
    declare_parameter("velocity_feedback.stop_on_timeout", false);
    declare_parameter("velocity_feedback.accept_zero_twist", true);
  }

  void configureControllers() {
    dataset_mode_ = get_parameter("zed.use_dataset_mode").as_bool();
    max_sensor_age_ = get_parameter("safety.max_sensor_age").as_double();
    if (!std::isfinite(max_sensor_age_) || max_sensor_age_ <= 0.0) {
      throw std::invalid_argument("safety.max_sensor_age must be positive and finite");
    }
    const std::string indoor_mode = get_parameter("dwa.indoor_test.mode").as_string();
    if (indoor_mode == "visualize") {
      indoor_test_mode_ = IndoorTestMode::Visualize;
    } else if (indoor_mode == "drive") {
      indoor_test_mode_ = IndoorTestMode::Drive;
    } else if (indoor_mode != "off") {
      throw std::invalid_argument("dwa.indoor_test.mode must be off, visualize, or drive");
    }
    if (indoor_test_mode_ == IndoorTestMode::Drive &&
        (!get_parameter("dwa.indoor_test.allow_motion").as_bool() || dataset_mode_)) {
      throw std::invalid_argument(
          "indoor drive requires allow_motion=true and zed.use_dataset_mode=false");
    }
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
    if (indoor_test_mode_ == IndoorTestMode::Drive) {
      const double indoor_limit = get_parameter("dwa.indoor_test.max_velocity").as_double();
      if (!std::isfinite(indoor_limit) || indoor_limit < 0.0) {
        throw std::invalid_argument("dwa.indoor_test.max_velocity must be nonnegative");
      }
      dwa_config.max_velocity = std::min(dwa_config.max_velocity, indoor_limit);
    }
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
    dwa_config.enable_stop_preference =
        get_parameter("dwa.stop_preference.enabled").as_bool();
    dwa_config.stop_penalty =
        get_parameter("dwa.stop_preference.penalty").as_double();
    dwa_config.minimum_moving_clearance =
        get_parameter("dwa.stop_preference.minimum_moving_clearance").as_double();
    dwa_config.minimum_clearance_gain =
        get_parameter("dwa.stop_preference.minimum_clearance_gain").as_double();
    dwa_config.obstacle_distance_threshold =
        get_parameter("dwa.obstacle_distance_threshold").as_double();
    dwa_config.robot_radius = get_parameter("dwa.robot_radius").as_double();
    if (indoor_test_mode_ == IndoorTestMode::Drive) {
      // Keep at least a modest gap beyond the circular wheelchair footprint.
      indoor_drive_emergency_distance_ = std::max(
          get_parameter("safety.emergency_stop_distance").as_double(),
          dwa_config.robot_radius + 0.35);
    }
    dwa_config.road_margin = get_parameter("dwa.road_margin").as_double();
    dwa_config.enable_hardware_constraints =
        get_parameter("dwa.hardware_constraints.enabled").as_bool();
    if (indoor_test_mode_ == IndoorTestMode::Drive &&
        (!dwa_config.enable_hardware_constraints ||
         get_parameter("safety.perception_timeout").as_double() <= 0.0 ||
         get_parameter("velocity_feedback.timeout").as_double() <= 0.0 ||
         get_parameter("safety.emergency_stop_distance").as_double() <= 0.0)) {
      throw std::invalid_argument(
          "indoor drive requires BLE constraints, perception/odom timeouts, and emergency stop");
    }
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
    max_dwa_velocity_ = dwa_config.max_velocity;
    if (indoor_test_mode_ == IndoorTestMode::Visualize) {
      const double preview = get_parameter("dwa.indoor_test.preview_velocity").as_double();
      if (!std::isfinite(preview) || preview < 0.0) {
        throw std::invalid_argument("dwa.indoor_test.preview_velocity must be nonnegative");
      }
    }
    max_angular_velocity_ = dwa_config.max_angular_velocity;
    simulation_dt_ = dwa_config.simulation_time_step;
    max_angular_acceleration_ = dwa_config.max_angular_acceleration;
    max_linear_acceleration_ = dwa_config.max_acceleration;
    minimum_turning_velocity_ = dwa_config.minimum_turning_velocity;
    minimum_turning_radius_ = dwa_config.minimum_turning_radius;
    minimum_moving_velocity_ = dwa_config.enable_hardware_constraints
        ? dwa_config.minimum_moving_velocity : 0.0;
    if (indoor_test_mode_ == IndoorTestMode::Drive &&
        dwa_config.max_velocity < minimum_moving_velocity_) {
      throw std::invalid_argument("indoor max_velocity is below BLE minimum moving velocity");
    }
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
    dwa::Velocity last_sent_command;
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
    snapshot.last_sent_command = last_output_velocity_;
    return snapshot;
  }

  Eigen::Vector3d transformCameraPointToBase(
      double x, double y, double z) const {
    return camera_rotation_in_base_ * Eigen::Vector3d(x, y, z) +
           camera_translation_in_base_;
  }

  dwa::RoadModel makePlanningRoadModel(
      const wheel_msgs::msg::PerceptionOutput& message) const {
    if (indoor_test_mode_ != IndoorTestMode::Off) {
      // Laboratory mode has no reliable road edge. Heading zero means +X.
      return {};
    }
    return makeBaseRoadModel(message);
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

  std::optional<dwa::RoadModel> makeObservedRightRoadModel(
      const wheel_msgs::msg::PerceptionOutput& message) const {
    if (!message.has_road_edge) return std::nullopt;
    const auto& a = message.debug_line_pt1;
    const auto& b = message.debug_line_pt2;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
        !std::isfinite(b.x) || !std::isfinite(b.y) ||
        !std::isfinite(dx) || !std::isfinite(dy) ||
        std::abs(dx) < 0.05 || std::hypot(dx, dy) < 0.10) {
      return std::nullopt;
    }
    const double slope = dy / dx;
    const double yaw = std::atan2(dx >= 0.0 ? dy : -dy,
                                  dx >= 0.0 ? dx : -dx);
    const double right_distance = slope * a.x - a.y;
    if (!std::isfinite(right_distance) || std::cos(yaw) <= 1e-6) {
      return std::nullopt;
    }
    dwa::RoadModel observed;
    observed.has_right_edge = true;
    observed.right_distance = right_distance;
    observed.yaw_error = yaw;
    return observed;
  }

  bool sourceFrameFresh(std::int64_t stamp_ns,
                        const rclcpp::Time& reference_time) const {
    const std::int64_t now_ns = reference_time.nanoseconds();
    return stamp_ns > 0 && now_ns >= stamp_ns &&
        static_cast<double>(now_ns - stamp_ns) * 1e-9 <= max_sensor_age_;
  }

  void storeObstacleFrame(std::int64_t stamp_ns,
                          std::vector<dwa::ObstaclePoint> points) {
    {
      std::lock_guard<std::mutex> lock(obstacle_mutex_);
      obstacle_frames_.push_back({stamp_ns, std::move(points)});
      while (obstacle_frames_.size() > 3) obstacle_frames_.pop_front();
    }
    cloud_cv_.notify_all();
  }

  std::optional<std::vector<dwa::ObstaclePoint>> matchingObstacleFrame(
      std::int64_t stamp_ns) {
    std::unique_lock<std::mutex> lock(obstacle_mutex_);
    const auto find_frame = [&] {
      return std::find_if(obstacle_frames_.begin(), obstacle_frames_.end(),
          [stamp_ns](const ObstacleFrame& frame) {
            return frame.stamp_ns == stamp_ns;
          });
    };
    cloud_cv_.wait_for(lock, std::chrono::milliseconds(20),
                       [&] { return find_frame() != obstacle_frames_.end(); });
    const auto frame = find_frame();
    if (frame == obstacle_frames_.end()) return std::nullopt;
    return frame->points;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (message->width == 0 || message->height == 0) {
      storeObstacleFrame(stamp_ns, {});
      return;
    }

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

    storeObstacleFrame(stamp_ns, std::move(points));
  }

  void perceptionCallback(const wheel_msgs::msg::PerceptionOutput::SharedPtr message) {
    const rclcpp::Time callback_time = now();
    last_perception_receive_ns_.store(callback_time.nanoseconds());
    const std::uint64_t stop_epoch = watchdog_stop_epoch_.load();
    if (watchdog_stopped_.exchange(false)) {
      motion_history_ = {};
      last_tracked_angular_ = 0.0;
      lqr_->reset();
    }
    const std::int64_t frame_stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    if (!sourceFrameFresh(frame_stamp_ns, callback_time)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Reject stale perception frame: age=%.3f s, limit=%.3f s",
          static_cast<double>(callback_time.nanoseconds() - frame_stamp_ns) * 1e-9,
          max_sensor_age_);
      publishStop("perception frame stale or timestamp invalid");
      return;
    }
    double control_dt = simulation_dt_;
    if (last_control_time_.nanoseconds() > 0 && callback_time > last_control_time_) {
      control_dt = std::clamp((callback_time - last_control_time_).seconds(), 0.01, 0.25);
    }
    last_control_time_ = callback_time;
    const double emergency_distance = indoor_test_mode_ == IndoorTestMode::Drive
        ? indoor_drive_emergency_distance_
        : get_parameter("safety.emergency_stop_distance").as_double();
    if (message->min_distance > 0.01 && message->min_distance < emergency_distance) {
      publishStop("emergency obstacle distance");
      return;
    }

    // Never combine a new road estimate with a previous obstacle cloud.
    const auto matching_cloud = matchingObstacleFrame(frame_stamp_ns);
    if (!matching_cloud) {
      publishStop("obstacle cloud missing for perception frame");
      return;
    }
    const std::vector<dwa::ObstaclePoint>& obstacles = *matching_cloud;
    const dwa::RoadModel road = makePlanningRoadModel(*message);
    const auto observed_right_road = indoor_test_mode_ == IndoorTestMode::Off
        ? makeObservedRightRoadModel(*message)
        : std::optional<dwa::RoadModel>{};
    if (indoor_test_mode_ == IndoorTestMode::Off &&
        (!road.has_right_edge || !observed_right_road)) {
      publishStop("current road boundary unavailable or invalid");
      return;
    }

    const auto motion = planningMotionSnapshot(callback_time);
    if (indoor_test_mode_ == IndoorTestMode::Drive && !motion.feedback_fresh) {
      publishStop("indoor drive requires fresh /odom twist");
      return;
    }
    if (indoor_test_mode_ == IndoorTestMode::Off &&
        !dataset_mode_ && !motion.feedback_fresh) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Velocity feedback unavailable or stale; using last controller output");
      if (get_parameter("velocity_feedback.stop_on_timeout").as_bool()) {
        publishStop("velocity feedback timeout");
        return;
      }
    }
    const bool indoor_test = indoor_test_mode_ != IndoorTestMode::Off;
    dwa::Velocity planning_velocity = motion.velocity;
    if (indoor_test_mode_ == IndoorTestMode::Visualize) {
      // Hypothetical snapshot only: no robot motion is inferred from this speed.
      const double preview = get_parameter("dwa.indoor_test.preview_velocity").as_double();
      planning_velocity = {std::clamp(preview, 0.0, max_dwa_velocity_), 0.0};
    }
    const bool use_dwa = indoor_test || updateDwaActivation(obstacles);
    dwa::DWAPlanner::Result result;
    if (use_dwa) {
      const dwa::Velocity* last_sent = indoor_test_mode_ == IndoorTestMode::Drive
          ? &motion.last_sent_command : nullptr;
      result = dwa_->plan(motion.pose, planning_velocity, road, obstacles,
                          indoor_test_mode_ == IndoorTestMode::Visualize
                              ? dwa::MotionHistory{} : motion_history_,
                          control_dt, last_sent);
    } else {
      result.valid = true;
      result.best = makeCruiseTrajectory(road, planning_velocity, control_dt);
    }

    if (use_dwa && !result.valid &&
        (indoor_test || get_parameter("safety.stop_on_no_path").as_bool())) {
      if (result.command_window_empty) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "INDOOR_DWA NO_WINDOW_INTERSECTION: odom_w=%.3f sent_w=%.3f dt=%.3f",
            motion.velocity.angular, motion.last_sent_command.angular, control_dt);
      }
      if (indoor_test) logIndoorPlan(
          result, geometry_msgs::msg::Twist{}, indoor_test_mode_ == IndoorTestMode::Drive);
      publishStop(result.command_window_empty ? "DWA command windows do not overlap"
                                               : "DWA found no collision-free path",
                  indoor_test_mode_ != IndoorTestMode::Visualize);
      return;
    }
    if (!result.valid) {
      publishStop("no valid local trajectory");
      return;
    }

    geometry_msgs::msg::Twist planner_command;
    planner_command.linear.x = result.best.command.linear;
    planner_command.angular.z = result.best.command.angular;

    if (indoor_test_mode_ == IndoorTestMode::Visualize) {
      publishVisualization(result, use_dwa);
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        pub_dwa_cmd_->publish(planner_command);
        pub_indoor_test_cmd_->publish(planner_command);
        pub_cmd_->publish(geometry_msgs::msg::Twist{});
        std::lock_guard<std::mutex> velocity_lock(velocity_mutex_);
        last_output_velocity_ = {};
      }
      motion_history_ = {};
      last_tracked_angular_ = 0.0;
      logIndoorPlan(result, planner_command, false);
      return;
    }

    double lateral_error = 0.0;
    double heading_error = 0.0;
    const double reference_w = result.best.command.angular;
    double requested_w = reference_w;
    if (!indoor_test) {
      const auto& reference = selectLookahead(result.best);
      lateral_error = -reference.y;
      heading_error = -reference.yaw;
      const double raw_tracked_w = lqr_->computeTracking(
          lateral_error, heading_error, reference_w, control_dt);
      // Normal DWA bounds LQR correction; road cruise has its own yaw cap.
      requested_w = use_dwa
          ? reference_w + std::clamp(
                raw_tracked_w - reference_w,
                -max_dwa_tracking_correction_, max_dwa_tracking_correction_)
          : std::clamp(raw_tracked_w,
                       -max_cruise_angular_velocity_, max_cruise_angular_velocity_);
    }
    const double tracked_w = limitAngularCommand(
        result.best.command.linear, requested_w, control_dt);
    if (indoor_test_mode_ == IndoorTestMode::Drive &&
        std::abs(tracked_w - reference_w) > 1e-6) {
      publishStop("actuator limit changed checked DWA trajectory");
      return;
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = result.best.command.linear;
    command.angular.z = tracked_w;

    // The blue/green path must describe the command actually sent to BLE,
    // including LQR correction and the final yaw limiter, not merely the
    // DWA reference chosen before those changes.
    dwa::Trajectory checked_trajectory;
    const dwa::Velocity final_velocity{command.linear.x, command.angular.z};
    if (!dwa_->assessCommandSafety(
            final_velocity, road, obstacles, &checked_trajectory)) {
      const char* failure = checked_trajectory.poses.empty()
          ? "invalid geometry or actuator command"
          : (!checked_trajectory.inside_road ? "fitted road boundary"
                                             : "obstacle collision");
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Final command rejected: %s v=%.2f w=%.2f right=%.2f yaw=%.3f clearance=%.2f",
          failure, final_velocity.linear, final_velocity.angular,
          road.right_distance, road.yaw_error, checked_trajectory.minimum_clearance);
      publishStop(std::string("final command unsafe: ") + failure);
      return;
    }
    // The yellow RViz edge points are the current raw measurement; the EMA
    // road line can lag behind them. Enforce the raw fit as a second boundary.
    if (observed_right_road &&
        !dwa_->assessCommandSafety(
            final_velocity, *observed_right_road, {}, nullptr)) {
      publishStop("final command crosses current observed right boundary");
      return;
    }
    result.best.poses = std::move(checked_trajectory.poses);
    result.best.minimum_clearance = checked_trajectory.minimum_clearance;
    // Marker publication can be expensive; keep it outside the command lock.
    // If the watchdog fires meanwhile, the final epoch check below refuses
    // this command and publishStop clears the just-published path.
    publishVisualization(result, use_dwa);

    // Serialize the last safety check and cmd_vel publication with watchdog
    // zero commands. A watchdog stop during planning invalidates this result.
    bool published = false;
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      const rclcpp::Time publication_time = now();
      const double receive_age = static_cast<double>(
          publication_time.nanoseconds() - last_perception_receive_ns_.load()) * 1e-9;
      if (watchdog_stop_epoch_.load() == stop_epoch &&
          sourceFrameFresh(frame_stamp_ns, publication_time) &&
          receive_age >= 0.0 &&
          receive_age <= get_parameter("safety.perception_timeout").as_double()) {
        pub_dwa_cmd_->publish(planner_command);
        if (indoor_test) pub_indoor_test_cmd_->publish(planner_command);
        pub_cmd_->publish(command);
        std::lock_guard<std::mutex> velocity_lock(velocity_mutex_);
        last_output_velocity_ = {command.linear.x, command.angular.z};
        published = true;
      }
    }
    if (!published) {
      publishStop("planning result expired before command publication");
      return;
    }

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
    last_tracked_angular_ = tracked_w;

    if (indoor_test) {
      logIndoorPlan(result, command, true);
      return;
    }
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

  double limitAngularCommand(double linear, double angular, double control_dt) const {
    double limited = std::clamp(angular, -max_angular_velocity_, max_angular_velocity_);
    const double max_delta_w = max_angular_acceleration_ * control_dt;
    limited = std::clamp(limited, last_tracked_angular_ - max_delta_w,
                         last_tracked_angular_ + max_delta_w);
    if (linear < minimum_turning_velocity_) {
      return 0.0;
    }
    if (minimum_turning_radius_ > 1e-6) {
      const double curvature_limited_w = std::abs(linear) / minimum_turning_radius_;
      limited = std::clamp(limited, -curvature_limited_w, curvature_limited_w);
    }
    const double hardware_limited_w = dwa_->maximumHardwareAngularVelocity(linear);
    return std::clamp(limited, -hardware_limited_w, hardware_limited_w);
  }

  void logIndoorPlan(const dwa::DWAPlanner::Result& result,
                     const geometry_msgs::msg::Twist& command, bool executable) {
    const auto feasible = std::count_if(result.candidates.begin(), result.candidates.end(),
        [](const dwa::Trajectory& candidate) {
          return candidate.collision_free && candidate.inside_road &&
                 candidate.dynamic_feasible;
        });
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "INDOOR_DWA %s: v=%.2f w=%.2f candidates=%zu feasible=%zu "
        "clearance=%.2f score=%.2f",
        executable ? "DRIVE" : "PREVIEW", command.linear.x, command.angular.z,
        result.candidates.size(), static_cast<std::size_t>(feasible),
        result.valid ? result.best.minimum_clearance : 0.0,
        result.valid ? result.best.score : 0.0);
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
        ? road.yaw_error
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

  void clearPathVisualization(bool clear_candidates) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "base_link";
    pub_local_trajectory_->publish(empty_path);
    pub_best_path_->publish(empty_path);
    visualization_msgs::msg::Marker clear_best_path;
    clear_best_path.header.stamp = empty_path.header.stamp;
    clear_best_path.header.frame_id = "base_link";
    clear_best_path.ns = "selected_local_path";
    clear_best_path.id = 0;
    clear_best_path.action = visualization_msgs::msg::Marker::DELETE;
    pub_best_path_marker_->publish(clear_best_path);
    visualization_msgs::msg::MarkerArray clear_markers;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear_markers.markers.push_back(clear);
    if (clear_candidates) pub_candidate_paths_->publish(clear_markers);
  }

  void publishStop(const std::string& reason, bool clear_candidates = true) {
    {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      watchdog_stop_epoch_.fetch_add(1);
      geometry_msgs::msg::Twist stop;
      pub_cmd_->publish(stop);
      pub_dwa_cmd_->publish(stop);
      if (indoor_test_mode_ != IndoorTestMode::Off) pub_indoor_test_cmd_->publish(stop);
      {
        std::lock_guard<std::mutex> velocity_lock(velocity_mutex_);
        last_output_velocity_ = {};
      }
      clearPathVisualization(clear_candidates);
    }
    motion_history_ = {};
    last_tracked_angular_ = 0.0;
    lqr_->reset();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Safety stop: %s", reason.c_str());
  }

  void watchdogCallback() {
    const double timeout = get_parameter("safety.perception_timeout").as_double();
    if (timeout <= 0.0) return;
    const auto timed_out = [&] {
      const std::int64_t age_ns = now().nanoseconds() -
          last_perception_receive_ns_.load();
      return age_ns >= 0 && static_cast<double>(age_ns) * 1e-9 > timeout;
    };
    if (!timed_out()) return;
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (watchdog_stopped_.load() || !timed_out()) return;
    watchdog_stopped_.store(true);
    watchdog_stop_epoch_.fetch_add(1);
    geometry_msgs::msg::Twist stop;
    pub_cmd_->publish(stop);
    pub_dwa_cmd_->publish(stop);
    if (indoor_test_mode_ != IndoorTestMode::Off) pub_indoor_test_cmd_->publish(stop);
    {
      std::lock_guard<std::mutex> velocity_lock(velocity_mutex_);
      last_output_velocity_ = {};
    }
    clearPathVisualization(true);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                         "Safety stop: perception callback timeout");
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
  rclcpp::CallbackGroup::SharedPtr odom_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::CallbackGroup::SharedPtr watchdog_group_;
  rclcpp::Subscription<wheel_msgs::msg::PerceptionOutput>::SharedPtr sub_perception_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dataset_odom_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_dwa_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_indoor_test_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_trajectory_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_best_path_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_best_path_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_candidate_paths_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dwa_obstacle_cloud_;

  std::mutex obstacle_mutex_;
  std::condition_variable cloud_cv_;
  std::deque<ObstacleFrame> obstacle_frames_;
  std::mutex velocity_mutex_;
  std::mutex command_mutex_;
  dwa::Pose2D robot_state_;
  dwa::Velocity measured_velocity_;
  dwa::MotionHistory motion_history_;
  dwa::Velocity last_output_velocity_;
  bool has_velocity_feedback_{false};
  bool dataset_mode_{true};
  IndoorTestMode indoor_test_mode_{IndoorTestMode::Off};
  std::string base_frame_id_{"base_link"};
  Eigen::Vector3d camera_translation_in_base_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d camera_rotation_in_base_{Eigen::Matrix3d::Identity()};
  double max_dwa_velocity_{1.0};
  double max_sensor_age_{0.25};
  double indoor_drive_emergency_distance_{0.8};
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
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::atomic<std::int64_t> last_perception_receive_ns_{0};
  std::atomic<std::uint64_t> watchdog_stop_epoch_{0};
  std::atomic<bool> watchdog_stopped_{false};
};

}  // namespace wheel_control

RCLCPP_COMPONENTS_REGISTER_NODE(wheel_control::ControllerNode)
