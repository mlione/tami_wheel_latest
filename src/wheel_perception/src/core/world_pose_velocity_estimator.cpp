#include "wheel_perception/core/world_pose_velocity_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace wheel_perception::core {

WorldPoseVelocityEstimator::WorldPoseVelocityEstimator(
    double minimum_sample_period,
    double maximum_sample_period)
    : minimum_sample_period_(std::max(0.0, minimum_sample_period)),
      maximum_sample_period_(std::max(minimum_sample_period_, maximum_sample_period)) {}

Eigen::Vector3d WorldPoseVelocityEstimator::rotationVector(
    const Eigen::Matrix3d& rotation) {
    const Eigen::AngleAxisd angle_axis(rotation);
    if (!std::isfinite(angle_axis.angle()) ||
        !angle_axis.axis().allFinite() ||
        std::abs(angle_axis.angle()) < 1e-12) {
        return Eigen::Vector3d::Zero();
    }
    return angle_axis.axis() * angle_axis.angle();
}

WorldPoseVelocityEstimate WorldPoseVelocityEstimator::update(
    const Eigen::Isometry3d& world_T_camera,
    const Eigen::Isometry3d& base_T_camera,
    std::uint64_t image_timestamp_ns) {
    WorldPoseVelocityEstimate estimate;
    if (image_timestamp_ns == 0 ||
        !world_T_camera.matrix().allFinite() ||
        !base_T_camera.matrix().allFinite()) {
        invalidateVelocityHistory();
        return estimate;
    }

    // ^W T_B = ^W T_C * ^C T_B.  This removes both the camera orientation
    // offset and the translational lever arm before taking a derivative.
    const Eigen::Isometry3d world_T_base =
        world_T_camera * base_T_camera.inverse();

    if (!has_origin_) {
        origin_world_T_base_ = world_T_base;
        has_origin_ = true;
    }
    estimate.odom_T_base = origin_world_T_base_.inverse() * world_T_base;
    estimate.pose_valid = true;

    if (!has_previous_pose_) {
        previous_world_T_base_ = world_T_base;
        previous_timestamp_ns_ = image_timestamp_ns;
        has_previous_pose_ = true;
        return estimate;
    }

    if (image_timestamp_ns <= previous_timestamp_ns_) {
        previous_world_T_base_ = world_T_base;
        previous_timestamp_ns_ = image_timestamp_ns;
        return estimate;
    }

    const double sample_period =
        static_cast<double>(image_timestamp_ns - previous_timestamp_ns_) * 1e-9;
    const Eigen::Isometry3d previous_T_current =
        previous_world_T_base_.inverse() * world_T_base;

    previous_world_T_base_ = world_T_base;
    previous_timestamp_ns_ = image_timestamp_ns;

    estimate.sample_period = sample_period;
    if (sample_period < minimum_sample_period_ ||
        sample_period > maximum_sample_period_) {
        return estimate;
    }

    // The relative transform is expressed in the previous base frame, which
    // is the body-frame convention expected by nav_msgs/Odometry.twist.
    estimate.linear_velocity = previous_T_current.translation() / sample_period;
    estimate.angular_velocity =
        rotationVector(previous_T_current.linear()) / sample_period;
    estimate.velocity_valid = estimate.linear_velocity.allFinite() &&
        estimate.angular_velocity.allFinite();
    return estimate;
}

void WorldPoseVelocityEstimator::invalidateVelocityHistory() {
    has_previous_pose_ = false;
    previous_timestamp_ns_ = 0;
    previous_world_T_base_.setIdentity();
}

void WorldPoseVelocityEstimator::reset() {
    invalidateVelocityHistory();
    has_origin_ = false;
    origin_world_T_base_.setIdentity();
}

}  // namespace wheel_perception::core
