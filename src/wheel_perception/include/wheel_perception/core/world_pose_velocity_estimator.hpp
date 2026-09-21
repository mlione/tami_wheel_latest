#pragma once

#include <cstdint>

#include <Eigen/Geometry>

namespace wheel_perception::core {

struct WorldPoseVelocityEstimate {
    Eigen::Isometry3d odom_T_base{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    double sample_period{0.0};
    bool pose_valid{false};
    bool velocity_valid{false};
};

/**
 * @brief Derive base-link velocity from consecutive absolute ZED WORLD poses.
 *
 * The estimator keeps SDK velocity out of the control feedback path.  It first
 * removes the fixed camera lever arm, then differences consecutive base poses
 * using the image timestamps that produced those poses.
 */
class WorldPoseVelocityEstimator {
public:
    WorldPoseVelocityEstimator(double minimum_sample_period,
                               double maximum_sample_period);

    WorldPoseVelocityEstimate update(
        const Eigen::Isometry3d& world_T_camera,
        const Eigen::Isometry3d& base_T_camera,
        std::uint64_t image_timestamp_ns);

    // Preserve the odom origin but require two fresh poses before publishing a
    // new velocity. Used after temporary tracking loss or an invalid timestamp.
    void invalidateVelocityHistory();

    // Reset both the velocity history and the local odom origin.
    void reset();

private:
    static Eigen::Vector3d rotationVector(const Eigen::Matrix3d& rotation);

    double minimum_sample_period_;
    double maximum_sample_period_;
    bool has_origin_{false};
    bool has_previous_pose_{false};
    std::uint64_t previous_timestamp_ns_{0};
    Eigen::Isometry3d origin_world_T_base_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d previous_world_T_base_{Eigen::Isometry3d::Identity()};
};

}  // namespace wheel_perception::core
