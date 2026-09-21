#include <gtest/gtest.h>

#include <cmath>

#include "wheel_perception/core/world_pose_velocity_estimator.hpp"

namespace {

using wheel_perception::core::WorldPoseVelocityEstimator;

TEST(WorldPoseVelocityEstimator, DerivesForwardSpeedFromImageTime) {
    WorldPoseVelocityEstimator estimator(0.01, 0.5);
    const Eigen::Isometry3d base_T_camera = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();

    EXPECT_FALSE(estimator.update(world_T_camera, base_T_camera, 1000000000ULL)
                     .velocity_valid);
    world_T_camera.translation().x() = 0.1;
    const auto estimate =
        estimator.update(world_T_camera, base_T_camera, 1100000000ULL);

    ASSERT_TRUE(estimate.velocity_valid);
    EXPECT_NEAR(estimate.sample_period, 0.1, 1e-9);
    EXPECT_NEAR(estimate.linear_velocity.x(), 1.0, 1e-9);
    EXPECT_NEAR(estimate.angular_velocity.z(), 0.0, 1e-9);
}

TEST(WorldPoseVelocityEstimator, RemovesCameraLeverArmBeforeDifferencing) {
    WorldPoseVelocityEstimator estimator(0.01, 0.5);
    Eigen::Isometry3d base_T_camera = Eigen::Isometry3d::Identity();
    base_T_camera.translation() = Eigen::Vector3d(0.2, -0.2, 0.0);

    // Stationary base: the camera pose is exactly the fixed base->camera pose.
    Eigen::Isometry3d world_T_camera = base_T_camera;
    EXPECT_FALSE(estimator.update(world_T_camera, base_T_camera, 1000000000ULL)
                     .velocity_valid);

    // Rotate the base 0.1 rad. The camera travels around the base, but removing
    // the extrinsic must leave zero base translation and 1 rad/s yaw rate.
    Eigen::Isometry3d world_T_base = Eigen::Isometry3d::Identity();
    world_T_base.linear() =
        Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    world_T_camera = world_T_base * base_T_camera;
    const auto estimate =
        estimator.update(world_T_camera, base_T_camera, 1100000000ULL);

    ASSERT_TRUE(estimate.velocity_valid);
    EXPECT_NEAR(estimate.linear_velocity.norm(), 0.0, 1e-9);
    EXPECT_NEAR(estimate.angular_velocity.z(), 1.0, 1e-9);
}

TEST(WorldPoseVelocityEstimator, RejectsInvalidSamplingGapAndRebasesVelocity) {
    WorldPoseVelocityEstimator estimator(0.01, 0.2);
    const Eigen::Isometry3d base_T_camera = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    estimator.update(pose, base_T_camera, 1000000000ULL);
    pose.translation().x() = 1.0;
    EXPECT_FALSE(estimator.update(pose, base_T_camera, 2000000000ULL)
                     .velocity_valid);

    pose.translation().x() = 1.05;
    const auto estimate = estimator.update(pose, base_T_camera, 2050000000ULL);
    ASSERT_TRUE(estimate.velocity_valid);
    EXPECT_NEAR(estimate.linear_velocity.x(), 1.0, 1e-9);
}

}  // namespace
