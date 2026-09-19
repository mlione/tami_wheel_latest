#include <cmath>
#include <string>

#include "ble_hardware_bridge/control_protocol.hpp"
#include "gtest/gtest.h"

namespace ble_hardware_bridge
{
namespace
{

TEST(ControlProtocol, UsesVehicleCoordinateAxisConvention)
{
  const ControlProtocol protocol;
  const auto result = protocol.map_twist(0.8, 0.5);

  ASSERT_TRUE(result.joystick.has_value()) << result.error;
  EXPECT_GT(result.joystick->x, 0.3);
  EXPECT_GT(result.joystick->y, 0.3);
}

TEST(ControlProtocol, AllowsZeroSteeringForStraightMotion)
{
  const ControlProtocol protocol;
  const auto result = protocol.map_twist(0.8, 0.0);

  ASSERT_TRUE(result.joystick.has_value()) << result.error;
  EXPECT_GT(result.joystick->x, 0.3);
  EXPECT_DOUBLE_EQ(result.joystick->y, 0.0);
}

TEST(ControlProtocol, StopsWhenForwardAxisIsInDeadzone)
{
  const ControlProtocol protocol;
  const auto result = protocol.map_twist(0.1, 0.0);

  EXPECT_FALSE(result.joystick.has_value());
  EXPECT_NE(result.error.find("X (forward)"), std::string::npos);
  EXPECT_NE(result.error.find("below deadzone 0.300"), std::string::npos);
}

TEST(ControlProtocol, AllowsSteeringAxisInsideDeadzoneWhenForwardAxisIsActive)
{
  const ControlProtocol protocol;
  const auto result = protocol.map_twist(0.8, 0.1);

  ASSERT_TRUE(result.joystick.has_value()) << result.error;
  EXPECT_GT(result.joystick->x, 0.3);
  EXPECT_GT(result.joystick->y, 0.0);
  EXPECT_LT(result.joystick->y, 0.3);
}

TEST(ControlProtocol, SteeringSignIsConfiguredAtTheHardwareBoundary)
{
  MappingParameters parameters;
  parameters.steering_sign = -1.0;
  const ControlProtocol protocol(parameters);
  const auto result = protocol.map_twist(0.8, 0.1);

  ASSERT_TRUE(result.joystick.has_value()) << result.error;
  EXPECT_LT(result.joystick->y, 0.0);
}

TEST(ControlProtocol, AcceptsDwppHardwareConstraintBoundaries)
{
  const ControlProtocol protocol;

  EXPECT_TRUE(protocol.map_twist(0.428475, 0.0).joystick.has_value());
  EXPECT_TRUE(protocol.map_twist(0.5, 0.1).joystick.has_value());
  EXPECT_TRUE(protocol.map_twist(0.5, -0.1).joystick.has_value());
}

TEST(ControlProtocol, SelectsGearFromLinearVelocityThresholds)
{
  const ControlProtocol protocol;

  const auto below_first_threshold = protocol.map_twist(0.349, 0.0);
  const auto at_first_threshold = protocol.map_twist(0.35, 0.0);
  const auto below_second_threshold = protocol.map_twist(0.699, 0.0);
  const auto at_second_threshold = protocol.map_twist(0.70, 0.0);

  ASSERT_TRUE(below_first_threshold.joystick) << below_first_threshold.error;
  ASSERT_TRUE(at_first_threshold.joystick) << at_first_threshold.error;
  ASSERT_TRUE(below_second_threshold.joystick) << below_second_threshold.error;
  ASSERT_TRUE(at_second_threshold.joystick) << at_second_threshold.error;
  EXPECT_EQ(below_first_threshold.joystick->gear, SpeedGear::k0001);
  EXPECT_EQ(at_first_threshold.joystick->gear, SpeedGear::k0003);
  EXPECT_EQ(below_second_threshold.joystick->gear, SpeedGear::k0003);
  EXPECT_EQ(at_second_threshold.joystick->gear, SpeedGear::k0005);
}

TEST(ControlProtocol, ScalesLinearAndAngularVelocityFromGear0005Calibration)
{
  const MappingParameters parameters;
  const ControlProtocol protocol(parameters);
  const double gear_0001_scale =
    parameters.gear_0001_maximum_linear_velocity /
    parameters.gear_0005_maximum_linear_velocity;
  const double gear_0003_scale =
    parameters.gear_0003_maximum_linear_velocity /
    parameters.gear_0005_maximum_linear_velocity;

  const auto gear_0005 = protocol.map_twist(1.0, 0.2);
  const auto gear_0003 =
    protocol.map_twist(gear_0003_scale, gear_0003_scale * 0.2);
  const auto gear_0001 =
    protocol.map_twist(gear_0001_scale, gear_0001_scale * 0.2);

  ASSERT_TRUE(gear_0005.joystick) << gear_0005.error;
  ASSERT_TRUE(gear_0003.joystick) << gear_0003.error;
  ASSERT_TRUE(gear_0001.joystick) << gear_0001.error;
  EXPECT_EQ(gear_0005.joystick->gear, SpeedGear::k0005);
  EXPECT_EQ(gear_0003.joystick->gear, SpeedGear::k0003);
  EXPECT_EQ(gear_0001.joystick->gear, SpeedGear::k0001);
  EXPECT_NEAR(gear_0003.joystick->x, gear_0005.joystick->x, 1.0e-12);
  EXPECT_NEAR(gear_0003.joystick->y, gear_0005.joystick->y, 1.0e-12);
  EXPECT_NEAR(gear_0001.joystick->x, gear_0005.joystick->x, 1.0e-12);
  EXPECT_NEAR(gear_0001.joystick->y, gear_0005.joystick->y, 1.0e-12);
}

TEST(ControlProtocol, AcceptsGear0005MaximumLinearVelocity)
{
  const ControlProtocol protocol;
  const auto result = protocol.map_twist(1.35, 0.0);

  ASSERT_TRUE(result.joystick) << result.error;
  EXPECT_EQ(result.joystick->gear, SpeedGear::k0005);
  EXPECT_DOUBLE_EQ(result.joystick->x, 1.0);
}

TEST(ControlProtocol, EncodesSteeringBeforeForwardVelocity)
{
  const ControlProtocol protocol;
  const auto frame = protocol.encode({0.5, -0.25});

  // round(2116 - 1600 * 0.25) = 1716 = 0x06B4
  EXPECT_EQ(frame[5], 0xB4);
  EXPECT_EQ(frame[6], 0x06);
  // round(2116 + 1600 * 0.5) = 2916 = 0x0B64
  EXPECT_EQ(frame[7], 0x64);
  EXPECT_EQ(frame[8], 0x0B);
  EXPECT_EQ(frame[9], 0x00);
  EXPECT_EQ(frame[10], 0x05);
}

TEST(ControlProtocol, EncodesSelectedGearAndUsesGear0001ForStop)
{
  const ControlProtocol protocol;
  const auto gear_0003_frame =
    protocol.encode({0.5, -0.25, SpeedGear::k0003});
  const auto stop_frame = protocol.stop_frame();

  EXPECT_EQ(gear_0003_frame[9], 0x00);
  EXPECT_EQ(gear_0003_frame[10], 0x03);
  EXPECT_EQ(stop_frame[9], 0x00);
  EXPECT_EQ(stop_frame[10], 0x01);
}

}  // namespace
}  // namespace ble_hardware_bridge
