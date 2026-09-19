#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

namespace ble_hardware_bridge
{

using BleFrame = std::array<std::uint8_t, 15>;

enum class SpeedGear : std::uint8_t
{
  k0001 = 0x01,
  k0003 = 0x03,
  k0005 = 0x05,
};

struct MappingParameters
{
  double wheel_separation{0.53};
  double maximum_wheel_velocity{1.35};
  double maximum_absolute_angular_velocity{1.0};
  double joystick_deadzone{0.3};
  double maximum_absolute_joystick_y{0.9};
  // ROS uses positive angular.z for a left turn. Change only this parameter
  // after a wheels-off-ground direction check if the device is reversed.
  double steering_sign{1.0};
  double longitudinal_gain{1.34449935};
  double longitudinal_exponent{0.94981010};
  double gear_0001_maximum_linear_velocity{0.45};
  double gear_0003_maximum_linear_velocity{0.90};
  double gear_0005_maximum_linear_velocity{1.35};
  double gear_0001_selection_threshold{0.35};
  double gear_0003_selection_threshold{0.70};
  double zero_command_epsilon{1.0e-6};
};

struct JoystickCommand
{
  // Vehicle-coordinate convention: X is forward and Y is steering.
  double x;
  double y;
  SpeedGear gear{SpeedGear::k0005};
};

struct MappingResult
{
  std::optional<JoystickCommand> joystick;
  std::string error;
};

class ControlProtocol
{
public:
  explicit ControlProtocol(MappingParameters parameters = {});

  [[nodiscard]] MappingResult map_twist(
    double linear_velocity,
    double angular_velocity) const;
  [[nodiscard]] BleFrame encode(const JoystickCommand & joystick) const;
  [[nodiscard]] BleFrame stop_frame() const;

private:
  MappingParameters parameters_;
};

} // namespace ble_hardware_bridge
