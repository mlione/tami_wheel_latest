#include "ble_hardware_bridge/control_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ble_hardware_bridge
{
namespace
{

constexpr int kPwmCenter = 2116;
constexpr int kPwmRange = 1600;

struct CalibrationPoint
{
  double x;
  double differential;
};

constexpr std::array<CalibrationPoint, 13> kXToDifferential{{
  {-0.90, -0.282140},
  {-0.80, -0.251145},
  {-0.70, -0.216893},
  {-0.40, -0.119475},
  {-0.20, -0.056844},
  {-0.10, 0.000000},
  {0.00, 0.000000},
  {0.05, 0.027066},
  {0.10, 0.045977},
  {0.20, 0.074416},
  {0.40, 0.142836},
  {0.70, 0.239035},
  {0.80, 0.272361},
}};

std::optional<double> inverse_differential(double target)
{
  if (std::abs(target) <= 1.0e-12) {
    return 0.0;
  }
  if (target < kXToDifferential.front().differential ||
    target > kXToDifferential.back().differential)
  {
    return std::nullopt;
  }

  const auto right =
    std::lower_bound(
    kXToDifferential.begin(), kXToDifferential.end(), target,
    [](const CalibrationPoint & point, double value) {
      return point.differential < value;
    });

  if (right == kXToDifferential.begin()) {
    return right->x;
  }
  if (right == kXToDifferential.end()) {
    return std::nullopt;
  }

  const auto & left = *std::prev(right);
  const double fraction =
    (target - left.differential) / (right->differential - left.differential);
  return left.x + fraction * (right->x - left.x);
}

void write_little_endian(
  std::uint16_t value, BleFrame & frame,
  std::size_t offset)
{
  frame[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  frame[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

std::uint16_t encode_axis(double axis)
{
  return static_cast<std::uint16_t>(
    std::nearbyint(kPwmCenter + kPwmRange * axis));
}

bool is_in_deadzone(double axis, const MappingParameters & parameters)
{
  return std::abs(axis) < parameters.joystick_deadzone;
}

struct GearMapping
{
  SpeedGear gear;
  double maximum_linear_velocity;
};

GearMapping select_gear(double linear_velocity, const MappingParameters & parameters)
{
  if (linear_velocity < parameters.gear_0001_selection_threshold) {
    return {SpeedGear::k0001, parameters.gear_0001_maximum_linear_velocity};
  }
  if (linear_velocity < parameters.gear_0003_selection_threshold) {
    return {SpeedGear::k0003, parameters.gear_0003_maximum_linear_velocity};
  }
  return {SpeedGear::k0005, parameters.gear_0005_maximum_linear_velocity};
}

std::string deadzone_error(
  const char * axis_name, double value, double deadzone)
{
  std::ostringstream message;
  message << std::fixed << std::setprecision(3)
          << "mapped joystick " << axis_name << " value " << value
          << " has magnitude below deadzone " << deadzone;
  return message.str();
}

} // namespace

ControlProtocol::ControlProtocol(MappingParameters parameters)
: parameters_(parameters) {}

MappingResult ControlProtocol::map_twist(
  double linear_velocity,
  double angular_velocity) const
{
  if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity)) {
    return {std::nullopt, "command contains NaN or infinity"};
  }

  if (std::abs(linear_velocity) <= parameters_.zero_command_epsilon &&
    std::abs(angular_velocity) <= parameters_.zero_command_epsilon)
  {
    return {JoystickCommand{0.0, 0.0, SpeedGear::k0001}, {}};
  }

  angular_velocity *= parameters_.steering_sign;
  const auto gear_mapping = select_gear(linear_velocity, parameters_);
  const double gear_scale =
    gear_mapping.maximum_linear_velocity /
    parameters_.gear_0005_maximum_linear_velocity;
  const double equivalent_linear_velocity = linear_velocity / gear_scale;
  const double equivalent_angular_velocity = angular_velocity / gear_scale;

  if (equivalent_linear_velocity >
    parameters_.gear_0005_maximum_linear_velocity)
  {
    return {std::nullopt, "linear velocity exceeds the selected gear range"};
  }

  if (std::abs(equivalent_angular_velocity) >
    parameters_.maximum_absolute_angular_velocity)
  {
    return {std::nullopt, "angular velocity exceeds the calibrated range"};
  }

  const double half_wheel_separation = 0.5 * parameters_.wheel_separation;
  const double right_wheel =
    equivalent_linear_velocity +
    half_wheel_separation * equivalent_angular_velocity;
  const double left_wheel =
    equivalent_linear_velocity -
    half_wheel_separation * equivalent_angular_velocity;

  if (right_wheel < 0.0 || right_wheel > parameters_.maximum_wheel_velocity) {
    return {std::nullopt,
      "right wheel target is outside [0, maximum_wheel_velocity]"};
  }
  if (left_wheel < 0.0 || left_wheel > parameters_.maximum_wheel_velocity) {
    return {std::nullopt,
      "left wheel target is outside [0, maximum_wheel_velocity]"};
  }

  const double raw_joystick_x =
    std::pow(
    equivalent_linear_velocity / parameters_.longitudinal_gain,
    1.0 / parameters_.longitudinal_exponent);
  const double joystick_x = std::min(raw_joystick_x, 1.0);

  const auto joystick_y =
    inverse_differential(half_wheel_separation * equivalent_angular_velocity);
  if (!joystick_y ||
    std::abs(*joystick_y) > parameters_.maximum_absolute_joystick_y)
  {
    return {std::nullopt,
      "differential is outside the calibrated joystick Y range"};
  }

  if (is_in_deadzone(joystick_x, parameters_)) {
    return {std::nullopt,
      deadzone_error("X (forward)", joystick_x, parameters_.joystick_deadzone)};
  }

  return {JoystickCommand{joystick_x, *joystick_y, gear_mapping.gear}, {}};
}

BleFrame ControlProtocol::encode(const JoystickCommand & joystick) const
{
  BleFrame frame{0xEB, 0x90, 0x0F, 0xA2, 0xAA,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xCC, 0x33, 0xC3, 0x3C};

  // The device protocol is steering first, then forward velocity, regardless
  // of the vehicle-coordinate X/Y naming used by JoystickCommand.
  write_little_endian(encode_axis(joystick.y), frame, 5);
  write_little_endian(encode_axis(joystick.x), frame, 7);
  frame[10] = static_cast<std::uint8_t>(joystick.gear);
  return frame;
}

BleFrame ControlProtocol::stop_frame() const
{
  return encode({0.0, 0.0, SpeedGear::k0001});
}

} // namespace ble_hardware_bridge
