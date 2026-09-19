#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "ble_hardware_bridge/bluez_ble_client.hpp"
#include "ble_hardware_bridge/control_protocol.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ble_hardware_bridge
{

using Clock = std::chrono::steady_clock;

class BleHardwareBridgeNode final : public rclcpp::Node
{
public:
  BleHardwareBridgeNode()
  : Node("ble_hardware_bridge_node")
  {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    send_rate_ = declare_parameter<double>("send_rate", 20.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.3);
    reconnect_delay_sec_ = declare_parameter<double>("reconnect_delay_sec", 2.0);
    invalid_warning_period_sec_ =
      declare_parameter<double>("invalid_command_warning_period_sec", 1.0);

    MappingParameters mapping;
    mapping.wheel_separation = declare_parameter<double>("wheel_separation", 0.53);
    mapping.maximum_wheel_velocity =
      declare_parameter<double>("maximum_wheel_velocity", 1.35);
    mapping.maximum_absolute_angular_velocity =
      declare_parameter<double>("maximum_absolute_angular_velocity", 1.0);
    mapping.joystick_deadzone =
      declare_parameter<double>("joystick_deadzone", 0.3);
    mapping.maximum_absolute_joystick_y =
      declare_parameter<double>("maximum_absolute_joystick_y", 0.9);
    mapping.steering_sign = declare_parameter<double>("steering_sign", 1.0);
    mapping.longitudinal_gain =
      declare_parameter<double>("longitudinal_gain", 1.34449935);
    mapping.longitudinal_exponent =
      declare_parameter<double>("longitudinal_exponent", 0.94981010);
    mapping.gear_0001_maximum_linear_velocity =
      declare_parameter<double>("gear_0001_maximum_linear_velocity", 0.45);
    mapping.gear_0003_maximum_linear_velocity =
      declare_parameter<double>("gear_0003_maximum_linear_velocity", 0.90);
    mapping.gear_0005_maximum_linear_velocity =
      declare_parameter<double>("gear_0005_maximum_linear_velocity", 1.35);
    mapping.gear_0001_selection_threshold =
      declare_parameter<double>("gear_0001_selection_threshold", 0.35);
    mapping.gear_0003_selection_threshold =
      declare_parameter<double>("gear_0003_selection_threshold", 0.70);
    mapping.zero_command_epsilon =
      declare_parameter<double>("zero_command_epsilon", 1.0e-6);

    BluezBleClient::Config ble_config;
    ble_config.adapter_path =
      declare_parameter<std::string>("adapter_path", "/org/bluez/hci0");
    ble_config.device_address =
      declare_parameter<std::string>("device_address", "11:89:88:11:A1:0C");
    ble_config.characteristic_uuid = declare_parameter<std::string>(
      "characteristic_uuid", "0000ffe1-0000-1000-8000-00805f9b34fb");

    validate_parameters(mapping, ble_config);
    ble_client_ = std::make_unique<BluezBleClient>(stop_requested_, std::move(ble_config));
    protocol_ = ControlProtocol(mapping);
    stop_frame_ = protocol_.stop_frame();
    latest_frame_ = stop_frame_;

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist & message) {
        on_cmd_vel(message);
      });

    worker_ = std::thread(&BleHardwareBridgeNode::run_ble, this);
    RCLCPP_INFO(
      get_logger(), "BLE bridge ready: topic=%s, send_rate=%.1f Hz",
      cmd_vel_topic_.c_str(), send_rate_);
  }

  ~BleHardwareBridgeNode() override
  {
    stop_requested_.store(true);
    wait_condition_.notify_all();
    ble_client_->cancel();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void validate_parameters(
    const MappingParameters & mapping,
    const BluezBleClient::Config & ble_config) const
  {
    if (cmd_vel_topic_.empty() || !(send_rate_ > 0.0) ||
      !(command_timeout_sec_ > 0.0) || !(reconnect_delay_sec_ > 0.0) ||
      !(mapping.wheel_separation > 0.0) ||
      !(mapping.maximum_wheel_velocity > 0.0) ||
      !(mapping.maximum_absolute_angular_velocity > 0.0) ||
      mapping.joystick_deadzone < 0.0 || mapping.joystick_deadzone >= 1.0 ||
      !(mapping.maximum_absolute_joystick_y > 0.0) ||
      std::abs(std::abs(mapping.steering_sign) - 1.0) > 1.0e-6 ||
      !(mapping.longitudinal_gain > 0.0) ||
      !(mapping.longitudinal_exponent > 0.0) ||
      !(mapping.gear_0001_maximum_linear_velocity > 0.0) ||
      !(mapping.gear_0003_maximum_linear_velocity >
      mapping.gear_0001_maximum_linear_velocity) ||
      !(mapping.gear_0005_maximum_linear_velocity >
      mapping.gear_0003_maximum_linear_velocity) ||
      !(mapping.maximum_wheel_velocity >=
      mapping.gear_0005_maximum_linear_velocity) ||
      !(mapping.gear_0001_selection_threshold > 0.0) ||
      !(mapping.gear_0003_selection_threshold >
      mapping.gear_0001_selection_threshold) ||
      !(mapping.gear_0001_maximum_linear_velocity >
      mapping.gear_0001_selection_threshold) ||
      !(mapping.gear_0003_maximum_linear_velocity >
      mapping.gear_0003_selection_threshold) ||
      mapping.zero_command_epsilon < 0.0 || ble_config.adapter_path.empty() ||
      ble_config.device_address.empty() || ble_config.characteristic_uuid.empty())
    {
      throw std::invalid_argument("invalid BLE bridge parameters");
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist & message)
  {
    const double linear_velocity = message.linear.x;
    const double angular_velocity = message.angular.z;
    const auto mapping = protocol_.map_twist(linear_velocity, angular_velocity);
    const auto now = Clock::now();

    BleFrame frame = stop_frame_;
    if (mapping.joystick) {
      frame = protocol_.encode(*mapping.joystick);
    } else if (!last_invalid_warning_ ||
      std::chrono::duration<double>(now - *last_invalid_warning_).count() >=
      invalid_warning_period_sec_)
    {
      RCLCPP_WARN(
        get_logger(), "Rejected cmd_vel and queued stop: v=%.3f m/s, w=%.3f rad/s: %.*s",
        linear_velocity, angular_velocity, static_cast<int>(mapping.error.size()),
        mapping.error.c_str());
      last_invalid_warning_ = now;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_frame_ = frame;
    last_command_time_ = now;
  }

  BleFrame sample_frame()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!last_command_time_ ||
      std::chrono::duration<double>(Clock::now() - *last_command_time_).count() >
      command_timeout_sec_)
    {
      return stop_frame_;
    }
    return latest_frame_;
  }

  bool wait_until(Clock::time_point deadline)
  {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    return wait_condition_.wait_until(
      lock, deadline, [this] {
        return stop_requested_.load() || !rclcpp::ok();
      });
  }

  void run_ble()
  {
    const auto period = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(1.0 / send_rate_));
    const auto reconnect_delay = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(reconnect_delay_sec_));

    while (!stop_requested_.load() && rclcpp::ok()) {
      bool connected = false;
      try {
        RCLCPP_INFO(get_logger(), "Connecting to BLE device...");
        ble_client_->connect();
        connected = true;
        RCLCPP_INFO(
          get_logger(), "Connected. Writing commands to %s",
          ble_client_->characteristic_path().c_str());

        auto next_send = Clock::now();
        while (!stop_requested_.load() && rclcpp::ok()) {
          ble_client_->write(sample_frame());
          next_send = std::max(next_send + period, Clock::now());
          if (wait_until(next_send)) {
            break;
          }
        }

        if (stop_requested_.load() || !rclcpp::ok()) {
          send_final_stop();
          break;
        }
      } catch (const std::exception & error) {
        if (stop_requested_.load() || !rclcpp::ok()) {
          if (connected) {
            send_final_stop();
          }
          break;
        }
        RCLCPP_ERROR(
          get_logger(), "BLE disconnected or failed: %s; retrying...", error.what());
        ble_client_->reset();
        if (wait_until(Clock::now() + reconnect_delay)) {
          break;
        }
      }
    }
  }

  void send_final_stop()
  {
    try {
      ble_client_->write_stop(stop_frame_);
      RCLCPP_INFO(get_logger(), "Final BLE stop command sent");
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(), "Failed to send final BLE stop command: %s", error.what());
    }
  }

  std::atomic_bool stop_requested_{false};
  std::unique_ptr<BluezBleClient> ble_client_;
  ControlProtocol protocol_;

  std::string cmd_vel_topic_;
  double send_rate_{20.0};
  double command_timeout_sec_{0.3};
  double reconnect_delay_sec_{2.0};
  double invalid_warning_period_sec_{1.0};

  BleFrame stop_frame_{};
  BleFrame latest_frame_{};
  std::mutex command_mutex_;
  std::optional<Clock::time_point> last_command_time_;
  std::optional<Clock::time_point> last_invalid_warning_;

  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::thread worker_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

} // namespace ble_hardware_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ble_hardware_bridge::BleHardwareBridgeNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
