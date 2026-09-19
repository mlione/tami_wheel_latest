#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "ble_hardware_bridge/control_protocol.hpp"

namespace ble_hardware_bridge
{

class BluezBleClient
{
public:
  struct Config
  {
    std::string adapter_path{"/org/bluez/hci0"};
    std::string device_address{"11:89:88:11:A1:0C"};
    std::string characteristic_uuid{"0000ffe1-0000-1000-8000-00805f9b34fb"};
  };

  BluezBleClient(const std::atomic_bool & stop_requested, Config config);
  ~BluezBleClient();

  BluezBleClient(const BluezBleClient &) = delete;
  BluezBleClient & operator=(const BluezBleClient &) = delete;

  void connect();
  void write(const BleFrame & frame);
  void write_stop(const BleFrame & frame);
  void reset();
  void cancel();

  [[nodiscard]] const std::string & characteristic_path() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ble_hardware_bridge
