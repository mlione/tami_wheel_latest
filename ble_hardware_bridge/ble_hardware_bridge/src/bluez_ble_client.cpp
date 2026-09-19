#include "ble_hardware_bridge/bluez_ble_client.hpp"

#include <gio/gio.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

namespace ble_hardware_bridge
{
namespace
{

constexpr auto kDiscoveryTimeout = std::chrono::seconds(10);
constexpr int kWriteTimeoutMs = 5000;
constexpr int kStopWriteTimeoutMs = 1000;

struct VariantDeleter
{
  void operator()(GVariant * value) const
  {
    if (value) {
      g_variant_unref(value);
    }
  }
};

struct ErrorDeleter
{
  void operator()(GError * error) const
  {
    if (error) {
      g_error_free(error);
    }
  }
};

struct ObjectDeleter
{
  template<typename T>
  void operator()(T * object) const
  {
    if (object) {
      g_object_unref(object);
    }
  }
};

using VariantPtr = std::unique_ptr<GVariant, VariantDeleter>;
using ErrorPtr = std::unique_ptr<GError, ErrorDeleter>;
using ConnectionPtr = std::unique_ptr<GDBusConnection, ObjectDeleter>;
using CancellablePtr = std::unique_ptr<GCancellable, ObjectDeleter>;

std::string lower_copy(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

bool belongs_to_device(
  std::string_view object_path,
  std::string_view device_path)
{
  return object_path.size() > device_path.size() &&
         object_path.compare(0, device_path.size(), device_path) == 0 &&
         object_path[device_path.size()] == '/';
}

} // namespace

class BluezBleClient::Impl
{
public:
  Impl(const std::atomic_bool & stop_requested, BluezBleClient::Config config)
  : stop_requested_(stop_requested), config_(std::move(config)),
    cancellable_(g_cancellable_new()) {}

  void connect()
  {
    reset();
    open_system_bus();
    set_adapter_powered();
    device_path_ = find_device_path();
    if (device_path_.empty()) {
      discover_device();
    }
    connect_device();
    wait_for_services();
    characteristic_path_ = find_characteristic_path();
  }

  void write(const BleFrame & frame)
  {
    write_value(frame, cancellable_.get(), kWriteTimeoutMs);
  }

  void write_stop(const BleFrame & frame)
  {
    // Shutdown cancels normal D-Bus work. The final stop must remain usable
    // after that cancellation, so it intentionally has no GCancellable.
    write_value(frame, nullptr, kStopWriteTimeoutMs);
  }

  void write_value(
    const BleFrame & frame, GCancellable * cancellable, int timeout_ms)
  {
    GVariantBuilder value;
    g_variant_builder_init(&value, G_VARIANT_TYPE("ay"));
    for (const auto byte : frame) {
      g_variant_builder_add(&value, "y", byte);
    }

    GVariantBuilder options;
    g_variant_builder_init(&options, G_VARIANT_TYPE("a{sv}"));
    g_variant_builder_add(
      &options, "{sv}", "type",
      g_variant_new_string("request"));

    call_with_cancellable(
      characteristic_path_, "org.bluez.GattCharacteristic1", "WriteValue",
      g_variant_new("(aya{sv})", &value, &options), nullptr, timeout_ms,
      cancellable);
  }

  void reset()
  {
    characteristic_path_.clear();
    device_path_.clear();
    bus_.reset();
  }

  void cancel() {g_cancellable_cancel(cancellable_.get());}

  const std::string & characteristic_path() const
  {
    return characteristic_path_;
  }

private:
  VariantPtr call(
    const std::string & path, const char * interface,
    const char * method, GVariant * parameters,
    const GVariantType * reply_type, int timeout_ms)
  {
    return call_with_cancellable(
      path, interface, method, parameters, reply_type, timeout_ms,
      cancellable_.get());
  }

  VariantPtr call_with_cancellable(
    const std::string & path, const char * interface,
    const char * method, GVariant * parameters,
    const GVariantType * reply_type, int timeout_ms,
    GCancellable * cancellable)
  {
    GError * raw_error = nullptr;
    VariantPtr reply(g_dbus_connection_call_sync(
        bus_.get(), "org.bluez", path.c_str(), interface, method, parameters,
        reply_type, G_DBUS_CALL_FLAGS_NONE, timeout_ms, cancellable,
        &raw_error));

    if (raw_error) {
      ErrorPtr error(raw_error);
      throw std::runtime_error(error->message);
    }
    if (!reply) {
      throw std::runtime_error("BlueZ D-Bus call returned no reply");
    }
    return reply;
  }

  void open_system_bus()
  {
    GError * raw_error = nullptr;
    bus_.reset(
      g_bus_get_sync(G_BUS_TYPE_SYSTEM, cancellable_.get(), &raw_error));
    if (raw_error) {
      ErrorPtr error(raw_error);
      throw std::runtime_error(
              std::string("failed to open system D-Bus: ") +
              error->message);
    }
    if (!bus_) {
      throw std::runtime_error("failed to open system D-Bus");
    }
  }

  void set_adapter_powered()
  {
    call(
      config_.adapter_path, "org.freedesktop.DBus.Properties", "Set",
      g_variant_new(
        "(ssv)", "org.bluez.Adapter1", "Powered",
        g_variant_new_boolean(TRUE)),
      nullptr, 5000);
  }

  VariantPtr managed_objects()
  {
    return call(
      "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects",
      nullptr, G_VARIANT_TYPE("(a{oa{sa{sv}}})"), 10000);
  }

  std::string find_device_path()
  {
    auto objects_reply = managed_objects();
    GVariantIter * objects = nullptr;
    g_variant_get(objects_reply.get(), "(a{oa{sa{sv}}})", &objects);

    const std::string target_address = lower_copy(config_.device_address);
    std::string found;
    gchar * object_path = nullptr;
    GVariant * interfaces = nullptr;

    while (g_variant_iter_next(
        objects, "{o@a{sa{sv}}}", &object_path,
        &interfaces))
    {
      VariantPtr interface_guard(interfaces);
      VariantPtr properties(g_variant_lookup_value(
          interfaces, "org.bluez.Device1", G_VARIANT_TYPE("a{sv}")));

      const char * address = nullptr;
      if (properties &&
        g_variant_lookup(properties.get(), "Address", "&s", &address) &&
        lower_copy(address) == target_address)
      {
        found = object_path;
      }

      g_free(object_path);
      if (!found.empty()) {
        break;
      }
    }

    g_variant_iter_free(objects);
    return found;
  }

  void discover_device()
  {
    bool discovery_started = false;
    try {
      call(
        config_.adapter_path, "org.bluez.Adapter1", "StartDiscovery",
        nullptr, nullptr, 5000);
      discovery_started = true;
    } catch (const std::exception &) {
      // Discovery may already be active; polling the object manager still
      // works.
    }

    const auto deadline =
      std::chrono::steady_clock::now() + kDiscoveryTimeout;
    while (!stop_requested_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
      device_path_ = find_device_path();
      if (!device_path_.empty()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    if (discovery_started) {
      try {
        call(
          config_.adapter_path, "org.bluez.Adapter1", "StopDiscovery",
          nullptr, nullptr, 5000);
      } catch (const std::exception &) {
        // Losing the adapter while stopping discovery is handled by the
        // reconnect loop.
      }
    }

    if (device_path_.empty()) {
      throw std::runtime_error(
              std::string("BLE device was not discovered: ") + config_.device_address);
    }
  }

  bool device_bool_property(const char * property)
  {
    auto reply = call(
      device_path_, "org.freedesktop.DBus.Properties", "Get",
      g_variant_new("(ss)", "org.bluez.Device1", property),
      G_VARIANT_TYPE("(v)"), 5000);

    GVariant * raw_value = nullptr;
    g_variant_get(reply.get(), "(v)", &raw_value);
    VariantPtr value(raw_value);
    return g_variant_get_boolean(value.get());
  }

  void connect_device()
  {
    if (device_bool_property("Connected")) {
      return;
    }

    try {
      call(
        device_path_, "org.bluez.Device1", "Connect", nullptr, nullptr,
        15000);
    } catch (const std::exception & error) {
      if (std::string_view(error.what()).find("AlreadyConnected") ==
        std::string_view::npos)
      {
        throw;
      }
    }
  }

  void wait_for_services()
  {
    const auto deadline =
      std::chrono::steady_clock::now() + kDiscoveryTimeout;
    while (!stop_requested_.load() &&
      std::chrono::steady_clock::now() < deadline)
    {
      if (device_bool_property("ServicesResolved")) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    throw std::runtime_error("timed out waiting for BLE GATT services");
  }

  std::string find_characteristic_path()
  {
    auto objects_reply = managed_objects();
    GVariantIter * objects = nullptr;
    g_variant_get(objects_reply.get(), "(a{oa{sa{sv}}})", &objects);

    std::string found;
    gchar * object_path = nullptr;
    GVariant * interfaces = nullptr;

    while (g_variant_iter_next(
        objects, "{o@a{sa{sv}}}", &object_path,
        &interfaces))
    {
      VariantPtr interface_guard(interfaces);
      if (belongs_to_device(object_path, device_path_)) {
        VariantPtr properties(
          g_variant_lookup_value(
            interfaces, "org.bluez.GattCharacteristic1",
            G_VARIANT_TYPE("a{sv}")));

        const char * uuid = nullptr;
        if (properties &&
          g_variant_lookup(properties.get(), "UUID", "&s", &uuid) &&
          lower_copy(uuid) == lower_copy(config_.characteristic_uuid))
        {
          found = object_path;
        }
      }

      g_free(object_path);
      if (!found.empty()) {
        break;
      }
    }

    g_variant_iter_free(objects);
    if (found.empty()) {
      throw std::runtime_error("configured GATT characteristic was not found");
    }
    return found;
  }

  const std::atomic_bool & stop_requested_;
  BluezBleClient::Config config_;
  ConnectionPtr bus_;
  CancellablePtr cancellable_;
  std::string device_path_;
  std::string characteristic_path_;
};

BluezBleClient::BluezBleClient(
  const std::atomic_bool & stop_requested, Config config)
: impl_(std::make_unique<Impl>(stop_requested, std::move(config))) {}

BluezBleClient::~BluezBleClient() = default;

void BluezBleClient::connect() {impl_->connect();}

void BluezBleClient::write(const BleFrame & frame) {impl_->write(frame);}

void BluezBleClient::write_stop(const BleFrame & frame) {impl_->write_stop(frame);}

void BluezBleClient::reset() {impl_->reset();}

void BluezBleClient::cancel() {impl_->cancel();}

const std::string & BluezBleClient::characteristic_path() const
{
  return impl_->characteristic_path();
}

} // namespace ble_hardware_bridge
