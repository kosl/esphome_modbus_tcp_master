#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/log.h"
#include <string>

namespace esphome {
namespace modbus_tcp {

static const char *const TAG = "modbus_tcp_manager";

// Forward declaration of the main manager class
class ModbusTCPManager {
public:
    bool is_connected() const;
};

// Connection status sensor implementation
class ModbusTCPConnectionSensor : public PollingComponent, public binary_sensor::BinarySensor {
public:
    ModbusTCPConnectionSensor(ModbusTCPManager *parent) : parent_(parent) {
        this->set_update_interval(1000);  // Check every 1 second for faster response
    }

    void setup() override {
        ESP_LOGD(TAG, "Setting up Modbus connection status sensor");
    }

    void update() override {
        bool connected = parent_->is_connected();
        this->publish_state(connected);
        ESP_LOGV(TAG, "Modbus connection status: %s", connected ? "Connected" : "Disconnected");
    }

private:
    ModbusTCPManager *parent_;
};

}  // namespace modbus_tcp
}  // namespace esphome
