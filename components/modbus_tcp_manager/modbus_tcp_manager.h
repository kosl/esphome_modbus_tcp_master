#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <string>
#include <vector>
#include <memory>

#ifdef USE_ESP32
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#endif

namespace esphome {
namespace modbus_tcp {

static const char *const TAG = "modbus_tcp_manager";

enum class ModbusFunction : uint8_t {
    READ_COILS = 0x01,
    READ_DISCRETE_INPUTS = 0x02,  
    READ_HOLDING_REGISTERS = 0x03,
    READ_INPUT_REGISTERS = 0x04,
    WRITE_SINGLE_COIL = 0x05,
    WRITE_SINGLE_REGISTER = 0x06,
    WRITE_MULTIPLE_COILS = 0x0F,
    WRITE_MULTIPLE_REGISTERS = 0x10
};

struct ModbusResponse {
    bool success;
    std::vector<uint16_t> data;
    std::string error_message;
};

class ModbusTCPManager : public Component {
public:
    ModbusTCPManager(const std::string &host, uint16_t port, uint8_t unit_id) 
        : host_(host), port_(port), unit_id_(unit_id), 
          is_connected_(false), last_connection_attempt_(0),
          watchdog_register_(0), watchdog_enabled_(false), 
          watchdog_interval_(10000), last_watchdog_time_(0),
          watchdog_counter_(0), safe_mode_active_(false),
          connection_check_state_(ConnectionCheckState::IDLE),
          connection_check_sock_(-1), connection_check_start_time_(0),
          connection_check_success_(false),
          persistent_sock_(-1), last_activity_(0),
          connection_timeout_(5000), read_timeout_(5000) {}  // 5 second timeouts

    void setup() override {
        ESP_LOGD(TAG, "Setting up Modbus TCP Manager for %s:%d", host_.c_str(), port_);
        ESP_LOGI(TAG, "Using persistent connections with %dms timeouts", read_timeout_);
    }

    void loop() override {
        uint32_t now = millis();
        
        // Close persistent connection if idle for too long (30 seconds)
        if (persistent_sock_ >= 0 && (now - last_activity_) > 30000) {
            ESP_LOGD(TAG, "Closing idle persistent connection");
            close_persistent_connection();
        }
        
        // Non-blocking connection health check - keep 10 second interval for slow devices
        if (now - last_connection_attempt_ > 10000) {
            last_connection_attempt_ = now;
            start_connection_check();
        }
        
        // Process connection check state machine
        process_connection_check();
        
        // Watchdog handling
        if (watchdog_enabled_ && now - last_watchdog_time_ > watchdog_interval_) {
            handle_watchdog();
        }
        
        // Yield regularly for responsiveness
        if (now % 20 == 0) {  // Less frequent yielding
            yield();
        }
    }

    float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

    // Configuration methods
    void set_watchdog_register(uint16_t reg) { 
        watchdog_register_ = reg; 
        watchdog_enabled_ = true;
        ESP_LOGD(TAG, "Watchdog enabled on register %d", reg);
    }
    
    void set_watchdog_interval(uint32_t interval) { 
        watchdog_interval_ = interval; 
    }
    
    void add_safe_mode_register(uint16_t reg, int16_t value) {
        safe_mode_registers_.push_back({reg, value});
        ESP_LOGD(TAG, "Added safe mode: register %d = %d", reg, value);
    }

    // Set custom timeouts for slow devices
    void set_connection_timeout(uint32_t timeout_ms) { 
        connection_timeout_ = timeout_ms; 
        ESP_LOGD(TAG, "Connection timeout set to %dms", timeout_ms);
    }
    
    void set_read_timeout(uint32_t timeout_ms) { 
        read_timeout_ = timeout_ms; 
        ESP_LOGD(TAG, "Read timeout set to %dms", timeout_ms);
    }

    // Connection status
    bool is_connected() const { return is_connected_; }
    
    // Force connection status update (used by sensors)
    void mark_connection_failed() { 
        is_connected_ = false;
        close_persistent_connection(); // Close on failure
    }

    // Public connection check method
    void check_connection() {
        if (connection_check_state_ != ConnectionCheckState::IDLE) {
            return;
        }
        start_connection_check();
    }

    // Read single register using persistent connection
    ModbusResponse read_register(uint16_t address, ModbusFunction function = ModbusFunction::READ_HOLDING_REGISTERS) {
        return read_registers(address, 1, function);
    }

    // Read multiple registers using persistent connection
    ModbusResponse read_registers(uint16_t start_address, uint16_t count, ModbusFunction function = ModbusFunction::READ_HOLDING_REGISTERS) {
        ModbusResponse response;
        response.success = false;

        // Try to ensure we have a persistent connection
        if (!ensure_persistent_connection()) {
            response.error_message = "Connection failed";
            is_connected_ = false;
            return response;
        }

        std::vector<uint8_t> request = build_read_request(start_address, count, function);
        
        // Send request using persistent connection
        if (!send_data_persistent(request)) {
            ESP_LOGW(TAG, "Send failed, reconnecting...");
            close_persistent_connection();
            
            // Try once more with fresh connection
            if (!ensure_persistent_connection()) {
                response.error_message = "Reconnection failed";
                is_connected_ = false;
                return response;
            }
            
            if (!send_data_persistent(request)) {
                response.error_message = "Send failed after reconnect";
                is_connected_ = false;
                close_persistent_connection();
                return response;
            }
        }

        // Receive response using persistent connection
        std::vector<uint8_t> resp_data = receive_data_persistent();
        
        if (resp_data.empty()) {
            ESP_LOGW(TAG, "Receive failed, closing connection");
            response.error_message = "Receive failed";
            is_connected_ = false;
            close_persistent_connection();
            return response;
        }

        if (!parse_read_response(resp_data, response, function)) {
            is_connected_ = false;
            return response;
        }

        last_activity_ = millis();
        is_connected_ = true;
        response.success = true;
        return response;
    }

    // Write single register using persistent connection
    bool write_register(uint16_t address, int16_t value) {
        ESP_LOGD(TAG, "Writing value %d to register %d", value, address);
        
        if (!ensure_persistent_connection()) {
            is_connected_ = false;
            return false;
        }

        std::vector<uint8_t> request = build_write_request(address, value);
        
        bool success = send_data_persistent(request);
        if (success) {
            std::vector<uint8_t> response = receive_data_persistent();
            success = !response.empty() && response.size() >= 8;
        }
        
        if (!success) {
            close_persistent_connection();
            is_connected_ = false;
        } else {
            last_activity_ = millis();
            is_connected_ = true;
        }
        
        if (success) {
            ESP_LOGD(TAG, "Successfully wrote value %d to register %d", value, address);
        } else {
            ESP_LOGW(TAG, "Failed to write to register %d", address);
        }
        
        return success;
    }

    // Write multiple registers using persistent connection
    bool write_registers(uint16_t start_address, const std::vector<int16_t>& values) {
        ESP_LOGD(TAG, "Writing %d values starting at register %d", values.size(), start_address);
        
        if (values.empty() || values.size() > 123) {
            ESP_LOGE(TAG, "Invalid value count: %d", values.size());
            return false;
        }

        if (!ensure_persistent_connection()) {
            is_connected_ = false;
            return false;
        }

        std::vector<uint8_t> request = build_write_multiple_request(start_address, values);
        
        bool success = send_data_persistent(request);
        if (success) {
            std::vector<uint8_t> response = receive_data_persistent();
            success = !response.empty() && response.size() >= 8;
        }
        
        if (!success) {
            close_persistent_connection();
            is_connected_ = false;
        } else {
            last_activity_ = millis();
            is_connected_ = true;
        }
        
        if (success) {
            ESP_LOGD(TAG, "Successfully wrote %d values starting at register %d", values.size(), start_address);
        } else {
            ESP_LOGW(TAG, "Failed to write multiple registers starting at %d", start_address);
        }
        
        return success;
    }

private:
    std::string host_;
    uint16_t port_;
    uint8_t unit_id_;
    bool is_connected_;
    uint32_t last_connection_attempt_;
    uint16_t transaction_id_ = 1;
    
    // Persistent connection variables
    int persistent_sock_;
    uint32_t last_activity_;
    uint32_t connection_timeout_;
    uint32_t read_timeout_;
    
    // Watchdog variables
    uint16_t watchdog_register_;
    bool watchdog_enabled_;
    uint32_t watchdog_interval_;
    uint32_t last_watchdog_time_;
    uint16_t watchdog_counter_;
    bool safe_mode_active_;
    
    // Non-blocking connection check state machine
    enum class ConnectionCheckState {
        IDLE,
        CONNECTING,
        CLEANUP
    };
    ConnectionCheckState connection_check_state_;
    int connection_check_sock_;
    uint32_t connection_check_start_time_;
    bool connection_check_success_;
    
    // Safe mode configuration
    struct SafeModeRegister {
        uint16_t register_addr;
        int16_t value;
    };
    std::vector<SafeModeRegister> safe_mode_registers_;

    // Ensure persistent connection is available
    bool ensure_persistent_connection() {
        if (persistent_sock_ >= 0) {
            // Test if connection is still alive with a quick check
            if (is_socket_connected(persistent_sock_)) {
                return true;
            } else {
                ESP_LOGD(TAG, "Persistent connection dead, reconnecting");
                close_persistent_connection();
            }
        }
        
        // Create new persistent connection
        ESP_LOGD(TAG, "Creating persistent connection to %s:%d", host_.c_str(), port_);
        persistent_sock_ = create_connection_with_timeout();
        if (persistent_sock_ >= 0) {
            last_activity_ = millis();
            ESP_LOGD(TAG, "Persistent connection established");
            return true;
        }
        
        ESP_LOGW(TAG, "Failed to create persistent connection");
        return false;
    }

    void close_persistent_connection() {
        if (persistent_sock_ >= 0) {
            ::close(persistent_sock_);
            persistent_sock_ = -1;
            ESP_LOGV(TAG, "Persistent connection closed");
        }
    }

    // Check if socket is still connected
    bool is_socket_connected(int sock) {
        if (sock < 0) return false;
        
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(sock, &read_fds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 1000; // 1ms quick check
        
        int result = ::select(sock + 1, &read_fds, nullptr, nullptr, &timeout);
        
        if (result > 0 && FD_ISSET(sock, &read_fds)) {
            // Socket has data or is closed
            char test_buf[1];
            int peek_result = ::recv(sock, test_buf, 1, MSG_PEEK | MSG_DONTWAIT);
            return peek_result != 0; // 0 means connection closed
        }
        
        return result == 0; // Timeout means connection is fine
    }

    // Send data using persistent connection with longer timeout
    bool send_data_persistent(const std::vector<uint8_t>& data) {
        if (persistent_sock_ < 0) return false;
        
        int sent = ::send(persistent_sock_, data.data(), data.size(), 0);
        if (sent != (int)data.size()) {
            ESP_LOGV(TAG, "Persistent send failed: %d/%d bytes", sent, data.size());
            return false;
        }
        return true;
    }

    // Receive data using persistent connection with longer timeout
    std::vector<uint8_t> receive_data_persistent() {
        std::vector<uint8_t> data;
        if (persistent_sock_ < 0) return data;
        
        uint8_t buffer[256];
        
        // Use select for timeout control
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(persistent_sock_, &read_fds);
        
        struct timeval timeout;
        timeout.tv_sec = read_timeout_ / 1000;
        timeout.tv_usec = (read_timeout_ % 1000) * 1000;
        
        int select_result = ::select(persistent_sock_ + 1, &read_fds, nullptr, nullptr, &timeout);
        
        if (select_result > 0 && FD_ISSET(persistent_sock_, &read_fds)) {
            int len = ::recv(persistent_sock_, buffer, sizeof(buffer), 0);
            if (len > 0) {
                data.assign(buffer, buffer + len);
                ESP_LOGVV(TAG, "Received %d bytes on persistent connection", len);
            } else if (len == 0) {
                ESP_LOGV(TAG, "Persistent connection closed by remote");
            } else {
                ESP_LOGV(TAG, "Persistent receive error: %d", errno);
            }
        } else if (select_result == 0) {
            ESP_LOGV(TAG, "Persistent receive timeout after %dms", read_timeout_);
        } else {
            ESP_LOGV(TAG, "Persistent select error: %d", errno);
        }
        
        return data;
    }

    // Create connection with configurable timeout
    int create_connection_with_timeout() {
        int sock = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            ESP_LOGV(TAG, "Could not create socket: %d", errno);
            return -1;
        }

        // Set socket to non-blocking mode for connection
        int flags = ::fcntl(sock, F_GETFL, 0);
        ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        // Set longer timeouts for slow devices
        struct timeval timeout;
        timeout.tv_sec = read_timeout_ / 1000;
        timeout.tv_usec = (read_timeout_ % 1000) * 1000;
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        
        if (::inet_aton(host_.c_str(), &server_addr.sin_addr) == 0) {
            struct hostent *he = ::gethostbyname(host_.c_str());
            if (he == nullptr) {
                ESP_LOGV(TAG, "DNS resolution failed: %s", host_.c_str());
                ::close(sock);
                return -1;
            }
            memcpy(&server_addr.sin_addr, he->h_addr, sizeof(server_addr.sin_addr));
        }

        // Non-blocking connect with configurable timeout
        int connect_result = ::connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (connect_result < 0) {
            if (errno == EINPROGRESS) {
                // Connection in progress, wait with select()
                fd_set write_fds;
                FD_ZERO(&write_fds);
                FD_SET(sock, &write_fds);
                
                struct timeval connect_timeout;
                connect_timeout.tv_sec = connection_timeout_ / 1000;
                connect_timeout.tv_usec = (connection_timeout_ % 1000) * 1000;
                
                int select_result = ::select(sock + 1, nullptr, &write_fds, nullptr, &connect_timeout);
                if (select_result <= 0) {
                    ESP_LOGV(TAG, "Connection timeout to %s:%d after %dms", host_.c_str(), port_, connection_timeout_);
                    ::close(sock);
                    return -1;
                }
                
                // Check if connection actually succeeded
                int error = 0;
                socklen_t len = sizeof(error);
                ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
                if (error != 0) {
                    ESP_LOGV(TAG, "Connection failed to %s:%d (error: %d)", host_.c_str(), port_, error);
                    ::close(sock);
                    return -1;
                }
            } else {
                ESP_LOGV(TAG, "Immediate connection failure to %s:%d", host_.c_str(), port_);
                ::close(sock);
                return -1;
            }
        }

        // Set back to blocking mode for data transfer
        ::fcntl(sock, F_SETFL, flags);

        ESP_LOGD(TAG, "Connected to %s:%d with %dms timeout", host_.c_str(), port_, read_timeout_);
        return sock;
    }

    // Rest of the methods remain the same...
    void start_connection_check() {
        if (connection_check_state_ != ConnectionCheckState::IDLE) {
            return;
        }
        
        ESP_LOGV(TAG, "Starting non-blocking connection check");
        connection_check_state_ = ConnectionCheckState::CONNECTING;
        connection_check_start_time_ = millis();
        connection_check_success_ = false;
        
        connection_check_sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (connection_check_sock_ < 0) {
            ESP_LOGV(TAG, "Could not create socket for connection check");
            connection_check_state_ = ConnectionCheckState::CLEANUP;
            return;
        }
        
        int flags = ::fcntl(connection_check_sock_, F_GETFL, 0);
        ::fcntl(connection_check_sock_, F_SETFL, flags | O_NONBLOCK);
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        ::inet_aton(host_.c_str(), &server_addr.sin_addr);
        
        int result = ::connect(connection_check_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (result == 0) {
            connection_check_success_ = true;
            connection_check_state_ = ConnectionCheckState::CLEANUP;
        } else if (errno != EINPROGRESS) {
            ESP_LOGV(TAG, "Connection check failed immediately");
            connection_check_state_ = ConnectionCheckState::CLEANUP;
        }
    }
    
    void process_connection_check() {
        uint32_t now = millis();
        
        switch (connection_check_state_) {
            case ConnectionCheckState::IDLE:
                return;
                
            case ConnectionCheckState::CONNECTING: {
                fd_set write_fds, error_fds;
                FD_ZERO(&write_fds);
                FD_ZERO(&error_fds);
                FD_SET(connection_check_sock_, &write_fds);
                FD_SET(connection_check_sock_, &error_fds);
                
                struct timeval timeout;
                timeout.tv_sec = 0;
                timeout.tv_usec = 5000;  // 5ms check
                
                int select_result = ::select(connection_check_sock_ + 1, nullptr, &write_fds, &error_fds, &timeout);
                
                if (select_result > 0) {
                    if (FD_ISSET(connection_check_sock_, &error_fds)) {
                        ESP_LOGV(TAG, "Connection check failed (error fd set)");
                        connection_check_success_ = false;
                        connection_check_state_ = ConnectionCheckState::CLEANUP;
                    } else if (FD_ISSET(connection_check_sock_, &write_fds)) {
                        int error = 0;
                        socklen_t len = sizeof(error);
                        ::getsockopt(connection_check_sock_, SOL_SOCKET, SO_ERROR, &error, &len);
                        
                        if (error == 0) {
                            ESP_LOGV(TAG, "Connection check succeeded");
                            connection_check_success_ = true;
                        } else {
                            ESP_LOGV(TAG, "Connection check failed (socket error: %d)", error);
                            connection_check_success_ = false;
                        }
                        connection_check_state_ = ConnectionCheckState::CLEANUP;
                    }
                } else if (now - connection_check_start_time_ > connection_timeout_) {
                    ESP_LOGV(TAG, "Connection check timeout");
                    connection_check_success_ = false;
                    connection_check_state_ = ConnectionCheckState::CLEANUP;
                }
                break;
            }
            
            case ConnectionCheckState::CLEANUP: {
                if (connection_check_sock_ >= 0) {
                    ::close(connection_check_sock_);
                    connection_check_sock_ = -1;
                }
                
                if (connection_check_success_) {
                    if (!is_connected_) {
                        ESP_LOGI(TAG, "Modbus connection restored to %s:%d", host_.c_str(), port_);
                        is_connected_ = true;
                    }
                } else {
                    if (is_connected_) {
                        ESP_LOGW(TAG, "Modbus connection lost to %s:%d", host_.c_str(), port_);
                        is_connected_ = false;
                        close_persistent_connection();
                    }
                }
                
                connection_check_state_ = ConnectionCheckState::IDLE;
                break;
            }
        }
    }

    void handle_watchdog() {
        last_watchdog_time_ = millis();
        
        if (!is_connected_) {
            ESP_LOGW(TAG, "Watchdog: Connection lost, activating safe mode");
            activate_safe_mode();
            return;
        }
        
        watchdog_counter_++;
        bool write_success = write_register(watchdog_register_, watchdog_counter_);
        
        if (write_success) {
            delay(100);
            ModbusResponse response = read_register(watchdog_register_);
            
            if (response.success && !response.data.empty()) {
                uint16_t read_value = response.data[0];
                
                if (read_value != watchdog_counter_) {
                    ESP_LOGD(TAG, "Watchdog OK: wrote %d, read %d", watchdog_counter_, read_value);
                    watchdog_counter_ = read_value;
                    
                    if (safe_mode_active_) {
                        ESP_LOGI(TAG, "Watchdog restored, deactivating safe mode");
                        safe_mode_active_ = false;
                    }
                } else {
                    ESP_LOGW(TAG, "Watchdog failed: remote device not responding");
                    activate_safe_mode();
                }
            } else {
                ESP_LOGW(TAG, "Watchdog read failed");
                activate_safe_mode();
            }
        } else {
            ESP_LOGW(TAG, "Watchdog write failed");
            activate_safe_mode();
        }
    }
    
    void activate_safe_mode() {
        if (safe_mode_active_) return;
        
        ESP_LOGW(TAG, "Activating safe mode - writing %d safe values", safe_mode_registers_.size());
        safe_mode_active_ = true;
        
        for (const auto& safe_reg : safe_mode_registers_) {
            write_register(safe_reg.register_addr, safe_reg.value);
            ESP_LOGI(TAG, "Safe mode: Set register %d = %d", safe_reg.register_addr, safe_reg.value);
        }
    }

    std::vector<uint8_t> build_read_request(uint16_t address, uint16_t count, ModbusFunction function) {
        return {
            static_cast<uint8_t>((transaction_id_ >> 8) & 0xFF),
            static_cast<uint8_t>(transaction_id_++ & 0xFF),
            0x00, 0x00,
            0x00, 0x06,
            unit_id_,
            static_cast<uint8_t>(function),
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((count >> 8) & 0xFF),
            static_cast<uint8_t>(count & 0xFF)
        };
    }

    std::vector<uint8_t> build_write_request(uint16_t address, int16_t value) {
        return {
            static_cast<uint8_t>((transaction_id_ >> 8) & 0xFF),
            static_cast<uint8_t>(transaction_id_++ & 0xFF),
            0x00, 0x00,
            0x00, 0x06,
            unit_id_,
            0x06,
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF)
        };
    }

    std::vector<uint8_t> build_write_multiple_request(uint16_t address, const std::vector<int16_t>& values) {
        uint16_t count = values.size();
        uint8_t byte_count = count * 2;
        
        std::vector<uint8_t> request = {
            static_cast<uint8_t>((transaction_id_ >> 8) & 0xFF),
            static_cast<uint8_t>(transaction_id_++ & 0xFF),
            0x00, 0x00,
            static_cast<uint8_t>(((7 + byte_count) >> 8) & 0xFF),
            static_cast<uint8_t>((7 + byte_count) & 0xFF),
            unit_id_,
            0x10,
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((count >> 8) & 0xFF),
            static_cast<uint8_t>(count & 0xFF),
            byte_count
        };
        
        for (int16_t value : values) {
            request.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
            request.push_back(static_cast<uint8_t>(value & 0xFF));
        }
        
        return request;
    }

    bool parse_read_response(const std::vector<uint8_t>& data, ModbusResponse& response, ModbusFunction function) {
        if (data.size() < 9) {
            response.error_message = "Response too short";
            return false;
        }

        if (data[7] != static_cast<uint8_t>(function)) {
            response.error_message = "Invalid function code";
            return false;
        }

        uint8_t byte_count = data[8];
        if (data.size() < 9 + byte_count) {
            response.error_message = "Incomplete response";
            return false;
        }

        response.data.clear();
        for (int i = 0; i < byte_count; i += 2) {
            uint16_t value = (data[9 + i] << 8) | data[9 + i + 1];
            response.data.push_back(value);
        }

        return true;
    }
};

// Forward declaration for binary_sensor::BinarySensor
namespace binary_sensor {
    class BinarySensor;
}

// Sensor class
class ModbusTCPSensor : public PollingComponent, public sensor::Sensor {
public:
    ModbusTCPSensor(ModbusTCPManager *parent, uint16_t register_address, 
                    uint8_t function_code, float scale, float offset, uint32_t update_interval) 
        : parent_(parent), register_address_(register_address), 
          function_code_(function_code), scale_(scale), offset_(offset) {
        this->set_update_interval(update_interval);
    }

    void setup() override {
        ESP_LOGD(TAG, "Setting up Modbus sensor for register %d", register_address_);
    }

    void update() override {
        if (!parent_->is_connected()) {
            ESP_LOGV(TAG, "Triggering connection check for register %d", register_address_);
            const_cast<ModbusTCPManager*>(parent_)->check_connection();
            
            if (!parent_->is_connected()) {
                ESP_LOGV(TAG, "Modbus still not connected, skipping update");
                return;
            }
        }

        // Reduced rate limiting for better responsiveness with persistent connections
        static uint32_t last_any_update = 0;
        uint32_t now = millis();
        
        if (now - last_any_update < 50) {  // Reduced from 200ms to 50ms
            ESP_LOGV(TAG, "Rate limiting sensor %d, skipping update", register_address_);
            return;
        }
        last_any_update = now;

        ModbusFunction func = (function_code_ == 4) ? 
            ModbusFunction::READ_INPUT_REGISTERS : 
            ModbusFunction::READ_HOLDING_REGISTERS;

        ESP_LOGD(TAG, "Reading register %d", register_address_);
        ModbusResponse response = parent_->read_register(register_address_, func);
        
        if (response.success && !response.data.empty()) {
            int16_t raw_value = static_cast<int16_t>(response.data[0]);
            float scaled_value = (raw_value * scale_) + offset_;
            
            ESP_LOGD(TAG, "Register %d: raw=%d, scaled=%.2f", register_address_, raw_value, scaled_value);
            this->publish_state(scaled_value);
        } else {
            ESP_LOGW(TAG, "Failed to read register %d: %s", register_address_, response.error_message.c_str());
            const_cast<ModbusTCPManager*>(parent_)->mark_connection_failed();
        }
        
        yield();
    }

private:
    ModbusTCPManager *parent_;
    uint16_t register_address_;
    uint8_t function_code_;
    float scale_;
    float offset_;
};

// Connection status sensor - separate class that gets defined in binary_sensor.cpp
class ModbusTCPConnectionSensor;

}  // namespace modbus_tcp
}  // namespace esphome
