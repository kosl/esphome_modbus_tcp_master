#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <string>
#include <vector>
#include <memory>
#include <queue>

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

// **FIXED**: Use function pointer instead of std::function to avoid memory issues
typedef void (*ModbusCallback)(const ModbusResponse&);

// **FIXED**: Simpler request structure without std::function
struct ModbusRequest {
    uint16_t start_address;
    uint16_t count;
    ModbusFunction function;
    ModbusCallback callback;
    uint32_t timestamp;
    uint16_t transaction_id;
    bool is_write;
    int16_t write_value;  // Only single value for simplicity
};

class ModbusTCPManager : public Component {
public:
    ModbusTCPManager(const std::string &host, uint16_t port, uint8_t unit_id) 
        : host_(host), port_(port), unit_id_(unit_id), 
          is_connected_(false), last_connection_attempt_(0),
          connection_state_(ConnectionState::DISCONNECTED),
          socket_(-1), transaction_id_(1),
          last_activity_(0), connection_timeout_(3000), read_timeout_(3000),
          current_request_(), pending_request_active_(false),
          loop_counter_(0), last_yield_time_(0) {}

    void setup() override {
        ESP_LOGD(TAG, "Setting up FIXED Non-Blocking Modbus TCP Manager for %s:%d", host_.c_str(), port_);
        ESP_LOGI(TAG, "Using FIXED async architecture with %dms timeout", read_timeout_);
    }

    // **FIXED**: Ultra-short loop with strict time limits
    void loop() override {
        uint32_t now = millis();
        uint32_t loop_start = micros();
	uint32_t p1, p2, p3, p4;
        
        // **CRITICAL**: Yield frequently to prevent WiFi starvation
        if (++loop_counter_ >= 5) {  // Yield every 5 loops instead of 10
            loop_counter_ = 0;
            yield();
            last_yield_time_ = now;
        }
        
        // **CRITICAL**: If we haven't yielded in 50ms, force yield
        if (now - last_yield_time_ > 50) {
            yield();
            last_yield_time_ = now;
        }
        
        // Process connection state machine (max 1ms per call)
        if ((p1=(micros() - loop_start)) < 1000) {
            process_connection_state_machine();
        }
        
        // Process pending requests (max 1ms per call)
        if ((p2=(micros() - loop_start)) < 2000) {
            process_pending_requests();
        }
        
        // Cleanup timeouts (max 500μs per call)
        if ((p3=(micros() - loop_start)) < 2500) {
            cleanup_timeouts();
        }
        
        // **CRITICAL**: Total loop time must be under 3ms
        if ((p4=(micros() - loop_start)) > 3000) {
            ESP_LOGW(TAG, "Loop took too long: %d μs (%d, %d, %d, %d)",
		     (micros() - loop_start), p1, p2, p3, p4);
        }
    }

    float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

    // Connection status
    bool is_connected() const { return is_connected_; }
    
    void mark_connection_failed() { 
        is_connected_ = false;
        close_connection();
    }

    // **FIXED**: Async read with proper memory management
    void read_registers_async(uint16_t start_address, uint16_t count, 
                             ModbusFunction function, ModbusCallback callback) {
        // **CRITICAL**: Check queue size to prevent memory bloat
        if (request_queue_.size() > 5) {
            ESP_LOGW(TAG, "Request queue full, dropping request");
            ModbusResponse error_response;
            error_response.success = false;
            error_response.error_message = "Queue full";
            callback(error_response);
            return;
        }
        
        ModbusRequest request;
        request.start_address = start_address;
        request.count = count;
        request.function = function;
        request.callback = callback;
        request.timestamp = millis();
        request.transaction_id = transaction_id_++;
        request.is_write = false;
        request.write_value = 0;
        
        request_queue_.push(request);
        ESP_LOGV(TAG, "Queued async read: addr=%d, count=%d", start_address, count);
    }

    // **FIXED**: Async write with proper memory management
    void write_register_async(uint16_t address, int16_t value, ModbusCallback callback) {
        if (request_queue_.size() > 5) {
            ESP_LOGW(TAG, "Request queue full, dropping write request");
            ModbusResponse error_response;
            error_response.success = false;
            error_response.error_message = "Queue full";
            callback(error_response);
            return;
        }
        
        ModbusRequest request;
        request.start_address = address;
        request.count = 1;
        request.function = ModbusFunction::WRITE_SINGLE_REGISTER;
        request.callback = callback;
        request.timestamp = millis();
        request.transaction_id = transaction_id_++;
        request.is_write = true;
        request.write_value = value;
        
        request_queue_.push(request);
        ESP_LOGV(TAG, "Queued async write: addr=%d, value=%d", address, value);
    }

    // **REMOVED**: No more blocking backward compatibility - forces async usage
    // This prevents accidental blocking calls

    // Configuration methods
    void set_connection_timeout(uint32_t timeout_ms) { 
        connection_timeout_ = timeout_ms; 
    }
    
    void set_read_timeout(uint32_t timeout_ms) { 
        read_timeout_ = timeout_ms; 
    }

private:
    std::string host_;
    uint16_t port_;
    uint8_t unit_id_;
    bool is_connected_;
    uint32_t last_connection_attempt_;
    uint16_t transaction_id_;
    
    // Connection timeouts
    uint32_t connection_timeout_;
    uint32_t read_timeout_;
    uint32_t last_activity_;
    
    // **SIMPLIFIED**: Async state machine
    enum class ConnectionState {
        DISCONNECTED,
        CONNECTING,
        CONNECTED,
        SENDING,
        RECEIVING,
        ERROR_RECOVERY
    };
    ConnectionState connection_state_;
    int socket_;
    
    // **FIXED**: Request queue and processing
    std::queue<ModbusRequest> request_queue_;
    ModbusRequest current_request_;
    bool pending_request_active_;
    std::vector<uint8_t> send_buffer_;
    std::vector<uint8_t> receive_buffer_;
    uint32_t pending_request_start_time_;
    
    // **ADDED**: Loop timing controls
    uint8_t loop_counter_;
    uint32_t last_yield_time_;

    // **ULTRA-FAST**: Connection state machine (max 1ms)
    void process_connection_state_machine() {
        uint32_t now = millis();

        switch (connection_state_) {
            case ConnectionState::DISCONNECTED: {
                // Try to connect every 5 seconds
                if (now - last_connection_attempt_ > 5000) {
                    last_connection_attempt_ = now;
                    start_connection();
                }
                break;
            }
            case ConnectionState::CONNECTING: {
                check_connection_progress();
                break;
            }
            case ConnectionState::CONNECTED: {
                is_connected_ = true;
                break;
            }
            case ConnectionState::SENDING: {
                continue_sending();
                break;
            }
            case ConnectionState::RECEIVING: {
                continue_receiving();
                break;
            }
            case ConnectionState::ERROR_RECOVERY: {
                close_connection();
                connection_state_ = ConnectionState::DISCONNECTED;
                break;
            }
        }
    }

    // **ULTRA-FAST**: Request processing (max 1ms)
    void process_pending_requests() {
        if (connection_state_ != ConnectionState::CONNECTED || request_queue_.empty()) {
            return;
        }
        
        if (!pending_request_active_) {
            // Start processing next request
            current_request_ = request_queue_.front();
            request_queue_.pop();
            pending_request_active_ = true;
            
            // Build request packet
            if (current_request_.is_write) {
                send_buffer_ = build_write_request(current_request_.start_address, 
                                                  current_request_.write_value);
            } else {
                send_buffer_ = build_read_request(current_request_.start_address,
                                                 current_request_.count,
                                                 current_request_.function);
            }
            
            pending_request_start_time_ = millis();
            connection_state_ = ConnectionState::SENDING;
            ESP_LOGV(TAG, "Started processing request for addr %d", current_request_.start_address);
        }
    }

    // **ULTRA-FAST**: Timeout cleanup (max 500μs)
    void cleanup_timeouts() {
        uint32_t now = millis();
        
        // Check for request timeout
        if (pending_request_active_ && 
            (now - pending_request_start_time_) > read_timeout_) {
            
            ESP_LOGW(TAG, "Request timeout after %dms", read_timeout_);
            
            // Call callback with timeout error
            ModbusResponse timeout_response;
            timeout_response.success = false;
            timeout_response.error_message = "Request timeout";
            
            if (current_request_.callback) {
                current_request_.callback(timeout_response);
            }
            
            pending_request_active_ = false;
            connection_state_ = ConnectionState::ERROR_RECOVERY;
        }
        
        // **SIMPLIFIED**: Just clear old requests without callbacks to save time
        while (!request_queue_.empty() && 
               (now - request_queue_.front().timestamp) > (read_timeout_ * 2)) {
            request_queue_.pop();
        }
    }

    void start_connection() {
        ESP_LOGD(TAG, "Starting non-blocking connection to %s:%d", host_.c_str(), port_);
        
        socket_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (socket_ < 0) {
            ESP_LOGW(TAG, "Socket creation failed");
            connection_state_ = ConnectionState::ERROR_RECOVERY;
            return;
        }
        
        // Set to non-blocking mode
        int flags = ::fcntl(socket_, F_GETFL, 0);
        ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK);
        
        // **ULTRA-SHORT** timeouts to prevent blocking
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;  // 10ms timeout!
        ::setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        
        if (::inet_aton(host_.c_str(), &server_addr.sin_addr) == 0) {
            ESP_LOGW(TAG, "Invalid IP address: %s", host_.c_str());
            close_connection();
            connection_state_ = ConnectionState::ERROR_RECOVERY;
            return;
        }
        
        int result = ::connect(socket_, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (result == 0) {
            connection_state_ = ConnectionState::CONNECTED;
            ESP_LOGD(TAG, "Immediate connection success");
        } else if (errno == EINPROGRESS) {
            connection_state_ = ConnectionState::CONNECTING;
            ESP_LOGV(TAG, "Connection in progress...");
        } else {
            ESP_LOGW(TAG, "Connection failed immediately");
            close_connection();
            connection_state_ = ConnectionState::ERROR_RECOVERY;
        }
    }

    void check_connection_progress() {
        fd_set write_fds, error_fds;
        FD_ZERO(&write_fds);
        FD_ZERO(&error_fds);
        FD_SET(socket_, &write_fds);
        FD_SET(socket_, &error_fds);
        
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100;  // 100μs check - ultra fast!
        
        int result = ::select(socket_ + 1, nullptr, &write_fds, &error_fds, &timeout);
        
        if (result > 0) {
            if (FD_ISSET(socket_, &error_fds)) {
                ESP_LOGW(TAG, "Connection failed");
                close_connection();
                connection_state_ = ConnectionState::ERROR_RECOVERY;
            } else if (FD_ISSET(socket_, &write_fds)) {
                ESP_LOGI(TAG, "Connection established");
                connection_state_ = ConnectionState::CONNECTED;
            }
        } else if (millis() - last_connection_attempt_ > connection_timeout_) {
            ESP_LOGW(TAG, "Connection timeout");
            close_connection();
            connection_state_ = ConnectionState::ERROR_RECOVERY;
        }
    }

    void continue_sending() {
        if (send_buffer_.empty()) {
            receive_buffer_.clear();
            connection_state_ = ConnectionState::RECEIVING;
            return;
        }
        
        int sent = ::send(socket_, send_buffer_.data(), send_buffer_.size(), MSG_DONTWAIT);
        if (sent > 0) {
            ESP_LOGVV(TAG, "Sent %d bytes", sent);
            send_buffer_.clear();
            receive_buffer_.clear();
            connection_state_ = ConnectionState::RECEIVING;
        } else if (sent < 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            ESP_LOGW(TAG, "Send error: %d", errno);
            connection_state_ = ConnectionState::ERROR_RECOVERY;
        }
    }

    void continue_receiving() {
        uint8_t buffer[32];  // Smaller buffer for faster processing
        int received = ::recv(socket_, buffer, sizeof(buffer), MSG_DONTWAIT);
        
        if (received > 0) {
            receive_buffer_.insert(receive_buffer_.end(), buffer, buffer + received);
            ESP_LOGVV(TAG, "Received %d bytes (total: %d)", received, receive_buffer_.size());
            
            if (is_response_complete()) {
                process_response();
            }
        } else if (received == 0) {
            ESP_LOGW(TAG, "Connection closed by remote");
            connection_state_ = ConnectionState::ERROR_RECOVERY;
        } else if (errno != EWOULDBLOCK && errno != EAGAIN) {
            ESP_LOGW(TAG, "Receive error: %d", errno);
            connection_state_ = ConnectionState::ERROR_RECOVERY;
        }
    }

    bool is_response_complete() {
        if (receive_buffer_.size() < 8) return false;
        
        if (receive_buffer_.size() >= 9) {
            uint8_t byte_count = receive_buffer_[8];
            return receive_buffer_.size() >= (9 + byte_count);
        }
        
        return false;
    }

    void process_response() {
        ESP_LOGV(TAG, "Processing complete response (%d bytes)", receive_buffer_.size());
        
        ModbusResponse response;
        if (parse_read_response(receive_buffer_, response, current_request_.function)) {
            response.success = true;
            ESP_LOGD(TAG, "Request completed successfully");
        } else {
            response.success = false;
            ESP_LOGW(TAG, "Response parsing failed: %s", response.error_message.c_str());
        }
        
        // Call the callback
        if (current_request_.callback) {
            current_request_.callback(response);
        }
        
        // Cleanup
        pending_request_active_ = false;
        connection_state_ = ConnectionState::CONNECTED;
        last_activity_ = millis();
    }

    void close_connection() {
        if (socket_ >= 0) {
            ::close(socket_);
            socket_ = -1;
        }
        is_connected_ = false;
    }

    // Helper methods
    std::vector<uint8_t> build_read_request(uint16_t address, uint16_t count, ModbusFunction function) {
        return {
            static_cast<uint8_t>((transaction_id_ >> 8) & 0xFF),
            static_cast<uint8_t>(transaction_id_ & 0xFF),
            0x00, 0x00, 0x00, 0x06,
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
            static_cast<uint8_t>(transaction_id_ & 0xFF),
            0x00, 0x00, 0x00, 0x06,
            unit_id_, 0x06,
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF)
        };
    }

    bool parse_read_response(const std::vector<uint8_t>& data, ModbusResponse& response, ModbusFunction function) {
        if (data.size() < 9) {
            response.error_message = "Response too short";
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

}  // namespace modbus_tcp
}  // namespace esphome
