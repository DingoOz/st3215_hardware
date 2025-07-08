#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "st3215_hardware/st3215_communication.hpp"

namespace st3215_hardware {

class ST3215Servo {
public:
    ST3215Servo(const std::string& port, uint8_t id);
    ~ST3215Servo() = default;
    
    ST3215Servo(const ST3215Servo&) = delete;
    ST3215Servo& operator=(const ST3215Servo&) = delete;
    ST3215Servo(ST3215Servo&&) = default;
    ST3215Servo& operator=(ST3215Servo&&) = default;
    
    bool connect();
    void disconnect();
    bool is_connected() const;
    
    bool set_position(double position_rad);
    bool set_velocity(double velocity_rad_s);
    bool set_effort(double effort_nm);
    
    std::optional<double> get_position() const;
    std::optional<double> get_velocity() const;
    std::optional<double> get_effort() const;
    std::optional<double> get_temperature() const;
    std::optional<double> get_voltage() const;
    
    bool ping() const;
    uint8_t get_id() const { return servo_id_; }
    
private:
    std::shared_ptr<ST3215Communication> comm_;
    uint8_t servo_id_;
    mutable std::mutex state_mutex_;
    
    std::optional<double> last_position_;
    std::optional<double> last_velocity_;
    std::optional<double> last_effort_;
    std::optional<double> last_temperature_;
    std::optional<double> last_voltage_;
};

}  // namespace st3215_hardware