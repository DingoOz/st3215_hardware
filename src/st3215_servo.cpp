#include "st3215_hardware/st3215_servo.hpp"

namespace st3215_hardware {

ST3215Servo::ST3215Servo(const std::string& port, uint8_t id)
    : servo_id_(id) {
    comm_ = std::make_shared<ST3215Communication>(port);
}

bool ST3215Servo::connect() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!comm_->connect()) {
        return false;
    }
    
    return ping();
}

void ST3215Servo::disconnect() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    comm_->disconnect();
}

bool ST3215Servo::is_connected() const {
    return comm_->is_connected();
}

bool ST3215Servo::set_position(double position_rad) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return false;
    }
    
    if (comm_->write_position(servo_id_, position_rad)) {
        last_position_ = position_rad;
        return true;
    }
    return false;
}

bool ST3215Servo::set_velocity(double velocity_rad_s) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return false;
    }
    
    if (comm_->write_velocity(servo_id_, velocity_rad_s)) {
        last_velocity_ = velocity_rad_s;
        return true;
    }
    return false;
}

bool ST3215Servo::set_effort(double effort_nm) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return false;
    }
    
    if (comm_->write_effort(servo_id_, effort_nm)) {
        last_effort_ = effort_nm;
        return true;
    }
    return false;
}

std::optional<double> ST3215Servo::get_position() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return last_position_;
    }
    
    auto position = comm_->read_position(servo_id_);
    if (position) {
        const_cast<ST3215Servo*>(this)->last_position_ = *position;
    }
    return position ? *position : last_position_;
}

std::optional<double> ST3215Servo::get_velocity() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return last_velocity_;
    }
    
    auto velocity = comm_->read_velocity(servo_id_);
    if (velocity) {
        const_cast<ST3215Servo*>(this)->last_velocity_ = *velocity;
    }
    return velocity ? *velocity : last_velocity_;
}

std::optional<double> ST3215Servo::get_effort() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return last_effort_;
    }
    
    auto effort = comm_->read_effort(servo_id_);
    if (effort) {
        const_cast<ST3215Servo*>(this)->last_effort_ = *effort;
    }
    return effort ? *effort : last_effort_;
}

std::optional<double> ST3215Servo::get_temperature() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return last_temperature_;
    }
    
    auto temperature = comm_->read_temperature(servo_id_);
    if (temperature) {
        const_cast<ST3215Servo*>(this)->last_temperature_ = *temperature;
    }
    return temperature ? *temperature : last_temperature_;
}

std::optional<double> ST3215Servo::get_voltage() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_connected()) {
        return last_voltage_;
    }
    
    auto voltage = comm_->read_voltage(servo_id_);
    if (voltage) {
        const_cast<ST3215Servo*>(this)->last_voltage_ = *voltage;
    }
    return voltage ? *voltage : last_voltage_;
}

bool ST3215Servo::ping() const {
    if (!is_connected()) {
        return false;
    }
    
    return comm_->ping_servo(servo_id_);
}

}  // namespace st3215_hardware