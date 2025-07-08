#include "st3215_hardware/st3215_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace st3215_hardware {

ST3215HardwareInterface::ST3215HardwareInterface()
    : logger_(rclcpp::get_logger("st3215_hardware_interface")) {
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    port_ = info_.hardware_parameters.at("port");
    baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
    timeout_ = std::chrono::milliseconds(std::stoi(info_.hardware_parameters.at("timeout_ms")));
    
    servos_.resize(info_.joints.size());
    
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto& joint_info = info_.joints[i];
        auto& servo_data = servos_[i];
        
        if (!parse_servo_config(joint_info, servo_data)) {
            RCLCPP_ERROR(logger_, "Failed to parse servo configuration for joint %s", joint_info.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(logger_, "Initialized servo %d for joint %s", servo_data.id, servo_data.name.c_str());
    }
    
    RCLCPP_INFO(logger_, "Successfully initialized ST3215 hardware interface with %zu servos", servos_.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Configuring ST3215 hardware interface");
    
    if (!initialize_servos()) {
        RCLCPP_ERROR(logger_, "Failed to initialize servos");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(logger_, "Successfully configured ST3215 hardware interface");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Activating ST3215 hardware interface");
    
    if (!connect_servos()) {
        RCLCPP_ERROR(logger_, "Failed to connect to servos");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (!read_servo_states()) {
        RCLCPP_WARN(logger_, "Failed to read initial servo states");
    }
    
    RCLCPP_INFO(logger_, "Successfully activated ST3215 hardware interface");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Deactivating ST3215 hardware interface");
    
    disconnect_servos();
    
    RCLCPP_INFO(logger_, "Successfully deactivated ST3215 hardware interface");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215HardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(logger_, "Cleaning up ST3215 hardware interface");
    
    disconnect_servos();
    servos_.clear();
    
    RCLCPP_INFO(logger_, "Successfully cleaned up ST3215 hardware interface");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ST3215HardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (auto& servo : servos_) {
        state_interfaces.emplace_back(servo.name, hardware_interface::HW_IF_POSITION, &servo.position_state);
        state_interfaces.emplace_back(servo.name, hardware_interface::HW_IF_VELOCITY, &servo.velocity_state);
        state_interfaces.emplace_back(servo.name, hardware_interface::HW_IF_EFFORT, &servo.effort_state);
        state_interfaces.emplace_back(servo.name, "temperature", &servo.temperature_state);
        state_interfaces.emplace_back(servo.name, "voltage", &servo.voltage_state);
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ST3215HardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (auto& servo : servos_) {
        command_interfaces.emplace_back(servo.name, hardware_interface::HW_IF_POSITION, &servo.position_command);
        command_interfaces.emplace_back(servo.name, hardware_interface::HW_IF_VELOCITY, &servo.velocity_command);
        command_interfaces.emplace_back(servo.name, hardware_interface::HW_IF_EFFORT, &servo.effort_command);
    }
    
    return command_interfaces;
}

hardware_interface::return_type ST3215HardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (!read_servo_states()) {
        RCLCPP_WARN(logger_, "Failed to read servo states");
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ST3215HardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    if (!write_servo_commands()) {
        RCLCPP_WARN(logger_, "Failed to write servo commands");
        return hardware_interface::return_type::ERROR;
    }
    
    return hardware_interface::return_type::OK;
}

bool ST3215HardwareInterface::parse_servo_config(const hardware_interface::ComponentInfo& joint_info, ServoData& servo_data) {
    servo_data.name = joint_info.name;
    
    auto servo_id_it = joint_info.parameters.find("servo_id");
    if (servo_id_it == joint_info.parameters.end()) {
        RCLCPP_ERROR(logger_, "Missing servo_id parameter for joint %s", joint_info.name.c_str());
        return false;
    }
    
    try {
        servo_data.id = static_cast<uint8_t>(std::stoi(servo_id_it->second));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Invalid servo_id parameter for joint %s: %s", joint_info.name.c_str(), e.what());
        return false;
    }
    
    return true;
}

bool ST3215HardwareInterface::initialize_servos() {
    bool success = true;
    
    for (auto& servo : servos_) {
        try {
            servo.servo = std::make_unique<ST3215Servo>(port_, servo.id);
            RCLCPP_INFO(logger_, "Initialized servo %d for joint %s", servo.id, servo.name.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to initialize servo %d for joint %s: %s", servo.id, servo.name.c_str(), e.what());
            success = false;
        }
    }
    
    return success;
}

bool ST3215HardwareInterface::connect_servos() {
    bool success = true;
    
    for (auto& servo : servos_) {
        if (!servo.servo->connect()) {
            RCLCPP_ERROR(logger_, "Failed to connect to servo %d for joint %s", servo.id, servo.name.c_str());
            success = false;
        } else {
            RCLCPP_INFO(logger_, "Connected to servo %d for joint %s", servo.id, servo.name.c_str());
        }
    }
    
    return success;
}

void ST3215HardwareInterface::disconnect_servos() {
    for (auto& servo : servos_) {
        if (servo.servo) {
            servo.servo->disconnect();
            RCLCPP_INFO(logger_, "Disconnected from servo %d for joint %s", servo.id, servo.name.c_str());
        }
    }
}

bool ST3215HardwareInterface::read_servo_states() {
    bool success = true;
    
    for (auto& servo : servos_) {
        if (!servo.servo || !servo.servo->is_connected()) {
            continue;
        }
        
        auto position = servo.servo->get_position();
        if (position) {
            servo.position_state = *position;
        } else {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Failed to read position from servo %d", servo.id);
            success = false;
        }
        
        auto velocity = servo.servo->get_velocity();
        if (velocity) {
            servo.velocity_state = *velocity;
        } else {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Failed to read velocity from servo %d", servo.id);
            success = false;
        }
        
        auto effort = servo.servo->get_effort();
        if (effort) {
            servo.effort_state = *effort;
        } else {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Failed to read effort from servo %d", servo.id);
            success = false;
        }
        
        auto temperature = servo.servo->get_temperature();
        if (temperature) {
            servo.temperature_state = *temperature;
        }
        
        auto voltage = servo.servo->get_voltage();
        if (voltage) {
            servo.voltage_state = *voltage;
        }
    }
    
    return success;
}

bool ST3215HardwareInterface::write_servo_commands() {
    bool success = true;
    
    for (auto& servo : servos_) {
        if (!servo.servo || !servo.servo->is_connected()) {
            continue;
        }
        
        static constexpr double POSITION_TOLERANCE = 0.001;
        static constexpr double VELOCITY_TOLERANCE = 0.001;
        static constexpr double EFFORT_TOLERANCE = 0.001;
        
        if (std::abs(servo.position_command - servo.position_state) > POSITION_TOLERANCE) {
            if (!servo.servo->set_position(servo.position_command)) {
                RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Failed to set position for servo %d", servo.id);
                success = false;
            }
        }
        
        if (std::abs(servo.velocity_command - servo.velocity_state) > VELOCITY_TOLERANCE) {
            if (!servo.servo->set_velocity(servo.velocity_command)) {
                RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Failed to set velocity for servo %d", servo.id);
                success = false;
            }
        }
        
        if (std::abs(servo.effort_command - servo.effort_state) > EFFORT_TOLERANCE) {
            if (!servo.servo->set_effort(servo.effort_command)) {
                RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 1000, "Failed to set effort for servo %d", servo.id);
                success = false;
            }
        }
    }
    
    return success;
}

}  // namespace st3215_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(st3215_hardware::ST3215HardwareInterface, hardware_interface::SystemInterface)