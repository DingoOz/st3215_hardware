#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "st3215_hardware/st3215_servo.hpp"

namespace st3215_hardware {

class ST3215HardwareInterface : public hardware_interface::SystemInterface {
public:
    ST3215HardwareInterface();
    ~ST3215HardwareInterface() override = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    struct ServoData {
        uint8_t id;
        std::string name;
        std::unique_ptr<ST3215Servo> servo;
        
        double position_state = 0.0;
        double velocity_state = 0.0;
        double effort_state = 0.0;
        double temperature_state = 0.0;
        double voltage_state = 0.0;
        
        double position_command = 0.0;
        double velocity_command = 0.0;
        double effort_command = 0.0;
        
        bool position_command_changed = false;
        bool velocity_command_changed = false;
        bool effort_command_changed = false;
    };
    
    std::string port_;
    int baudrate_;
    std::chrono::milliseconds timeout_;
    std::vector<ServoData> servos_;
    
    rclcpp::Logger logger_;
    
    bool parse_servo_config(const hardware_interface::ComponentInfo& joint_info, ServoData& servo_data);
    bool initialize_servos();
    bool connect_servos();
    void disconnect_servos();
    bool read_servo_states();
    bool write_servo_commands();
};

}  // namespace st3215_hardware