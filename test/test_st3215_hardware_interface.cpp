#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include <string>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "st3215_hardware/st3215_hardware_interface.hpp"

using namespace st3215_hardware;

class ST3215HardwareInterfaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        hardware_interface_ = std::make_unique<ST3215HardwareInterface>();
        
        hardware_info_.name = "st3215_hardware";
        hardware_info_.type = "st3215_hardware/ST3215HardwareInterface";
        
        hardware_info_.hardware_parameters["port"] = "/dev/null";
        hardware_info_.hardware_parameters["baudrate"] = "1000000";
        hardware_info_.hardware_parameters["timeout_ms"] = "100";
        
        hardware_interface::ComponentInfo joint_info;
        joint_info.name = "joint_1";
        joint_info.type = "joint";
        joint_info.parameters["servo_id"] = "1";
        
        hardware_interface::InterfaceInfo position_cmd;
        position_cmd.name = "position";
        joint_info.command_interfaces.push_back(position_cmd);
        
        hardware_interface::InterfaceInfo velocity_cmd;
        velocity_cmd.name = "velocity";
        joint_info.command_interfaces.push_back(velocity_cmd);
        
        hardware_interface::InterfaceInfo effort_cmd;
        effort_cmd.name = "effort";
        joint_info.command_interfaces.push_back(effort_cmd);
        
        hardware_interface::InterfaceInfo position_state;
        position_state.name = "position";
        joint_info.state_interfaces.push_back(position_state);
        
        hardware_interface::InterfaceInfo velocity_state;
        velocity_state.name = "velocity";
        joint_info.state_interfaces.push_back(velocity_state);
        
        hardware_interface::InterfaceInfo effort_state;
        effort_state.name = "effort";
        joint_info.state_interfaces.push_back(effort_state);
        
        hardware_interface::InterfaceInfo temperature_state;
        temperature_state.name = "temperature";
        joint_info.state_interfaces.push_back(temperature_state);
        
        hardware_interface::InterfaceInfo voltage_state;
        voltage_state.name = "voltage";
        joint_info.state_interfaces.push_back(voltage_state);
        
        hardware_info_.joints.push_back(joint_info);
    }

    void TearDown() override {
        hardware_interface_.reset();
        rclcpp::shutdown();
    }

    std::unique_ptr<ST3215HardwareInterface> hardware_interface_;
    hardware_interface::HardwareInfo hardware_info_;
};

TEST_F(ST3215HardwareInterfaceTest, InitializationTest) {
    auto result = hardware_interface_->on_init(hardware_info_);
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(ST3215HardwareInterfaceTest, ConfigurationTest) {
    hardware_interface_->on_init(hardware_info_);
    
    rclcpp_lifecycle::State dummy_state;
    auto result = hardware_interface_->on_configure(dummy_state);
    EXPECT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(ST3215HardwareInterfaceTest, StateInterfacesTest) {
    hardware_interface_->on_init(hardware_info_);
    
    auto state_interfaces = hardware_interface_->export_state_interfaces();
    EXPECT_EQ(state_interfaces.size(), 5);
    
    auto interface_names = std::vector<std::string>();
    for (const auto& interface : state_interfaces) {
        interface_names.push_back(interface.get_interface_name());
    }
    
    EXPECT_THAT(interface_names, ::testing::UnorderedElementsAre(
        "position", "velocity", "effort", "temperature", "voltage"
    ));
}

TEST_F(ST3215HardwareInterfaceTest, CommandInterfacesTest) {
    hardware_interface_->on_init(hardware_info_);
    
    auto command_interfaces = hardware_interface_->export_command_interfaces();
    EXPECT_EQ(command_interfaces.size(), 3);
    
    auto interface_names = std::vector<std::string>();
    for (const auto& interface : command_interfaces) {
        interface_names.push_back(interface.get_interface_name());
    }
    
    EXPECT_THAT(interface_names, ::testing::UnorderedElementsAre(
        "position", "velocity", "effort"
    ));
}

TEST_F(ST3215HardwareInterfaceTest, ReadWriteOperationsTest) {
    hardware_interface_->on_init(hardware_info_);
    
    rclcpp_lifecycle::State dummy_state;
    hardware_interface_->on_configure(dummy_state);
    
    rclcpp::Time dummy_time;
    rclcpp::Duration dummy_duration(0, 0);
    
    auto read_result = hardware_interface_->read(dummy_time, dummy_duration);
    EXPECT_EQ(read_result, hardware_interface::CallbackReturn::ERROR);
    
    auto write_result = hardware_interface_->write(dummy_time, dummy_duration);
    EXPECT_EQ(write_result, hardware_interface::CallbackReturn::ERROR);
}

TEST_F(ST3215HardwareInterfaceTest, LifecycleTest) {
    hardware_interface_->on_init(hardware_info_);
    
    rclcpp_lifecycle::State dummy_state;
    
    auto configure_result = hardware_interface_->on_configure(dummy_state);
    EXPECT_EQ(configure_result, hardware_interface::CallbackReturn::SUCCESS);
    
    auto activate_result = hardware_interface_->on_activate(dummy_state);
    EXPECT_EQ(activate_result, hardware_interface::CallbackReturn::ERROR);
    
    auto deactivate_result = hardware_interface_->on_deactivate(dummy_state);
    EXPECT_EQ(deactivate_result, hardware_interface::CallbackReturn::SUCCESS);
    
    auto cleanup_result = hardware_interface_->on_cleanup(dummy_state);
    EXPECT_EQ(cleanup_result, hardware_interface::CallbackReturn::SUCCESS);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}