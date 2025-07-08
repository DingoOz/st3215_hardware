#include "st3215_hardware/st3215_communication.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace st3215_hardware {

ST3215Communication::ST3215Communication(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), logger_(rclcpp::get_logger("st3215_communication")) {
    serial_port_ = std::make_unique<LibSerial::SerialPort>();
}

ST3215Communication::~ST3215Communication() {
    disconnect();
}

bool ST3215Communication::connect() {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    try {
        serial_port_->Open(port_);
        serial_port_->SetBaudRate(static_cast<LibSerial::BaudRate>(baudrate_));
        serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_->SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        
        connected_ = true;
        RCLCPP_INFO(logger_, "Connected to ST3215 on port %s at %d baud", port_.c_str(), baudrate_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to connect to ST3215: %s", e.what());
        connected_ = false;
        return false;
    }
}

void ST3215Communication::disconnect() {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    if (serial_port_->IsOpen()) {
        serial_port_->Close();
    }
    connected_ = false;
    RCLCPP_INFO(logger_, "Disconnected from ST3215");
}

bool ST3215Communication::is_connected() const {
    return connected_;
}

std::optional<double> ST3215Communication::read_position(uint8_t servo_id) {
    auto position = read_register_u16(servo_id, Register::POSITION_L);
    if (position) {
        return position_to_radians(*position);
    }
    return std::nullopt;
}

std::optional<double> ST3215Communication::read_velocity(uint8_t servo_id) {
    auto velocity = read_register_u16(servo_id, Register::VELOCITY_L);
    if (velocity) {
        return velocity_to_rad_per_sec(*velocity);
    }
    return std::nullopt;
}

std::optional<double> ST3215Communication::read_effort(uint8_t servo_id) {
    auto effort = read_register_u16(servo_id, Register::EFFORT_L);
    if (effort) {
        return effort_to_nm(*effort);
    }
    return std::nullopt;
}

std::optional<double> ST3215Communication::read_temperature(uint8_t servo_id) {
    auto temp = read_register_u8(servo_id, Register::TEMPERATURE);
    if (temp) {
        return temperature_to_celsius(*temp);
    }
    return std::nullopt;
}

std::optional<double> ST3215Communication::read_voltage(uint8_t servo_id) {
    auto voltage = read_register_u8(servo_id, Register::VOLTAGE);
    if (voltage) {
        return voltage_to_volts(*voltage);
    }
    return std::nullopt;
}

bool ST3215Communication::write_position(uint8_t servo_id, double position_rad) {
    uint16_t position = radians_to_position(position_rad);
    return write_register_u16(servo_id, Register::POSITION_L, position);
}

bool ST3215Communication::write_velocity(uint8_t servo_id, double velocity_rad_s) {
    uint16_t velocity = rad_per_sec_to_velocity(velocity_rad_s);
    return write_register_u16(servo_id, Register::VELOCITY_L, velocity);
}

bool ST3215Communication::write_effort(uint8_t servo_id, double effort_nm) {
    uint16_t effort = nm_to_effort(effort_nm);
    return write_register_u16(servo_id, Register::EFFORT_L, effort);
}

bool ST3215Communication::ping_servo(uint8_t servo_id) {
    Packet packet;
    packet.servo_id = servo_id;
    packet.length = 2;
    packet.instruction = static_cast<uint8_t>(Command::PING);
    packet.checksum = calculate_checksum(packet);
    
    if (!send_packet(packet)) {
        return false;
    }
    
    auto response = receive_packet();
    return response.has_value() && response->servo_id == servo_id;
}

bool ST3215Communication::set_timeout(std::chrono::milliseconds timeout) {
    timeout_ = timeout;
    return true;
}

bool ST3215Communication::send_packet(const Packet& packet) {
    if (!connected_) {
        return false;
    }
    
    std::vector<uint8_t> buffer;
    buffer.push_back(packet.header1);
    buffer.push_back(packet.header2);
    buffer.push_back(packet.servo_id);
    buffer.push_back(packet.length);
    buffer.push_back(packet.instruction);
    
    for (const auto& param : packet.parameters) {
        buffer.push_back(param);
    }
    
    buffer.push_back(packet.checksum);
    
    try {
        serial_port_->Write(buffer);
        serial_port_->DrainWriteBuffer();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to send packet: %s", e.what());
        return false;
    }
}

std::optional<ST3215Communication::Packet> ST3215Communication::receive_packet() {
    if (!connected_) {
        return std::nullopt;
    }
    
    try {
        Packet packet;
        std::vector<uint8_t> buffer(1);
        
        auto timeout_ms = static_cast<int32_t>(timeout_.count());
        
        serial_port_->ReadByte(buffer[0], timeout_ms);
        if (buffer[0] != 0xFF) return std::nullopt;
        packet.header1 = buffer[0];
        
        serial_port_->ReadByte(buffer[0], timeout_ms);
        if (buffer[0] != 0xFF) return std::nullopt;
        packet.header2 = buffer[0];
        
        serial_port_->ReadByte(packet.servo_id, timeout_ms);
        serial_port_->ReadByte(packet.length, timeout_ms);
        serial_port_->ReadByte(packet.instruction, timeout_ms);
        
        size_t param_count = packet.length - 2;
        packet.parameters.resize(param_count);
        
        for (size_t i = 0; i < param_count; ++i) {
            serial_port_->ReadByte(packet.parameters[i], timeout_ms);
        }
        
        serial_port_->ReadByte(packet.checksum, timeout_ms);
        
        if (!validate_checksum(packet)) {
            RCLCPP_WARN(logger_, "Invalid checksum in received packet");
            return std::nullopt;
        }
        
        return packet;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to receive packet: %s", e.what());
        return std::nullopt;
    }
}

uint8_t ST3215Communication::calculate_checksum(const Packet& packet) const {
    uint8_t checksum = packet.servo_id + packet.length + packet.instruction;
    for (const auto& param : packet.parameters) {
        checksum += param;
    }
    return ~checksum;
}

bool ST3215Communication::validate_checksum(const Packet& packet) const {
    return calculate_checksum(packet) == packet.checksum;
}

ST3215Communication::Packet ST3215Communication::create_read_packet(uint8_t servo_id, Register reg, uint8_t length) {
    Packet packet;
    packet.servo_id = servo_id;
    packet.length = 4;
    packet.instruction = static_cast<uint8_t>(Command::READ_DATA);
    packet.parameters = {static_cast<uint8_t>(reg), length};
    packet.checksum = calculate_checksum(packet);
    return packet;
}

ST3215Communication::Packet ST3215Communication::create_write_packet(uint8_t servo_id, Register reg, const std::vector<uint8_t>& data) {
    Packet packet;
    packet.servo_id = servo_id;
    packet.length = static_cast<uint8_t>(3 + data.size());
    packet.instruction = static_cast<uint8_t>(Command::WRITE_DATA);
    packet.parameters.push_back(static_cast<uint8_t>(reg));
    packet.parameters.insert(packet.parameters.end(), data.begin(), data.end());
    packet.checksum = calculate_checksum(packet);
    return packet;
}

std::optional<uint16_t> ST3215Communication::read_register_u16(uint8_t servo_id, Register reg) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    auto packet = create_read_packet(servo_id, reg, 2);
    if (!send_packet(packet)) {
        return std::nullopt;
    }
    
    auto response = receive_packet();
    if (!response || response->parameters.size() < 2) {
        return std::nullopt;
    }
    
    uint16_t value = response->parameters[0] | (response->parameters[1] << 8);
    return value;
}

std::optional<uint8_t> ST3215Communication::read_register_u8(uint8_t servo_id, Register reg) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    auto packet = create_read_packet(servo_id, reg, 1);
    if (!send_packet(packet)) {
        return std::nullopt;
    }
    
    auto response = receive_packet();
    if (!response || response->parameters.empty()) {
        return std::nullopt;
    }
    
    return response->parameters[0];
}

bool ST3215Communication::write_register_u16(uint8_t servo_id, Register reg, uint16_t value) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    
    std::vector<uint8_t> data = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };
    
    auto packet = create_write_packet(servo_id, reg, data);
    return send_packet(packet);
}

double ST3215Communication::position_to_radians(uint16_t position) const {
    return (static_cast<double>(position) / 4095.0) * 2.0 * M_PI;
}

double ST3215Communication::velocity_to_rad_per_sec(uint16_t velocity) const {
    return (static_cast<double>(velocity) / 4095.0) * 10.0;
}

double ST3215Communication::effort_to_nm(uint16_t effort) const {
    return (static_cast<double>(effort) / 4095.0) * 20.0;
}

double ST3215Communication::temperature_to_celsius(uint8_t temp) const {
    return static_cast<double>(temp);
}

double ST3215Communication::voltage_to_volts(uint8_t voltage) const {
    return static_cast<double>(voltage) / 10.0;
}

uint16_t ST3215Communication::radians_to_position(double radians) const {
    return static_cast<uint16_t>((radians / (2.0 * M_PI)) * 4095.0);
}

uint16_t ST3215Communication::rad_per_sec_to_velocity(double rad_per_sec) const {
    return static_cast<uint16_t>((rad_per_sec / 10.0) * 4095.0);
}

uint16_t ST3215Communication::nm_to_effort(double nm) const {
    return static_cast<uint16_t>((nm / 20.0) * 4095.0);
}

}  // namespace st3215_hardware