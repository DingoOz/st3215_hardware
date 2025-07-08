#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>

namespace st3215_hardware {

class ST3215Communication {
public:
    explicit ST3215Communication(const std::string& port, int baudrate = 1000000);
    ~ST3215Communication();

    ST3215Communication(const ST3215Communication&) = delete;
    ST3215Communication& operator=(const ST3215Communication&) = delete;
    ST3215Communication(ST3215Communication&&) = delete;
    ST3215Communication& operator=(ST3215Communication&&) = delete;

    bool connect();
    void disconnect();
    bool is_connected() const;

    std::optional<double> read_position(uint8_t servo_id);
    std::optional<double> read_velocity(uint8_t servo_id);
    std::optional<double> read_effort(uint8_t servo_id);
    std::optional<double> read_temperature(uint8_t servo_id);
    std::optional<double> read_voltage(uint8_t servo_id);

    bool write_position(uint8_t servo_id, double position_rad);
    bool write_velocity(uint8_t servo_id, double velocity_rad_s);
    bool write_effort(uint8_t servo_id, double effort_nm);

    bool ping_servo(uint8_t servo_id);
    bool set_timeout(std::chrono::milliseconds timeout);

private:
    enum class Command : uint8_t {
        PING = 0x01,
        READ_DATA = 0x02,
        WRITE_DATA = 0x03,
        REG_WRITE = 0x04,
        ACTION = 0x05,
        RESET = 0x06,
        SYNC_WRITE = 0x83
    };

    enum class Register : uint8_t {
        POSITION_L = 0x38,
        POSITION_H = 0x39,
        VELOCITY_L = 0x3A,
        VELOCITY_H = 0x3B,
        EFFORT_L = 0x3C,
        EFFORT_H = 0x3D,
        TEMPERATURE = 0x3E,
        VOLTAGE = 0x3F
    };

    struct Packet {
        uint8_t header1{0xFF};
        uint8_t header2{0xFF};
        uint8_t servo_id{0};
        uint8_t length{0};
        uint8_t instruction{0};
        std::vector<uint8_t> parameters;
        uint8_t checksum{0};
    };

    std::string port_;
    int baudrate_;
    std::unique_ptr<LibSerial::SerialPort> serial_port_;
    std::chrono::milliseconds timeout_{100};
    mutable std::mutex serial_mutex_;
    std::atomic<bool> connected_{false};
    rclcpp::Logger logger_;

    bool send_packet(const Packet& packet);
    std::optional<Packet> receive_packet();
    uint8_t calculate_checksum(const Packet& packet) const;
    bool validate_checksum(const Packet& packet) const;
    
    Packet create_read_packet(uint8_t servo_id, Register reg, uint8_t length = 2);
    Packet create_write_packet(uint8_t servo_id, Register reg, const std::vector<uint8_t>& data);
    
    std::optional<uint16_t> read_register_u16(uint8_t servo_id, Register reg);
    std::optional<uint8_t> read_register_u8(uint8_t servo_id, Register reg);
    bool write_register_u16(uint8_t servo_id, Register reg, uint16_t value);
    
    double position_to_radians(uint16_t position) const;
    double velocity_to_rad_per_sec(uint16_t velocity) const;
    double effort_to_nm(uint16_t effort) const;
    double temperature_to_celsius(uint8_t temp) const;
    double voltage_to_volts(uint8_t voltage) const;
    
    uint16_t radians_to_position(double radians) const;
    uint16_t rad_per_sec_to_velocity(double rad_per_sec) const;
    uint16_t nm_to_effort(double nm) const;
};

}  // namespace st3215_hardware