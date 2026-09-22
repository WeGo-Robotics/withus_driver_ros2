#include "withus_driver/withus_protocol.hpp"

#include <cstddef>
#include <cstdint>

namespace withus_driver
{
namespace
{
bool is_standard_data_frame(const CanFrame& frame, std::uint32_t expected_id)
{
    return frame.id == expected_id && frame.length == 8 &&
           !frame.is_extended && !frame.is_remote && !frame.is_error;
}

// Internal helpers: fixed, in-range offsets after DLC validation.
std::uint16_t read_u16_le(const CanFrame& frame, std::size_t offset)
{
    return static_cast<std::uint16_t>(
        frame.data[offset] | (frame.data[offset + 1] << 8)
    );
}

bool read_bit(const CanFrame& frame, std::size_t bit)
{
    return (frame.data[bit / 8] & (1U << (bit % 8))) != 0;
}

std::uint8_t read_message_counter(const CanFrame& frame)
{
    return static_cast<std::uint8_t>((frame.data[7] >> 4) & 0x0F);
}
} // namespace

bool decode_vehicle_diagnosis(const CanFrame& frame, VehicleDiagnosis& output)
{
    if (!is_standard_data_frame(frame, kVehicleDiagnosisId)) {
        return false;
    }
    // Start bit 22, length 10: byte 2 bits 6..7 followed by byte 3.
    const auto raw_voltage = static_cast<std::uint16_t>(
        (read_u16_le(frame, 2) >> 6) & 0x03FF
    );
    if (raw_voltage > 900) { // Physical range: 0..90 V.
        return false;
    }

    VehicleDiagnosis decoded{};
    decoded.battery_voltage_v = raw_voltage * 0.1;
    decoded.emergency_stop_pressed = read_bit(frame, 0);
    decoded.motor_fault = read_bit(frame, 1);
    decoded.remote_fault = read_bit(frame, 5);
    decoded.ad_signal_received = !read_bit(frame, 6); // 0 means received.
    decoded.dbs_fault = read_bit(frame, 7);
    decoded.parallel_driving_active = !read_bit(frame, 8); // 0 means active.
    decoded.bms_fault = read_bit(frame, 9);
    decoded.eps_fault = read_bit(frame, 10);
    decoded.front_bumper_triggered = read_bit(frame, 12);
    decoded.rear_bumper_triggered = read_bit(frame, 13);
    decoded.headlight_on = read_bit(frame, 15);
    decoded.left_indicator_on = read_bit(frame, 32);
    decoded.brake_light_on = read_bit(frame, 40);
    decoded.oil_level_abnormal = read_bit(frame, 41);
    decoded.horn_on = read_bit(frame, 43);
    decoded.right_indicator_on = read_bit(frame, 48);
    decoded.message_counter = read_message_counter(frame);
    output = decoded;
    return true;
}

bool decode_vehicle_status1(const CanFrame& frame, VehicleStatus1& output)
{
    if (!is_standard_data_frame(frame, kVehicleStatus1Id)) {
        return false;
    }
    const auto raw_requested_speed = read_u16_le(frame, 2);
    const auto raw_soc = frame.data[6];
    if (raw_requested_speed > 1600 || raw_soc > 100) {
        return false;
    }

    VehicleStatus1 decoded{};
    decoded.gear = static_cast<std::uint8_t>(frame.data[0] & 0x03);
    decoded.drive_mode = static_cast<std::uint8_t>(frame.data[1] & 0x0F);
    decoded.brake_hold_active = read_bit(frame, 3);
    decoded.requested_speed_mps = (raw_requested_speed * 0.1 - 80.0) / 3.6;
    decoded.battery_soc = raw_soc / 100.0; // 0..1, not percent.
    decoded.message_counter = read_message_counter(frame);
    output = decoded;
    return true;
}

bool decode_vehicle_status2(const CanFrame& frame, VehicleStatus2& output)
{
    if (!is_standard_data_frame(frame, kVehicleStatus2Id)) {
        return false;
    }
    const auto raw_speed = read_u16_le(frame, 0);
    const auto raw_pressure = read_u16_le(frame, 2);
    const auto raw_angle = static_cast<std::uint16_t>(
        read_u16_le(frame, 4) & 0x03FF
    );
    if (raw_speed > 1600 || raw_pressure > 800) {
        return false;
    }

    VehicleStatus2 decoded{};
    decoded.speed_mps = (raw_speed * 0.1 - 80.0) / 3.6;
    constexpr double kPi = 3.14159265358979323846;
    // Encoding: -35..67.3 deg; keep datasheet sign pending vehicle validation.
    decoded.steering_angle_rad = (raw_angle * 0.1 - 35.0) * kPi / 180.0;
    decoded.brake_pressure_pa = raw_pressure * 10000.0; // 0.01 MPa -> 10,000 Pa.
    decoded.message_counter = read_message_counter(frame);
    output = decoded;
    return true;
}
} // namespace withus_driver