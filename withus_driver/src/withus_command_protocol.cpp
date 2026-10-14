#include "withus_driver/withus_protocol.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace withus_driver
{
namespace
{
bool in_range(double value, double minimum, double maximum)
{
    return std::isfinite(value) && value >= minimum && value <= maximum;
}

CanFrame make_command_frame(std::uint32_t id, std::uint8_t counter)
{
    CanFrame frame{};
    frame.id = id;
    frame.length = 8;
    frame.data[0] = static_cast<std::uint8_t>(counter << 4);
    return frame;
}

// Internal helper: callers use fixed offsets 4 and 6 in an 8-byte frame.
void write_u16_le(CanFrame& frame, std::size_t offset, std::uint16_t value)
{
    frame.data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    frame.data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}
} // namespace

bool encode_control_request(
    const ControlRequest& command, std::uint8_t counter, CanFrame& output)
{
    if (counter > 15) {
        return false;
    }

    auto encoded = make_command_frame(kAdControlFlagId, counter);
    encoded.data[0] |= command.request_control ? 1U : 0U;
    output = encoded;
    return true;
}

bool encode_steering_command(
    const SteeringCommand& command, std::uint8_t counter, CanFrame& output)
{
    if (counter > 15 ||
        !in_range(command.steering_angle_rad,
                  -kMaxCommandSteeringRad, kMaxCommandSteeringRad)) {
        return false;
    }

    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const double angle_deg = command.steering_angle_rad * kRadToDeg;
    const auto raw_angle = static_cast<std::uint16_t>(
        std::lround((angle_deg + 30.0) / 0.1));

    auto encoded = make_command_frame(kAdControlSteeringId, counter);
    encoded.data[0] |= command.valid ? 1U : 0U;
    write_u16_le(encoded, 4, raw_angle);
    output = encoded;
    return true;
}

bool encode_brake_command(
    const BrakeCommand& command, std::uint8_t counter, CanFrame& output)
{
    if (counter > 15 || !in_range(command.brake_percent, 0.0, 100.0)) {
        return false;
    }

    auto encoded = make_command_frame(kAdControlBrakeId, counter);
    encoded.data[0] |= command.valid ? 1U : 0U;
    encoded.data[1] = static_cast<std::uint8_t>(std::lround(command.brake_percent));
    output = encoded;
    return true;
}

bool encode_speed_command(
    const SpeedCommand& command, std::uint8_t counter, CanFrame& output)
{
    const auto gear = static_cast<std::uint8_t>(command.gear);
    if (counter > 15 || gear > 3 ||
        !in_range(command.speed_mps, 0.0, kMaxCommandSpeedMps)) {
        return false;
    }

    const auto raw_speed = static_cast<std::uint16_t>(
        std::lround(command.speed_mps * 3.6 / 0.1));

    auto encoded = make_command_frame(kAdControlAccelerateId, counter);
    encoded.data[0] |= command.valid ? 1U : 0U;
    encoded.data[2] = 1; // Speed control mode only.
    encoded.data[3] = gear;
    // Acceleration field: raw * 0.1 - 5.0. Raw 50 represents 0 m/s^2.
    // Initialize this inactive field to physical zero, torque to zero.
    encoded.data[4] = 50;
    encoded.data[5] = 0;
    write_u16_le(encoded, 6, raw_speed);
    output = encoded;
    return true;
}

bool encode_body_command(
    const BodyCommand& command, std::uint8_t counter, CanFrame& output)
{
    if (counter > 15) {
        return false;
    }

    auto encoded = make_command_frame(kAdControlBodyId, counter);
    encoded.data[0] |= command.left_indicator_on ? 0x01U : 0U;
    encoded.data[0] |= command.right_indicator_on ? 0x02U : 0U;
    encoded.data[0] |= command.horn_on ? 0x04U : 0U;
    encoded.data[0] |= command.headlight_on ? 0x08U : 0U;
    encoded.data[1] = command.brake_light_on ? 1U : 0U;
    output = encoded;
    return true;
}
} // namespace withus_driver
