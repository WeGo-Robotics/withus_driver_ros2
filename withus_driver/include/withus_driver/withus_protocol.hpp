#pragma once

#include <cstdint>
#include <limits>
#include "withus_driver/can_frame.hpp"

namespace withus_driver
{
constexpr std::uint32_t kRemoteControlIoId = 0x10A;
constexpr std::uint32_t kVehicleDiagnosisId = 0x301;
constexpr std::uint32_t kVehicleStatus1Id = 0x303;
constexpr std::uint32_t kVehicleStatus2Id = 0x304;

struct RemoteControlIo
{
    // Observed on this vehicle: 0 before A selection, 1 after selection.
    // Preserve other raw values; freshness is tracked separately.
    std::uint8_t a_switch_raw{255};
};

struct VehicleDiagnosis
{
    double battery_voltage_v{std::numeric_limits<double>::quiet_NaN()};
    bool emergency_stop_pressed{false};
    bool motor_fault{false};
    bool remote_fault{false};
    bool ad_signal_received{false};
    bool dbs_fault{false};
    bool parallel_driving_active{false};
    bool bms_fault{false};
    bool eps_fault{false};
    bool front_bumper_triggered{false};
    bool rear_bumper_triggered{false};
    bool headlight_on{false};
    bool left_indicator_on{false};
    bool brake_light_on{false};
    bool oil_level_abnormal{false};
    bool horn_on{false};
    bool right_indicator_on{false};
    std::uint8_t message_counter{255};
};

struct VehicleStatus1
{
    std::uint8_t gear{255};
    // Preserve raw 4-bit values; only 0..3 have named modes in the datasheet.
    std::uint8_t drive_mode{255};
    bool brake_hold_active{false};
    double requested_speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double battery_soc{std::numeric_limits<double>::quiet_NaN()};
    std::uint8_t message_counter{255};
};

struct VehicleStatus2
{
    double speed_mps{std::numeric_limits<double>::quiet_NaN()};
    double steering_angle_rad{std::numeric_limits<double>::quiet_NaN()};
    double brake_pressure_pa{std::numeric_limits<double>::quiet_NaN()};
    std::uint8_t message_counter{255};
};

// On failure, leave the output argument unchanged.
bool decode_remote_control_io(const CanFrame& frame, RemoteControlIo& output);
bool decode_vehicle_diagnosis(const CanFrame& frame, VehicleDiagnosis& output);
bool decode_vehicle_status1(const CanFrame& frame, VehicleStatus1& output);
bool decode_vehicle_status2(const CanFrame& frame, VehicleStatus2& output);

// AD command CAN IDs.
constexpr std::uint32_t kAdControlFlagId = 0x501;
constexpr std::uint32_t kAdControlSteeringId = 0x502;
constexpr std::uint32_t kAdControlBrakeId = 0x503;
constexpr std::uint32_t kAdControlAccelerateId = 0x504;
constexpr std::uint32_t kAdControlBodyId = 0x506;

// Protocol limits, not the operating limits of a particular vehicle.
constexpr double kMaxCommandSteeringRad = 3.14159265358979323846 / 6.0;
constexpr double kMaxCommandSpeedMps = 80.0 / 3.6;

enum class Gear : std::uint8_t
{
    Park = 0,
    Drive = 1,
    Neutral = 2,
    Reverse = 3
};

struct ControlRequest
{
    bool request_control{false};
};

struct SteeringCommand
{
    bool valid{false};
    // Preserve the datasheet sign; vehicle steering convention is separate.
    double steering_angle_rad{0.0};
};

struct BrakeCommand
{
    bool valid{false};
    // Command percentage, not measured pressure in Pa.
    double brake_percent{0.0};
};

struct SpeedCommand
{
    bool valid{false};
    Gear gear{Gear::Park};
    // Nonnegative magnitude. Direction is represented by gear.
    double speed_mps{0.0};
};

struct BodyCommand
{
    bool left_indicator_on{false};
    bool right_indicator_on{false};
    bool horn_on{false};
    bool headlight_on{false};
    bool brake_light_on{false};
};

// counter: 0..15. On failure, output is unchanged.
// These functions only encode bytes; they do not transmit or obtain control.
bool encode_control_request(
    const ControlRequest& command, std::uint8_t counter, CanFrame& output);
bool encode_steering_command(
    const SteeringCommand& command, std::uint8_t counter, CanFrame& output);
bool encode_brake_command(
    const BrakeCommand& command, std::uint8_t counter, CanFrame& output);
bool encode_speed_command(
    const SpeedCommand& command, std::uint8_t counter, CanFrame& output);
bool encode_body_command(
    const BodyCommand& command, std::uint8_t counter, CanFrame& output);

} // namespace withus_driver
