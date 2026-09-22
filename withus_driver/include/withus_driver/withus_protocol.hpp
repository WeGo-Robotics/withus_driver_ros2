#pragma once

#include <cstdint>
#include <limits>
#include "withus_driver/can_frame.hpp"

namespace withus_driver
{
constexpr std::uint32_t kVehicleDiagnosisId = 0x301;
constexpr std::uint32_t kVehicleStatus1Id = 0x303;
constexpr std::uint32_t kVehicleStatus2Id = 0x304;

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
bool decode_vehicle_diagnosis(const CanFrame& frame, VehicleDiagnosis& output);
bool decode_vehicle_status1(const CanFrame& frame, VehicleStatus1& output);
bool decode_vehicle_status2(const CanFrame& frame, VehicleStatus2& output);
} // namespace withus_driver