#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include "withus_driver/withus_protocol.hpp"

namespace withus_driver
{
struct FeedbackState
{
    bool received{false};
    std::chrono::steady_clock::time_point received_at{};
    std::int64_t received_ros_nanoseconds{0};
    std::uint8_t last_counter{255};
    std::uint64_t counter_discontinuities{0};
    std::uint64_t rejected_frames{0};

    void mark_received(
        std::uint8_t counter,
        std::chrono::steady_clock::time_point steady_time,
        std::int64_t ros_nanoseconds)
    {
        if (received) {
            const auto expected = static_cast<std::uint8_t>((last_counter + 1U) & 0x0F);
            if (counter != expected) {
                ++counter_discontinuities;
            }
        }
        received = true;
        received_at = steady_time;
        received_ros_nanoseconds = ros_nanoseconds;
        last_counter = counter;
    }

    double age_sec(std::chrono::steady_clock::time_point now) const
    {
        if (!received) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return std::chrono::duration<double>(now - received_at).count();
    }

    bool valid(std::chrono::steady_clock::time_point now, double timeout_sec) const
    {
        const double age = age_sec(now);
        return received && age >= 0.0 && age <= timeout_sec;
    }
};

struct VehicleState
{
    VehicleDiagnosis diagnosis{};
    VehicleStatus1 status1{};
    VehicleStatus2 status2{};
    FeedbackState status_301{};
    FeedbackState status_303{};
    FeedbackState status_304{};
};
} // namespace withus_driver