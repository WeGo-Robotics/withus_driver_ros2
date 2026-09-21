#pragma once

#include <chrono>
#include <limits>

namespace withus_driver
{
struct VehicleState
{
    double speed_mps{
        std::numeric_limits<double>::quiet_NaN()
    };

    bool speed_received{false};
    std::chrono::steady_clock::time_point speed_received_at{};
};
}// withus_dirver