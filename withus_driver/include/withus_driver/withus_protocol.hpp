#pragma once

#include "withus_driver/can_frame.hpp"

namespace withus_driver
{
bool decode_vehicle_speed(const CanFrame& frame, double& speed_mps);
} // withus_driver
