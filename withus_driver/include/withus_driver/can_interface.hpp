#pragma once

#include <string>

namespace withus_driver
{

// Query the kernel's link kind. Interface names and sysfs placement are not proof
// that a CAN interface is virtual. Any query or validation failure returns false.
bool is_vcan_interface(const std::string& interface_name);

}  // namespace withus_driver
