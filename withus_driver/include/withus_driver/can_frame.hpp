#pragma once

#include <array>
#include <cstdint>

namespace withus_driver
{

struct CanFrame
{
    std::uint32_t id{0};
    std::uint8_t length{0};
    std::array<std::uint8_t, 8> data{};
    bool is_extended{false};
    bool is_remote{false};
    bool is_error{false};
};

} // namespace withus_driver
