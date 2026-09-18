#include "withus_driver/withus_protocol.hpp"

#include <cstdint>
#include <cmath>

namespace withus_driver
{
bool decode_vehicle_speed(const CanFrame& frame, double& speed_mps)
{
    // 0x304는 표준 데이터 프레임이며, 길이는 8바이트
    if(frame.id != 0x304 || frame.length != 8 ||
        frame.is_extended || frame.is_remote || frame.is_error)
    {
        return false;
    }

    // Little-endian: data[0]이 하위, data[1]이 상위 바이트
    const std::uint16_t raw_speed = static_cast<std::uint16_t>(
        frame.data[0] | (frame.data[1] << 8)
    );

    const double speed_kmh = raw_speed * 0.1 - 80.0;

    // 범위 검사
    if(std::abs(speed_kmh) > 80.0){
        return false;
    }

    speed_mps = speed_kmh / 3.6; // km/h to m/s
    return true;
}
} // namespace withus driver