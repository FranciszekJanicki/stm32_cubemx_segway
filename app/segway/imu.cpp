#include "imu.hpp"

namespace Segway {

    std::float32_t IMU::get_tilt(this IMU& self) noexcept
    {
        return std::visit([](auto& sensor) { return sensor.get_roll().value(); }, self.sensor);
    }

    std::float32_t IMU::get_rotation(this IMU& self) noexcept
    {
        return std::visit([](auto& sensor) { return sensor.get_yaw().value(); }, self.sensor);
    }

}; // namespace Segway
