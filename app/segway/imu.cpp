#include "imu.hpp"

namespace Segway {

    std::float32_t get_tilt(IMU& imu) noexcept
    {
        return std::visit([](auto& sensor) { return sensor.get_roll().value(); }, imu);
    }

    std::float32_t get_rotation(IMU& imu) noexcept
    {
        return std::visit([](auto& sensor) { return sensor.get_yaw().value(); }, imu);
    }

}; // namespace Segway
