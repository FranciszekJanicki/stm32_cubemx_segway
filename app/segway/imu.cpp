#include "imu.hpp"
#include "i2c.h"
#include "log.hpp"

namespace {

    constexpr auto TAG = "IMU";

};

namespace segway {

    std::float64_t get_tilt_angle(IMU& imu) noexcept
    {
        auto tilt_angle = std::visit([](auto& sensor) { return sensor.get_roll().value(); }, imu);
        auto tilt_angle_deg = utility::radians_to_degrees(tilt_angle);

        // LOG(TAG, "Measured tilt angle of: %f", tilt_angle_deg);
        return tilt_angle_deg;
    }

    std::float64_t get_rotation_angle(IMU& imu) noexcept
    {
        auto rotation_angle = std::visit([](auto& sensor) { return sensor.get_yaw().value(); }, imu);
        auto rotation_angle_deg = utility::radians_to_degrees(rotation_angle);

        // LOG(TAG, "Measured rotation angle of: %f", rotation_angle_deg);
        return rotation_angle_deg;
    }

    bool bus_scan(IMU& imu) noexcept
    {
        return HAL_I2C_IsDeviceReady(&hi2c1, 104, 10, 100) == HAL_OK;
    }

}; // namespace segway
