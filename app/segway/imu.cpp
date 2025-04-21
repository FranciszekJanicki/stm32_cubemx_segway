#include "imu.hpp"
#include "log.hpp"

namespace {

    constexpr auto TAG = "IMU";

};

namespace Segway {

    std::float64_t get_tilt_angle(IMU& imu) noexcept
    {
        auto tilt_angle = std::visit([](auto& sensor) { return sensor.get_roll().value(); }, imu);

        LOG(TAG, "Measured tilt angle of: %f", tilt_angle);
        return Utility::radians_to_degrees(tilt_angle);
    }

    std::float64_t get_rotation_angle(IMU& imu) noexcept
    {
        auto rotation_angle = std::visit([](auto& sensor) { return sensor.get_yaw().value(); }, imu);

        LOG(TAG, "Measured rotation angle of: %f", rotation_angle);
        return Utility::radians_to_degrees(rotation_angle);
    }

}; // namespace Segway
