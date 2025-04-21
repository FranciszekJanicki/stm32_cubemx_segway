#include "imu.hpp"
#include "log.hpp"

namespace {

    constexpr auto TAG = "IMU";

};

namespace Segway {

    std::float64_t get_tilt_angle(IMU& imu) noexcept
    {
        auto tilt_angle = std::visit([](auto& sensor) { return sensor.get_roll().value(); }, imu);
        auto tilt_angle_deg = Utility::radians_to_degrees(tilt_angle);

        LOG(TAG, "Measured tilt angle of: %f", tilt_angle_deg);
        return tilt_angle_deg;
    }

    std::float64_t get_rotation_angle(IMU& imu) noexcept
    {
        auto rotation_angle = std::visit([](auto& sensor) { return sensor.get_yaw().value(); }, imu);
        auto rotation_angle_deg = Utility::radians_to_degrees(rotation_angle);

        LOG(TAG, "Measured rotation angle of: %f", rotation_angle_deg);
        return rotation_angle_deg;
    }

}; // namespace Segway
