#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "icm20948.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "pid.hpp"
#include "step_driver.hpp"
#include <variant>

namespace Segway {

    using namespace Utility;
    using namespace ICM20948;
    using namespace MPU6050;
    using namespace A4988;
    using namespace StepDriver;
    using namespace STM32_Utility;

    using A4988 = ::A4988::A4988;
    using ICM20948 = ::ICM20948::ICM20948;
    using MPU6050 = ::MPU6050::MPU6050;
    using MPU6050_DMP = ::MPU6050::MPU6050_DMP;
    using IMU = std::variant<ICM20948, MPU6050_DMP>;
    using Regulator = ::Utility::PID<std::float32_t>;
    using Driver = ::StepDriver::StepDriver;

    enum struct Channel : std::uint8_t {
        CHANNEL_1,
        CHANNEL_2,
    };

    struct DriverChannel {
        Channel channel{};
        Driver driver{};
    };

    struct Segway {
        void update_step_count(Channel const channel) noexcept;
        void set_speed(Channel const channel, std::float32_t const speed, std::float32_t const sampling_time) noexcept;

        void operator()(std::float32_t const angle, std::float32_t const sampling_time) noexcept;
        void set_angle(std::float32_t const angle, std::float32_t const sampling_time) noexcept;

        IMU imu{};
        Regulator regulator{};
        std::array<DriverChannel, 2UL> driver_channels{};

        std::float32_t prev_control_speed{};

    private:
        std::float32_t angle_to_angular_speed(std::float32_t const angle, std::float32_t const sampling_time) noexcept;

        Driver& get_driver(Channel const channel) noexcept;
    };

}; // namespace Segway

#endif // SEGWAY_HPP