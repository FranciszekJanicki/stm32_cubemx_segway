#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "icm20948.hpp"
#include "pid.hpp"
#include "step_driver.hpp"

namespace Segway {

    using namespace STM32_Utility;
    using A4988 = A4988::A4988;
    using ICM20948 = ICM20948::ICM20948;
    using PID = Utility::PID<std::float32_t>;
    using StepDriver = StepDriver::StepDriver;

    struct Segway {
        void update_step_count() noexcept;

        void operator()(std::float32_t const angle, std::float32_t const sampling_time) noexcept;

        void set_angle(std::float32_t const angle, std::float32_t const sampling_time) noexcept;

        std::float32_t angle_to_angular_speed(std::float32_t const angle, std::float32_t const sampling_time) noexcept;

        ICM20948 imu{};
        PID regulator{};
        std::array<StepDriver, 2UL> drivers{};

        std::float32_t prev_control_speed{};
    };

}; // namespace Segway

#endif // SEGWAY_HPP