#ifndef SEGWAY_HPP
#define SEGWAY_HPP

#include "../step_driver/step_driver.hpp"
#include "icm20948.hpp"
#include "pid.hpp"

namespace Segway {

    using StepDriver = StepDriver::StepDriver;
    using ICM20948 = ICM20948::ICM20948;
    using PID = Utility::PID<float>;

    struct Segway {
        void update_step_count() noexcept;

        void operator()(float const angle, float const sampling_time) noexcept;

        void set_angle(float const angle, float const sampling_time) noexcept;

        float angle_to_angular_speed(float const angle, float const sampling_time) noexcept;

        ICM20948 imu{};
        PID regulator{};
        std::array<StepDriver, 2UL> step_drivers{};

        float prev_control_speed{};
    };

}; // namespace Segway

#endif // SEGWAY_HPP