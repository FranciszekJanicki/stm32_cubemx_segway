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
    using Driver = StepDriver::StepDriver;

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

        ICM20948 imu{};
        PID regulator{};
        std::array<DriverChannel, 2UL> driver_channels{};

        std::float32_t prev_control_speed{};

    private:
        std::float32_t angle_to_angular_speed(std::float32_t const angle, std::float32_t const sampling_time) noexcept;

        Driver& get_driver(Channel const channel) noexcept;
    };

}; // namespace Segway

#endif // SEGWAY_HPP