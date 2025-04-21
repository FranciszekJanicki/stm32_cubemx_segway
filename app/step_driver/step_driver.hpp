#ifndef STEP_DRIVER_HPP
#define STEP_DRIVER_HPP

#include "a4988.hpp"
#include "pid.hpp"

namespace StepDriver {

    using namespace A4988;
    using namespace Utility;
    using A4988 = ::A4988::A4988;

    Microstep speed_to_microstep(std::float64_t const speed, std::float64_t const step_change) noexcept;
    Direction speed_to_direction(std::float64_t const speed) noexcept;
    std::uint16_t speed_to_frequency(std::float64_t const speed, std::float64_t const step_change) noexcept;

    struct StepDriver {
    public:
        void start(this StepDriver& self) noexcept;
        void stop(this StepDriver& self) noexcept;

        void update_step_count(this StepDriver& self) noexcept;

        void set_position(this StepDriver& self, std::float64_t const position, std::float64_t const dt) noexcept;
        void set_speed(this StepDriver& self, std::float64_t const speed, std::float64_t const dt) noexcept;
        void
        set_acceleration(this StepDriver& self, std::float64_t const acceleration, std::float64_t const dt) noexcept;

        std::float64_t get_position(this StepDriver& self, std::float64_t const dt) noexcept;
        std::float64_t get_speed(this StepDriver& self, std::float64_t const dt) noexcept;
        std::float64_t get_acceleration(this StepDriver& self, std::float64_t const dt) noexcept;

        A4988 driver = {};
        std::uint16_t steps_per_360 = {};

        Microstep microstep = {};
        Direction direction = {};
        std::uint16_t frequency = {};

        std::int64_t step_count = {};

        std::float64_t prev_position = {};
        std::float64_t prev_speed = {};
        std::float64_t prev_acceleration = {};

        bool is_stopped = {};
        bool is_initialized = {};

    private:
        static constexpr auto MAX_SPEED = 1000.0F64;
        static constexpr auto MIN_SPEED = 1.0F64;

        std::float64_t step_change(this StepDriver& self) noexcept;

        void set_control_speed(this StepDriver& self, std::float64_t const control_speed) noexcept;
        void set_microstep(this StepDriver& self, Microstep const microstep) noexcept;
        void set_direction(this StepDriver& self, Direction const direction) noexcept;
        void set_frequency(this StepDriver& self, std::uint16_t const frequency) noexcept;
    };

}; // namespace StepDriver

#endif // STEP_DRIVER_HPP