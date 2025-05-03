#ifndef STEP_DRIVER_HPP
#define STEP_DRIVER_HPP

#include "a4988.hpp"

namespace step_driver {

    using namespace a4988;

    struct StepDriver {
    public:
        void initialize(this StepDriver& self) noexcept;
        void deinitialize(this StepDriver& self) noexcept;

        void start(this StepDriver& self) noexcept;
        void stop(this StepDriver& self) noexcept;

        void update_step_count(this StepDriver& self) noexcept;

        void set_position(this StepDriver& self,
                          std::float64_t const position,
                          std::float64_t const dt) noexcept;
        void set_speed(this StepDriver& self,
                       std::float64_t const speed,
                       std::float64_t const dt) noexcept;
        void set_acceleration(this StepDriver& self,
                              std::float64_t const acceleration,
                              std::float64_t const dt) noexcept;

        std::float64_t get_position(this StepDriver& self, std::float64_t const dt) noexcept;
        std::float64_t get_speed(this StepDriver& self, std::float64_t const dt) noexcept;
        std::float64_t get_acceleration(this StepDriver& self, std::float64_t const dt) noexcept;

        A4988 driver = {};
        std::uint16_t steps_per_360 = {};

        Microstep microstep = {};
        Direction direction = {};
        std::uint32_t frequency = {};

        std::int64_t step_count = {};

        std::float64_t prev_position = {};
        std::float64_t prev_speed = {};
        std::float64_t prev_acceleration = {};

        bool is_stopped = {};
        bool is_initialized = {};

    private:
        static Microstep speed_to_microstep(std::float64_t const speed,
                                            std::float64_t const step_change) noexcept;
        static Direction speed_to_direction(std::float64_t const speed) noexcept;
        static std::uint32_t speed_to_frequency(std::float64_t const speed,
                                                std::float64_t const step_change) noexcept;

        static constexpr auto MAX_SPEED = 3000.0F64;
        static constexpr auto MIN_SPEED = 0.0F64;

        std::float64_t step_change(this StepDriver& self) noexcept;

        void set_control_speed(this StepDriver& self, std::float64_t const control_speed) noexcept;
        void set_microstep(this StepDriver& self, Microstep const microstep) noexcept;
        void set_direction(this StepDriver& self, Direction const direction) noexcept;
        void set_frequency(this StepDriver& self, std::uint32_t const frequency) noexcept;

        bool should_start(this StepDriver& self, std::float64_t const control_speed) noexcept;
        bool should_stop(this StepDriver& self, std::float64_t const control_speed) noexcept;
    };

}; // namespace step_driver

#endif // STEP_DRIVER_HPP