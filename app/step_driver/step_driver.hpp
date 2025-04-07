#ifndef STEP_DRIVER_HPP
#define STEP_DRIVER_HPP

#include "a4988.hpp"
#include "pid.hpp"

namespace StepDriver {

    using namespace A4988;
    using namespace Utility;
    using A4988 = ::A4988::A4988;

    Microstep speed_to_microstep(std::float32_t const speed,
                                 std::float32_t const step_change) noexcept;

    Direction speed_to_direction(std::float32_t const speed) noexcept;

    std::uint16_t speed_to_frequency(std::float32_t const speed,
                                     std::float32_t const step_change) noexcept;

    struct StepDriver {
    public:
        void update_step_count(this StepDriver& self) noexcept;

        void set_position(this StepDriver& self,
                          std::float32_t const position,
                          std::float32_t const sampling_time) noexcept;
        void set_speed(this StepDriver& self,
                       std::float32_t const speed,
                       std::float32_t const sampling_time) noexcept;
        void set_acceleration(this StepDriver& self,
                              std::float32_t const acceleration,
                              std::float32_t const sampling_time) noexcept;

        std::float32_t get_position(this StepDriver& self,
                                    std::float32_t const sampling_time) noexcept;
        std::float32_t get_speed(this StepDriver& self,
                                 std::float32_t const sampling_time) noexcept;
        std::float32_t get_acceleration(this StepDriver& self,
                                        std::float32_t const sampling_time) noexcept;

        A4988 driver = A4988{};
        std::uint16_t steps_per_360 = 0U;

        Microstep microstep = Microstep{};
        Direction direction = Direction{};
        std::uint16_t frequency = 0U;

        std::int64_t step_count = 0LL;

        std::float32_t prev_position = 0.0F;
        std::float32_t prev_speed = 0.0F;
        std::float32_t prev_acceleration = 0.0F;

        bool is_stopped = false;

    private:
        void start(this StepDriver& self) noexcept;
        void stop(this StepDriver& self) noexcept;

        std::float32_t step_change(this StepDriver& self) noexcept;

        void set_control_speed(this StepDriver& self, std::float32_t const control_speed) noexcept;
        void set_microstep(this StepDriver& self, Microstep const microstep) noexcept;
        void set_direction(this StepDriver& self, Direction const direction) noexcept;
        void set_frequency(this StepDriver& self, std::uint16_t const frequency) noexcept;
    };

}; // namespace StepDriver

#endif // STEP_DRIVER_HPP