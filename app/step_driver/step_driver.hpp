#ifndef STEP_DRIVER_HPP
#define STEP_DRIVER_HPP

#include "a4988.hpp"
#include "pid.hpp"

namespace StepDriver {

    using namespace A4988;
    using namespace Utility;
    using A4988 = ::A4988::A4988;

    Microstep speed_to_microstep(std::float32_t const speed, std::float32_t const step_change) noexcept;

    Direction speed_to_direction(std::float32_t const speed) noexcept;

    std::uint16_t speed_to_frequency(std::float32_t const speed, std::float32_t const step_change) noexcept;

    struct StepDriver {
    public:
        void update_step_count() noexcept;

        void set_position(std::float32_t const position, std::float32_t const sampling_time) noexcept;
        void set_speed(std::float32_t const speed, std::float32_t const sampling_time) noexcept;
        void set_acceleration(std::float32_t const acceleration, std::float32_t const sampling_time) noexcept;

        A4988 driver{};
        PID<std::float32_t> regulator{};
        std::uint16_t steps_per_360{};

        Microstep microstep{};
        Direction direction{};
        std::uint16_t frequency{};

        std::int64_t step_count{};

        std::float32_t prev_position{};
        std::float32_t prev_speed{};
        std::float32_t prev_acceleration{};

        bool is_stopped{};

    private:
        void start() noexcept;
        void stop() noexcept;

        std::float32_t step_change() const noexcept;

        void set_control_speed(std::float32_t const control_speed) noexcept;
        void set_microstep(Microstep const microstep) noexcept;
        void set_direction(Direction const direction) noexcept;
        void set_frequency(std::uint16_t const frequency) noexcept;

        std::float32_t get_position(std::float32_t const sampling_time) noexcept;
        std::float32_t get_speed(std::float32_t const sampling_time) noexcept;
        std::float32_t get_acceleration(std::float32_t const sampling_time) noexcept;
    };

}; // namespace StepDriver

#endif // STEP_DRIVER_HPP