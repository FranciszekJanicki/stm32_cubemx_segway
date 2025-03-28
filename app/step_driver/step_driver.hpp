#ifndef STEP_DRIVER_HPP
#define STEP_DRIVER_HPP

#include "../a4988/a4988.hpp"
#include "../utility/pid.hpp"

namespace StepDriver {

    using namespace A4988;
    using namespace Utility;
    using A4988 = ::A4988::A4988;

    Microstep speed_to_microstep(float const speed, float const step_change) noexcept;

    Direction speed_to_direction(float const speed) noexcept;

    std::uint16_t speed_to_frequency(float const speed, float const step_change) noexcept;

    float get_processed_speed(float const speed,
                              float const prev_speed,
                              float const step_change,
                              float const sampling_time,
                              float const max_speed,
                              float const max_acceleration) noexcept;

    auto constexpr MIN_SPEED = 10.0F; // degrees per second

    struct StepDriver {
    public:
        void update_step_count() noexcept;

        void set_position(float const position, float const sampling_time) noexcept;
        void set_speed(float const speed, float const sampling_time) noexcept;
        void set_acceleration(float const acceleration, float const sampling_time) noexcept;

        A4988 driver{};
        PID<float> regulator{};
        std::uint16_t steps_per_360{};

        Microstep microstep{};
        Direction direction{};
        std::uint16_t frequency{};

        std::int64_t step_count{};

        float prev_position{};
        float prev_speed{};
        float prev_acceleration{};

        bool stopped{};

    private:
        void start_pwm() noexcept;
        void stop_pwm() noexcept;

        float step_change() const noexcept;

        void set_control_speed(float const control_speed) noexcept;
        void set_microstep(Microstep const microstep) noexcept;
        void set_direction(Direction const direction) noexcept;
        void set_frequency(std::uint16_t const frequency) noexcept;

        float get_position(float const sampling_time) noexcept;
        float get_speed(float const sampling_time) noexcept;
        float get_acceleration(float const sampling_time) noexcept;
    };

}; // namespace StepDriver

#endif // STEP_DRIVER_HPP