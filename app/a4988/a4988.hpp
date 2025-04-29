#ifndef A4988_HPP
#define A4988_HPP

#include "a4988_config.hpp"

namespace a4988 {

    struct A4988 {
    public:
        void initialize(this A4988& self) noexcept;
        void deinitialize(this A4988& self) noexcept;

        void start_pulses(this A4988& self) noexcept;
        void stop_pulses(this A4988& self) noexcept;

        void set_frequency(this A4988& self, std::uint32_t const frequency) noexcept;

        void set_microstep(this A4988& self, Microstep const microstep) noexcept;
        void set_full_microstep(this A4988& self) noexcept;
        void set_half_microstep(this A4988& self) noexcept;
        void set_quarter_microstep(this A4988& self) noexcept;
        void set_eighth_microstep(this A4988& self) noexcept;
        void set_sixteenth_microstep(this A4988& self) noexcept;

        void set_direction(this A4988& self, Direction const direction) noexcept;
        void set_forward_direction(this A4988& self) noexcept;
        void set_backward_direction(this A4988& self) noexcept;
        void set_stop_direction(this A4988& self) noexcept;

        void set_reset(this A4988 const& self, bool const reset = true) noexcept;
        void set_enable(this A4988 const& self, bool const enable = true) noexcept;
        void set_sleep(this A4988 const& self, bool const sleep = true) noexcept;

        Config config = {};
        Interface interface = {};

        bool has_pulses_started = {};
        bool is_initialized = {};

    private:
        void pulse_init(this A4988 const& self) noexcept;
        void pulse_deinit(this A4988 const& self) noexcept;
        void pulse_start(this A4988 const& self) noexcept;
        void pulse_stop(this A4988 const& self) noexcept;
        void pulse_set_freq(this A4988 const& self, std::uint32_t const freq) noexcept;

        void gpio_init(this A4988 const& self) noexcept;
        void gpio_deinit(this A4988 const& self) noexcept;
        void gpio_write_pin(this A4988 const& self, std::int16_t const pin, bool const state) noexcept;
    };

}; // namespace a4988

#endif // A4988_HPP