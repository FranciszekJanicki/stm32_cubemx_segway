#ifndef A4988_HPP
#define A4988_HPP

#include "a4988_config.hpp"

namespace A4988 {

    struct A4988 {
    public:
        A4988() noexcept = default;
        A4988(PWMDevice&& pwm_device,
              GPIO const pin_ms1,
              GPIO const pin_ms2,
              GPIO const pin_ms3,
              GPIO const pin_reset,
              GPIO const pin_sleep,
              GPIO const pin_dir,
              GPIO const pin_enable) noexcept;

        A4988(A4988 const& other) = delete;
        A4988(A4988&& other) noexcept = default;

        A4988& operator=(A4988 const& other) = delete;
        A4988& operator=(A4988&& other) noexcept = default;

        ~A4988() noexcept;

        void start(this A4988 const& self) noexcept;
        void stop(this A4988 const& self) noexcept;

        void set_frequency(this A4988 const& self, std::uint32_t const frequency) noexcept;

        void set_microstep(this A4988 const& self, Microstep const microstep) noexcept;
        void set_full_microstep(this A4988 const& self) noexcept;
        void set_half_microstep(this A4988 const& self) noexcept;
        void set_quarter_microstep(this A4988 const& self) noexcept;
        void set_eighth_microstep(this A4988 const& self) noexcept;
        void set_sixteenth_microstep(this A4988 const& self) noexcept;

        void set_direction(this A4988 const& self, Direction const direction) noexcept;
        void set_forward_direction(this A4988 const& self) noexcept;
        void set_backward_direction(this A4988 const& self) noexcept;
        void set_stop_direction(this A4988 const& self) noexcept;

        void set_reset(this A4988 const& self, bool const reset = true) noexcept;
        void set_enable(this A4988 const& self, bool const enable = true) noexcept;
        void set_sleep(this A4988 const& self, bool const sleep = true) noexcept;

        //  private:
        void initialize(this A4988 const& self) noexcept;
        void deinitialize(this A4988 const& self) noexcept;

        PWMDevice pwm_device_{};

        GPIO pin_ms1_{};
        GPIO pin_ms2_{};
        GPIO pin_ms3_{};
        GPIO pin_reset_{};
        GPIO pin_sleep_{};
        GPIO pin_dir_{};
        GPIO pin_enable_{};
    };

}; // namespace A4988

#endif // A4988_HPP