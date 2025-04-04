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

        void start() const noexcept;
        void stop() const noexcept;

        void set_frequency(std::uint32_t const frequency) noexcept;

        void set_microstep(Microstep const microstep) const noexcept;
        void set_full_microstep() const noexcept;
        void set_half_microstep() const noexcept;
        void set_quarter_microstep() const noexcept;
        void set_eighth_microstep() const noexcept;
        void set_sixteenth_microstep() const noexcept;

        void set_direction(Direction const direction) const noexcept;
        void set_forward_direction() const noexcept;
        void set_backward_direction() const noexcept;
        void set_stop_direction() const noexcept;

        void set_reset(bool const reset = true) const noexcept;
        void set_enable(bool const enable = true) const noexcept;
        void set_sleep(bool const sleep = true) const noexcept;

        //  private:
        void initialize() noexcept;
        void deinitialize() noexcept;

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