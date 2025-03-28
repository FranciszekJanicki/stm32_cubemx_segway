#include "unit_test.hpp"
#include "a4988.hpp"
#include "gpio.hpp"
#include "hardware.hpp"
#include "i2c_device.hpp"
#include "icm20948.hpp"
#include "pwm_device.hpp"
#include "segway.hpp"
#include "step_driver.hpp"

namespace Segway {

    void test_a4988_1() noexcept
    {
        auto constexpr MS1 = STM32_Utility::GPIO::PA11;
        auto constexpr MS2 = STM32_Utility::GPIO::PA10;
        auto constexpr MS3 = STM32_Utility::GPIO::PA9;
        auto constexpr DIR = STM32_Utility::GPIO::PB15;

        auto pwm_device = STM32_Utility::PWMDevice{&htim1, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988::A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            if (tim1_pulse_finished) {
                a4988.set_frequency(i);

                tim1_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            }
        }
    }

    void test_a4988_2() noexcept
    {
        auto constexpr MS1 = STM32_Utility::GPIO::PA3;
        auto constexpr MS2 = STM32_Utility::GPIO::PA4;
        auto constexpr MS3 = STM32_Utility::GPIO::PA5;
        auto constexpr DIR = STM32_Utility::GPIO::PA7;

        auto pwm_device = STM32_Utility::PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988::A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            if (tim3_pulse_finished) {
                a4988.set_frequency(i);

                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }
        }
    }

    void test_icm20948() noexcept
    {
        auto i2c_device = STM32_Utility::I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD0};

        auto icm20948 = ICM20948::ICM20948{std::move(i2c_device)};

        while (1) {
            if (gpio_pin6_exti) {
                auto const& [r, p, y] = icm20948.get_roll_pitch_yaw().value();
                std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);

                gpio_pin6_exti = false;
            }
        }
    }

    void test_step_driver_1() noexcept
    {
        auto constexpr MS1 = STM32_Utility::GPIO::PB4;
        auto constexpr MS2 = STM32_Utility::GPIO::PB5;
        auto constexpr MS3 = STM32_Utility::GPIO::PB6;
        auto constexpr DIR = STM32_Utility::GPIO::PB7;
        auto constexpr SAMPLING_TIME = 1.0F;

        auto pwm_device = STM32_Utility::PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988::A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        auto pid = Utility::PID<float>{.proportion_gain = 10.0F,
                                       .integral_gain = 0.0F,
                                       .derivative_gain = 0.0F,
                                       .time_constant = 0.0F,
                                       .control_gain = 0.0F,
                                       .saturation = 100.0F};

        auto step_driver = StepDriver::StepDriver{.driver = std::move(a4988), .regulator = pid, .steps_per_360 = 200U};

        HAL_TIM_Base_Start_IT(&htim3);
        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            if (tim2_period_elapsed) {
                step_driver.set_speed(i += 3, SAMPLING_TIME);

                tim2_period_elapsed = false;
                HAL_TIM_Base_Start_IT(&htim2);
            }

            if (tim1_pulse_finished) {
                step_driver.update_step_count();

                tim1_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            }
        }
    }

    void test_step_driver_2() noexcept
    {
        auto constexpr MS1 = STM32_Utility::GPIO::PA3;
        auto constexpr MS2 = STM32_Utility::GPIO::PA4;
        auto constexpr MS3 = STM32_Utility::GPIO::PA5;
        auto constexpr DIR = STM32_Utility::GPIO::PA7;
        auto constexpr SAMPLING_TIME = 1.0F;

        auto pwm_device = STM32_Utility::PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988::A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        auto pid = Utility::PID<float>{.proportion_gain = 10.0F,
                                       .integral_gain = 0.0F,
                                       .derivative_gain = 0.0F,
                                       .time_constant = 0.0F,
                                       .control_gain = 0.0F,
                                       .saturation = 100.0F};

        auto step_driver = StepDriver::StepDriver{.driver = std::move(a4988), .regulator = pid, .steps_per_360 = 200U};

        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            if (tim2_period_elapsed) {
                step_driver.set_speed(i += 3, SAMPLING_TIME);

                tim2_period_elapsed = false;
                HAL_TIM_Base_Start_IT(&htim2);
            }

            if (tim3_pulse_finished) {
                step_driver.update_step_count();

                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }
        }
    }

}; // namespace Segway