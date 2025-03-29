#include "unit_test.hpp"
#include "a4988.hpp"
#include "gpio.h"
#include "gpio.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
#include "icm20948.hpp"
#include "nvic.hpp"
#include "pwm_device.hpp"
#include "segway.hpp"
#include "step_driver.hpp"
#include "tim.h"
#include "usart.h"

namespace Segway {

    void test_a4988_1() noexcept
    {
        constexpr auto MS1 = GPIO::PA11;
        constexpr auto MS2 = GPIO::PA10;
        constexpr auto MS3 = GPIO::PA9;
        constexpr auto DIR = GPIO::PB15;

        auto pwm_device = PWMDevice{&htim1, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

        auto i = 0U;

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
        constexpr auto MS1 = GPIO::PA3;
        constexpr auto MS2 = GPIO::PA4;
        constexpr auto MS3 = GPIO::PA5;
        constexpr auto DIR = GPIO::PA7;

        auto pwm_device = PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0U;

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
        auto i2c_device = I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD0};

        auto icm20948 = ICM20948{std::move(i2c_device)};

        while (1) {
            if (gpio_pin6_exti) {
                auto const rpy = icm20948.get_roll_pitch_yaw();
                auto const& [r, p, y] = rpy.value();
                std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);

                gpio_pin6_exti = false;
            }
        }
    }

    void test_step_driver_1() noexcept
    {
        constexpr auto MS1 = GPIO::PB4;
        constexpr auto MS2 = GPIO::PB5;
        constexpr auto MS3 = GPIO::PB6;
        constexpr auto DIR = GPIO::PB7;
        constexpr auto SAMPLING_TIME = 1.0F;

        auto pwm_device = PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        auto pid = PID{.proportion_gain = 10.0F,
                       .integral_gain = 0.0F,
                       .derivative_gain = 0.0F,
                       .time_constant = 0.0F,
                       .control_gain = 0.0F,
                       .saturation = 100.0F};

        auto step_driver = StepDriver{.driver = std::move(a4988), .regulator = pid, .steps_per_360 = 200U};

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
        constexpr auto MS1 = GPIO::PA3;
        constexpr auto MS2 = GPIO::PA4;
        constexpr auto MS3 = GPIO::PA5;
        constexpr auto DIR = GPIO::PA7;
        constexpr auto SAMPLING_TIME = 1.0F;

        auto pwm_device = PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988{std::move(pwm_device), MS1, MS2, MS3, {}, {}, DIR, {}};

        auto pid = PID{.proportion_gain = 10.0F,
                       .integral_gain = 0.0F,
                       .derivative_gain = 0.0F,
                       .time_constant = 0.0F,
                       .control_gain = 0.0F,
                       .saturation = 100.0F};

        auto step_driver = StepDriver{.driver = std::move(a4988), .regulator = pid, .steps_per_360 = 200U};

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

    void test_segway() noexcept
    {
        constexpr auto MS1_1 = GPIO::PB4;
        constexpr auto MS2_1 = GPIO::PB5;
        constexpr auto MS3_1 = GPIO::PB6;
        constexpr auto DIR_1 = GPIO::PB7;

        constexpr auto MS1_2 = GPIO::PA3;
        constexpr auto MS2_2 = GPIO::PA4;
        constexpr auto MS3_2 = GPIO::PA5;
        constexpr auto DIR_2 = GPIO::PA7;

        constexpr auto ANGLE = 0.0F;
        constexpr auto SAMPLING_TIME = 1.0F;

        auto pwm_device_1 = PWMDevice{&htim1, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988_1 = A4988{std::move(pwm_device_1), MS1_1, MS2_1, MS3_1, {}, {}, DIR_1, {}};

        auto speed_PID_1 = PID{.proportion_gain = 10.0F,
                               .integral_gain = 0.0F,
                               .derivative_gain = 0.0F,
                               .time_constant = 0.0F,
                               .control_gain = 0.0F,
                               .saturation = 100.0F};

        auto step_driver_1 = StepDriver{.driver = std::move(a4988_1), .regulator = speed_PID_1, .steps_per_360 = 200U};

        auto pwm_device_2 = PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988_2 = A4988{std::move(pwm_device_2), MS1_2, MS2_2, MS3_2, {}, {}, DIR_2, {}};

        auto speed_PID_2 = PID{.proportion_gain = 10.0F,
                               .integral_gain = 0.0F,
                               .derivative_gain = 0.0F,
                               .time_constant = 0.0F,
                               .control_gain = 0.0F,
                               .saturation = 100.0F};

        auto step_driver_2 = StepDriver{.driver = std::move(a4988_2), .regulator = speed_PID_2, .steps_per_360 = 200U};

        auto i2c_device = I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD0};

        auto icm20948 = ICM20948{std::move(i2c_device)};

        auto angle_PID = PID{.proportion_gain = 10.0F,
                             .integral_gain = 0.0F,
                             .derivative_gain = 0.0F,
                             .time_constant = 0.0F,
                             .control_gain = 0.0F,
                             .saturation = 100.0F};

        auto segway = Segway{.imu = std::move(icm20948),
                             .regulator = angle_PID,
                             .drivers = {std::move(step_driver_1), std::move(step_driver_2)}};

        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        while (1) {
            if (gpio_pin6_exti) {
                segway.set_angle(ANGLE, SAMPLING_TIME);

                gpio_pin6_exti = false;
                HAL_TIM_Base_Start_IT(&htim2);
            }

            if (tim3_pulse_finished) {
                segway.update_step_count();

                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }

            if (tim1_pulse_finished) {
                segway.update_step_count();

                tim1_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            }
        }
    }

}; // namespace Segway