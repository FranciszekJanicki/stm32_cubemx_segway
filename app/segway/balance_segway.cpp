#include "balance_segway.hpp"

namespace Segway {

    void balance_segway()
    {
        Hardware::initialize_gpio();
        Hardware::initialize_tim1();
        Hardware::initialize_tim2();
        Hardware::initialize_tim3();
        Hardware::deinitialize_uart2();
        Hardware::initialize_i2c1();

        auto constexpr MS1_1 = Utility::GPIO::PB4;
        auto constexpr MS2_1 = Utility::GPIO::PB5;
        auto constexpr MS3_1 = Utility::GPIO::PB6;
        auto constexpr DIR_1 = Utility::GPIO::PB7;

        auto constexpr MS1_2 = Utility::GPIO::PA3;
        auto constexpr MS2_2 = Utility::GPIO::PA4;
        auto constexpr MS3_2 = Utility::GPIO::PA5;
        auto constexpr DIR_2 = Utility::GPIO::PA7;

        auto constexpr ANGLE = 0.0F;
        auto constexpr SAMPLING_TIME = 1.0F;

        auto pwm_device_1 = Utility::PWMDevice{&htim1, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988_1 = A4988::A4988{std::move(pwm_device_1), MS1_1, MS2_1, MS3_1, {}, {}, DIR_1, {}};

        auto speed_regulator_1 = Utility::PID<float>{.proportion_gain = 10.0F,
                                                     .integral_gain = 0.0F,
                                                     .derivative_gain = 0.0F,
                                                     .time_constant = 0.0F,
                                                     .control_gain = 0.0F,
                                                     .saturation = 100.0F};

        auto step_driver_1 =
            StepDriver::StepDriver{.driver = std::move(a4988_1), .regulator = speed_regulator_1, .steps_per_360 = 200U};

        auto pwm_device_2 = Utility::PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988_2 = A4988::A4988{std::move(pwm_device_2), MS1_2, MS2_2, MS3_2, {}, {}, DIR_2, {}};

        auto speed_regulator_2 = Utility::PID<float>{.proportion_gain = 10.0F,
                                                     .integral_gain = 0.0F,
                                                     .derivative_gain = 0.0F,
                                                     .time_constant = 0.0F,
                                                     .control_gain = 0.0F,
                                                     .saturation = 100.0F};

        auto step_driver_2 =
            StepDriver::StepDriver{.driver = std::move(a4988_2), .regulator = speed_regulator_2, .steps_per_360 = 200U};

        auto i2c_device = Utility::I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD0};

        auto icm20948 = ICM20948::ICM20948{std::move(i2c_device)};

        auto angle_regulator = Utility::PID<float>{.proportion_gain = 10.0F,
                                                   .integral_gain = 0.0F,
                                                   .derivative_gain = 0.0F,
                                                   .time_constant = 0.0F,
                                                   .control_gain = 0.0F,
                                                   .saturation = 100.0F};

        auto segway = Segway::Segway{.imu = std::move(icm20948),
                                     .regulator = angle_regulator,
                                     .step_drivers = {std::move(step_driver_1), std::move(step_driver_2)}};

        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0.0F;

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