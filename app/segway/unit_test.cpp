#include "unit_test.hpp"
#include "a4988.hpp"
#include "gpio.h"
#include "gpio.hpp"
#include "i2c.h"
#include "i2c_device.hpp"
#include "icm20948.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "nvic.hpp"
#include "pwm_device.hpp"
#include "segway.hpp"
#include "step_driver.hpp"
#include "tim.h"
#include "usart.h"
#include "vector3d.hpp"

namespace Segway {

    constexpr auto MS1_1 = GPIO::PB4;
    constexpr auto MS2_1 = GPIO::PB5;
    constexpr auto MS3_1 = GPIO::PB6;
    constexpr auto DIR_1 = GPIO::PB7;

    constexpr auto MS1_2 = GPIO::PA3;
    constexpr auto MS2_2 = GPIO::PA4;
    constexpr auto MS3_2 = GPIO::PA5;
    constexpr auto DIR_2 = GPIO::PA7;

    constexpr auto ANGLE = 0.0F;
    constexpr auto SAMPLING_TIME = 0.01F;

    void test_icm20948() noexcept
    {
        using namespace ICM20948;

        auto i2c_device = I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD1};
        i2c_device.bus_scan();

        auto icm20948 = ICM20948{std::move(i2c_device)};

        while (1) {
            if (gpio_pin6_exti) {
                auto const& [r, p, y] = icm20948.get_roll_pitch_yaw();
                std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);
                gpio_pin6_exti = false;
            }
        }
    }

    void test_mpu6050() noexcept
    {
        using namespace MPU6050;

        auto i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};
        i2c_device.bus_scan();

        auto mpu6050 = MPU6050{std::move(i2c_device),
                               200U,
                               GyroRange::GYRO_FS_2000,
                               AccelRange::ACCEL_FS_2,
                               DLPF::BW_42,
                               DHPF::DHPF_RESET};

        auto mpu_dmp = MPU6050_DMP{std::move(mpu6050)};

        while (1) {
            //  if (gpio_pin6_exti) {
            auto const& [r, p, y] = mpu_dmp.get_roll_pitch_yaw();
            std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);
            gpio_pin6_exti = false;
            HAL_Delay(50);
            // }
        }
    }

    void test_a4988_1() noexcept
    {
        auto pwm_device = PWMDevice{&htim1, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988{std::move(pwm_device), MS1_1, MS2_1, MS3_1, {}, {}, DIR_1, {}};

        auto regulator = Regulator{.proportion_gain = 1.0F, .integral_gain = 0.0F, .derivative_gain = 0.0F};

        auto step_driver = Driver{.driver = std::move(a4988), .regulator = regulator, .steps_per_360 = 200U};

        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            if (tim2_period_elapsed) {
                step_driver.set_speed(i += 1.0F, SAMPLING_TIME);
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

    void test_a4988_2() noexcept
    {
        auto pwm_device = PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988 = A4988{std::move(pwm_device), MS1_2, MS2_2, MS3_2, {}, {}, DIR_2, {}};

        auto regulator = Regulator{.proportion_gain = 10.0F, .integral_gain = 0.0F, .derivative_gain = 0.0F};

        auto driver = Driver{.driver = std::move(a4988), .regulator = regulator, .steps_per_360 = 200U};

        HAL_TIM_Base_Start_IT(&htim2);
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            if (tim2_period_elapsed) {
                driver.set_speed(i += 1.0F, SAMPLING_TIME);
                tim2_period_elapsed = false;
                HAL_TIM_Base_Start_IT(&htim2);
            }

            if (tim3_pulse_finished) {
                driver.update_step_count();
                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }
        }
    }

    void test_segway() noexcept
    {
        auto pwm_device_1 = PWMDevice{&htim1, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988_1 = A4988{std::move(pwm_device_1), MS1_1, MS2_1, MS3_1, {}, {}, DIR_1, {}};

        auto driver_1 = Driver{.driver = std::move(a4988_1), .regulator = {}, .steps_per_360 = 200U};

        auto pwm_device_2 = PWMDevice{&htim3, TIM_CHANNEL_1, 65535U, 3.3F};

        auto a4988_2 = A4988{std::move(pwm_device_2), MS1_2, MS2_2, MS3_2, {}, {}, DIR_2, {}};

        auto driver_2 = Driver{.driver = std::move(a4988_2), .regulator = {}, .steps_per_360 = 200U};

        auto i2c_device = I2CDevice{&hi2c1, ICM_20948_I2C_ADDR_AD0};

        auto icm20948 = ICM20948{std::move(i2c_device)};

        auto angle_regulator =
            Regulator{.proportion_gain = 1.0F, .integral_gain = 1.0F, .derivative_gain = 0.0F, .saturation = 100000.0F};

        auto driver_channels = std::array{DriverChannel{.channel = Channel::CHANNEL_1, .driver = std::move(driver_1)},
                                          DriverChannel{.channel = Channel::CHANNEL_2, .driver = std::move(driver_2)}};

        auto segway = Segway{.imu = std::move(icm20948),
                             .regulator = angle_regulator,
                             .driver_channels = std::move(driver_channels)};

        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        while (1) {
            if (gpio_pin6_exti) {
                segway.set_angle(ANGLE, SAMPLING_TIME);
                gpio_pin6_exti = false;
            }

            if (tim3_pulse_finished) {
                segway.update_step_count(Channel::CHANNEL_1);
                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }

            if (tim1_pulse_finished) {
                segway.update_step_count(Channel::CHANNEL_2);
                tim1_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            }
        }
    }

}; // namespace Segway