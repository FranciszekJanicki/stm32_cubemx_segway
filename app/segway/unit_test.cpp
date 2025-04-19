#include "unit_test.hpp"
#include "gpio.h"
#include "i2c.h"
#include "icm_sensor.h"
#include "log.hpp"
#include "segway.hpp"
#include "segway_config.hpp"
#include "tim.h"
#include "usbd_cdc_if.h"

namespace {

    constexpr auto TAG = "Unit test";

};

namespace Segway {

    void test_icm20948() noexcept
    {
        auto i2c_device = I2CDevice{.i2c_bus = &hi2c1, .dev_address = ICM20948_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto icm20948_dmp = ICM20948_DMP{std::move(i2c_device)};

        while (1) {
            if (gpio_pin6_exti) {
                auto const& [r, p, y] = icm20948_dmp.get_roll_pitch_yaw().value();
                LOG(TAG, "r: %f, p: %f, y: %f\n\r", r, p, y);

                gpio_pin6_exti = false;
            }
        }
    }

    void test_mpu6050() noexcept
    {
        auto i2c_device = I2CDevice{.i2c_bus = &hi2c1, .dev_address = MPU6050_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto mpu6050 = MPU6050{std::move(i2c_device),
                               MPU6050_FREQ,
                               MPU6050_GYRO_RANGE,
                               MPU6050_ACCEL_RANGE,
                               MPU6050_DLPF,
                               MPU6050_DHPF};

        auto mpu6050_dmp = MPU6050_DMP{std::move(mpu6050)};

        while (1) {
            if (gpio_pin6_exti) {
                auto const& [r, p, y] = mpu6050_dmp.get_roll_pitch_yaw().value();
                LOG(TAG, "roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);

                gpio_pin6_exti = false;
            }
        }
    }

    void test_a4988_1() noexcept
    {
        auto pwm_device = PWMDevice{.timer = &htim1, .channel_mask = TIM_CHANNEL_1};

        auto a4988 = A4988{std::move(pwm_device), MS1_1, MS2_1, MS3_1, RESET_1, SLEEP_1, DIR_1, EN_1};

        auto step_driver = StepDriver{.driver = std::move(a4988), .steps_per_360 = STEPS_PER_360};

        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            step_driver.set_speed(i += 1.0F, DT);
            tim2_period_elapsed = false;

            if (tim1_pulse_finished) {
                step_driver.update_step_count();

                tim1_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            }
        }
    }

    void test_a4988_2() noexcept
    {
        auto pwm_device = PWMDevice{.timer = &htim3, .channel_mask = TIM_CHANNEL_1};

        auto a4988 = A4988{std::move(pwm_device), MS1_2, MS2_2, MS3_2, RESET_2, SLEEP_2, DIR_2, EN_2};

        auto step_driver = StepDriver{.driver = std::move(a4988), .steps_per_360 = STEPS_PER_360};

        HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            step_driver.set_speed(i += 1.0F, DT);

            tim2_period_elapsed = false;

            if (tim3_pulse_finished) {
                step_driver.update_step_count();

                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }
        }
    }

    void test_segway() noexcept
    {
        auto pwm_device_1 = PWMDevice{.timer = &htim1, .channel_mask = TIM_CHANNEL_1};

        auto a4988_1 = A4988{std::move(pwm_device_1), MS1_1, MS2_1, MS3_1, RESET_1, SLEEP_1, DIR_1, EN_1};

        auto step_driver_1 = StepDriver{.driver = std::move(a4988_1), .steps_per_360 = STEPS_PER_360};

        auto pwm_device_2 = PWMDevice{.timer = &htim3, .channel_mask = TIM_CHANNEL_1};

        auto a4988_2 = A4988{std::move(pwm_device_2), MS1_2, MS2_2, MS3_2, RESET_2, SLEEP_2, DIR_2, EN_2};

        auto step_driver_2 = StepDriver{.driver = std::move(a4988_2), .steps_per_360 = STEPS_PER_360};

        auto i2c_device = I2CDevice{.i2c_bus = &hi2c1, .dev_address = MPU6050_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto mpu6050 = MPU6050{std::move(i2c_device),
                               MPU6050_FREQ,
                               MPU6050_GYRO_RANGE,
                               MPU6050_ACCEL_RANGE,
                               MPU6050_DLPF,
                               MPU6050_DHPF};

        auto sensor = IMU{std::in_place_type<MPU6050_DMP>, std::move(mpu6050)};

        auto regulator = SFR{};

        auto observer = SFO{};

        auto wheels =
            std::array{Wheel{.type = WheelType::LEFT,
                             .driver = WheelDriver{.driver = std::move(step_driver_1), .wheel_radius = 3.0F}},
                       Wheel{.type = WheelType::RIGHT,
                             .driver = WheelDriver{.driver = std::move(step_driver_2), .wheel_radius = 3.0F}}};

        auto config =
            Config{.Kx = {10.0, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F}, .Ki = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}};

        auto segway = Segway{.sensor = std::move(sensor),
                             .regulator = regulator,
                             .observer = observer,
                             .config = config,
                             .wheels = std::move(wheels)};

        HAL_TIM_Base_Start_IT(&htim2);
        // HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
        // HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        while (1) {
            if (gpio_pin6_exti) {
                segway.run_segway(X_REF, DT);
                gpio_pin6_exti = false;
                HAL_TIM_Base_Start_IT(&htim2);
            }

            // if (tim3_pulse_finished) {
            //     segway.update_step_count(WheelType::RIGHT);
            //     tim3_pulse_finished = false;
            //     HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            // }

            // if (tim1_pulse_finished) {
            //     segway.update_step_count(WheelType::LEFT);
            //     tim1_pulse_finished = false;
            //     HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
            // }

            // if (i2c1_error) {
            //     i2c1_error = false;
            //     HAL_I2C_DeInit(&hi2c1);
            //     HAL_I2C_Init(&hi2c1);
            // }
        }
    }

}; // namespace Segway