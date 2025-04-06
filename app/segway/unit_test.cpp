#include "unit_test.hpp"
#include "segway_config.hpp"

namespace Segway {

    void test_icm20948() noexcept
    {
        using namespace ICM20948;

        auto i2c_device = I2CDevice{&hi2c1, ICM20948_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto icm20948 = ICM20948{std::move(i2c_device)};

        while (1) {
            if (gpio_pin6_exti) {
                auto const& [r, p, y] = icm20948.get_roll_pitch_yaw().value();
                std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);
                gpio_pin6_exti = false;
            }
        }
    }

    void test_mpu6050() noexcept
    {
        using namespace MPU6050;

        auto i2c_device = I2CDevice{&hi2c1, MPU6050_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto mpu6050 = MPU6050{std::move(i2c_device),
                               MPU6050_FREQ,
                               MPU6050_GYRO_RANGE,
                               MPU6050_ACCEL_RANGE,
                               MPU6050_DLPF,
                               MPU6050_DHPF};

        auto mpu_dmp = MPU6050_DMP{std::move(mpu6050)};

        while (1) {
            if (gpio_pin6_exti) {
                if (auto const rpy = mpu_dmp.get_roll_pitch_yaw(); rpy.has_value()) {
                    auto const& [r, p, y] = rpy.value();
                    std::printf("roll: %f, pitch: %f, yaw: %f\n\r", r, p, y);
                }

                gpio_pin6_exti = false;
            }
        }
    }

    void test_a4988_1() noexcept
    {
        auto pwm_device = PWMDevice{&htim1, TIM_CHANNEL_1};

        auto a4988 =
            A4988{std::move(pwm_device), MS1_1, MS2_1, MS3_1, RESET_1, SLEEP_1, DIR_1, EN_1};

        auto step_driver =
            Driver{.driver = std::move(a4988), .regulator = {}, .steps_per_360 = STEPS_PER_360};

        HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            step_driver.set_speed(i += 1.0F, SAMPLING_TIME);
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
        auto pwm_device = PWMDevice{&htim3, TIM_CHANNEL_1};

        auto a4988 =
            A4988{std::move(pwm_device), MS1_2, MS2_2, MS3_2, RESET_2, SLEEP_2, DIR_2, EN_2};

        auto driver =
            Driver{.driver = std::move(a4988), .regulator = {}, .steps_per_360 = STEPS_PER_360};

        //   HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto i = 0.0F;

        while (1) {
            driver.set_speed(i += 1.0F, SAMPLING_TIME);

            tim2_period_elapsed = false;

            if (tim3_pulse_finished) {
                driver.update_step_count();

                tim3_pulse_finished = false;
                HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
            }
        }
    }

    void test_segway() noexcept
    {
        auto pwm_device_1 = PWMDevice{&htim1, TIM_CHANNEL_1};

        auto a4988_1 =
            A4988{std::move(pwm_device_1), MS1_1, MS2_1, MS3_1, RESET_1, SLEEP_1, DIR_1, EN_1};

        auto driver_1 =
            Driver{.driver = std::move(a4988_1), .regulator = {}, .steps_per_360 = STEPS_PER_360};

        auto pwm_device_2 = PWMDevice{&htim3, TIM_CHANNEL_1};

        auto a4988_2 =
            A4988{std::move(pwm_device_2), MS1_2, MS2_2, MS3_2, RESET_2, SLEEP_2, DIR_2, EN_2};

        auto driver_2 =
            Driver{.driver = std::move(a4988_2), .regulator = {}, .steps_per_360 = STEPS_PER_360};

        auto i2c_device = I2CDevice{&hi2c1, MPU6050_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto mpu6050 = MPU6050{std::move(i2c_device),
                               MPU6050_FREQ,
                               MPU6050_GYRO_RANGE,
                               MPU6050_ACCEL_RANGE,
                               MPU6050_DLPF,
                               MPU6050_DHPF};

        auto imu = IMU{std::in_place_type<MPU6050_DMP>, std::move(mpu6050)};

        auto angle_regulator = Regulator{.proportion_gain = P,
                                         .integral_gain = I,
                                         .derivative_gain = D,
                                         .saturation = SAT};

        auto driver_channels =
            std::array{DriverChannel{.channel = Channel::CHANNEL_1, .driver = std::move(driver_1)},
                       DriverChannel{.channel = Channel::CHANNEL_2, .driver = std::move(driver_2)}};

        auto segway = Segway{.imu = std::move(imu),
                             .regulator = angle_regulator,
                             .driver_channels = std::move(driver_channels)};

        //  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
        //  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

        auto speed = 100.0F32;

        while (1) {
            // if (gpio_pin6_exti) {
            segway.set_angle(INPUT_ANGLE, SAMPLING_TIME);

            gpio_pin6_exti = false;
            // }
            HAL_Delay(50);

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