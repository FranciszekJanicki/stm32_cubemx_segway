#include "unit_test.hpp"
#include "gpio.h"
#include "i2c.h"
#include "imu.hpp"
#include "log.hpp"
#include "segway.hpp"
#include "segway_config.hpp"
#include "tim.h"
#include "timers.hpp"
#include "wheels.hpp"

namespace {

    constexpr auto TAG = "Unit test";

}; // namespace

namespace segway {

    void test(TestType const test_type) noexcept
    {
        switch (test_type) {
            case TestType::ICM20948:
                test_icm20948();
            case TestType::MPU6050:
                test_mpu6050();
            case TestType::A4988_1:
                test_a4988_1();
            case TestType::A4988_2:
                test_a4988_2();
            case TestType::SEGWAY:
                test_segway();
            default:
                break;
        }
    }

    void test_icm20948() noexcept
    {
        auto i2c_device = stm32_utility::I2CDevice{.i2c_bus = &hi2c1, .dev_address = ICM20948_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto icm20948_dmp = icm20948::ICM20948_DMP{std::move(i2c_device)};

        while (1) {
            if (nvic_mask.data_ready) {
                nvic_mask.data_ready = false;

                auto const& [r, p, y] = icm20948_dmp.get_roll_pitch_yaw().value();
                LOG(TAG, "r: %f, p: %f, y: %f", r, p, y);
            }
        }
    }

    void test_mpu6050() noexcept
    {
        auto i2c_device = stm32_utility::I2CDevice{.i2c_bus = &hi2c1, .dev_address = MPU6050_I2C_ADDRESS};
        i2c_device.bus_scan();

        auto mpu6050 = mpu6050::MPU6050{std::move(i2c_device),
                                        MPU6050_FREQ,
                                        MPU6050_GYRO_RANGE,
                                        MPU6050_ACCEL_RANGE,
                                        MPU6050_DLPF,
                                        MPU6050_DHPF};

        auto mpu6050_dmp = mpu6050::MPU6050_DMP{std::move(mpu6050)};

        while (1) {
            if (nvic_mask.data_ready) {
                nvic_mask.data_ready = false;

                auto const& [r, p, y] = mpu6050_dmp.get_roll_pitch_yaw().value();
                LOG(TAG, "roll: %f, pitch: %f, yaw: %f", r, p, y);
            }
        }
    }

    void test_a4988_1() noexcept
    {
        auto pwm_device = stm32_utility::PWMDevice{.timer = &htim1, .channel_mask = TIM_CHANNEL_1};

        auto a4988 = a4988::A4988{.config = a4988::Config{.pwm_device = std::move(pwm_device),
                                                          .pin_ms1 = MS1_1,
                                                          .pin_ms2 = MS2_1,
                                                          .pin_ms3 = MS3_1,
                                                          .pin_reset = RESET_1,
                                                          .pin_sleep = SLEEP_1,
                                                          .pin_dir = DIR_1,
                                                          .pin_enable = EN_1}};

        auto step_driver = step_driver::StepDriver{.driver = std::move(a4988), .steps_per_360 = STEPS_PER_360};

        step_driver.initialize();

        auto i = 0.0F64;

        while (1) {
            step_driver.set_speed(i += 1.0F64, DT);
            HAL_Delay(1);

            if (nvic_mask.motor1_pwm_pulse) {
                nvic_mask.motor1_pwm_pulse = false;

                step_driver.update_step_count();
            }
        }
    }

    void test_a4988_2() noexcept
    {
        auto pwm_device = stm32_utility::PWMDevice{.timer = &htim3, .channel_mask = TIM_CHANNEL_1};

        auto a4988 = a4988::A4988{.config = a4988::Config{.pwm_device = std::move(pwm_device),
                                                          .pin_ms1 = MS1_2,
                                                          .pin_ms2 = MS2_2,
                                                          .pin_ms3 = MS3_2,
                                                          .pin_reset = RESET_2,
                                                          .pin_sleep = SLEEP_2,
                                                          .pin_dir = DIR_2,
                                                          .pin_enable = EN_2}};

        auto step_driver = step_driver::StepDriver{.driver = std::move(a4988), .steps_per_360 = STEPS_PER_360};

        step_driver.initialize();

        auto i = 0.0F;

        while (1) {
            step_driver.set_speed(i += 1.0F, DT);
            HAL_Delay(1);

            if (nvic_mask.motor2_pwm_pulse) {
                nvic_mask.motor2_pwm_pulse = false;

                step_driver.update_step_count();
            }
        }
    }

    void test_segway() noexcept
    {
        auto pwm_device_1 = stm32_utility::PWMDevice{.timer = &htim1, .channel_mask = TIM_CHANNEL_1};

        auto pwm_device_2 = stm32_utility::PWMDevice{.timer = &htim3, .channel_mask = TIM_CHANNEL_1};

        auto a4988_1 = a4988::A4988{.config = a4988::Config{.pwm_device = std::move(pwm_device_1),
                                                            .pin_ms1 = MS1_1,
                                                            .pin_ms2 = MS2_1,
                                                            .pin_ms3 = MS3_1,
                                                            .pin_reset = RESET_1,
                                                            .pin_sleep = SLEEP_1,
                                                            .pin_dir = DIR_1,
                                                            .pin_enable = EN_1}};

        auto a4988_2 = a4988::A4988{.config = a4988::Config{.pwm_device = std::move(pwm_device_2),
                                                            .pin_ms1 = MS1_2,
                                                            .pin_ms2 = MS2_2,
                                                            .pin_ms3 = MS3_2,
                                                            .pin_reset = RESET_2,
                                                            .pin_sleep = SLEEP_2,
                                                            .pin_dir = DIR_2,
                                                            .pin_enable = EN_2}};

        auto step_driver_1 = step_driver::StepDriver{.driver = std::move(a4988_1), .steps_per_360 = STEPS_PER_360};

        auto step_driver_2 = step_driver::StepDriver{.driver = std::move(a4988_2), .steps_per_360 = STEPS_PER_360};

        auto left_wheel = segway::Wheel{
            .type = segway::WheelType::LEFT,
            .driver = segway::WheelDriver{.driver = std::move(step_driver_1), .wheel_radius = WHEEL_RADIUS}};

        auto right_wheel = segway::Wheel{
            .type = segway::WheelType::RIGHT,
            .driver = segway::WheelDriver{.driver = std::move(step_driver_2), .wheel_radius = WHEEL_RADIUS}};

        auto wheels = segway::Wheels{std::move(left_wheel), std::move(right_wheel)};

        auto i2c_device = stm32_utility::I2CDevice{.i2c_bus = &hi2c1, .dev_address = MPU6050_I2C_ADDRESS};

        auto mpu6050 = mpu6050::MPU6050{std::move(i2c_device),
                                        MPU6050_FREQ,
                                        MPU6050_GYRO_RANGE,
                                        MPU6050_ACCEL_RANGE,
                                        MPU6050_DLPF,
                                        MPU6050_DHPF};

        auto imu = mpu6050::MPU6050_DMP{std::move(mpu6050)};

        auto regulator = PID{.kP = PID_KP, .kI = PID_KI, .kD = PID_KD, .tD = PID_TD, .kC = PID_KC, .sat = PID_SAT};

        auto config = Config{.wheel_distance = WHEEL_DIST,
                             .tilt_fault_thresh_low = TILT_FAULT_THRESH_LOW,
                             .tilt_fault_thresh_high = TILT_FAULT_THRESH_HIGH,
                             .imu_fault_thresh_low = IMU_FAULT_THRESH_LOW,
                             .imu_fault_thresh_high = IMU_FAULT_THRESH_HIGH,
                             .wheel_fault_thresh_low = WHEEL_FAULT_THRESH_LOW,
                             .wheel_fault_thresh_high = WHEEL_FAULT_THRESH_HIGH};

        auto segway =
            Segway{.imu = std::move(imu), .wheels = std::move(wheels), .regulator = regulator, .config = config};

        segway.initialize();

        sampling_timer_start();
        motor1_step_timer_start();
        motor2_step_timer_start();

        while (1) {
            if (nvic_mask.sampling_timer) {
                nvic_mask.sampling_timer = false;

                segway.run_segway_pid(PID_Y_REF, DT);
                sampling_timer_start();
            }

            if (nvic_mask.motor1_step_timer) {
                nvic_mask.motor1_step_timer = false;
                gpio_toggle_pin(STEP_1);
                LOG(TAG, "MOTOR1");
                motor1_step_timer_start();
            }

            if (nvic_mask.motor2_step_timer) {
                nvic_mask.motor2_step_timer = false;
                LOG(TAG, "MOTOR2");
                gpio_toggle_pin(STEP_2);
                motor2_step_timer_start();
            }

            // if (nvic_mask.motor1_pwm_pulse) {
            //     nvic_mask.motor1_pwm_pulse = false;

            //     segway.update_step_count(WheelType::RIGHT);
            //     motor1_pwm_timer_start();
            // }

            // if (nvic_mask.motor2_pwm_pulse) {
            //     nvic_mask.motor2_pwm_pulse = false;

            //     segway.update_step_count(WheelType::LEFT);
            //     motor2_pwm_timer_start();
            // }

            // if (nvic_mask.i2c_error) {
            //     nvic_mask.i2c_error = false;
            //     HAL_I2C_DeInit(&hi2c1);
            //     HAL_I2C_Init(&hi2c1);
            // }
        }
    }
}; // namespace segway