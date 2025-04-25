#ifndef SEGWAY_CONFIG_HPP
#define SEGWAY_CONFIG_HPP

#include "a4988.hpp"
#include "gpio.hpp"
#include "i2c_device.hpp"
#include "icm20948_dmp.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "nvic.hpp"
#include "pwm_device.hpp"
#include "sfo.hpp"
#include "sfr.hpp"
#include "step_driver.hpp"
#include "wheel.hpp"

namespace segway {

    using namespace mpu6050;
    using namespace icm20948;
    using namespace step_driver;

    using PID = ::utility::PID<std::float64_t>;

    struct LQR {
        std::array<std::float64_t, 6UL> Kx = {};
        std::array<std::float64_t, 6UL> Ki = {};

        std::array<std::float64_t, 6UL> prev_x = {};
        std::array<std::float64_t, 6UL> prev_e = {};
        std::array<std::float64_t, 6UL> int_e = {};
        std::array<std::float64_t, 6UL> x = {};
        std::array<std::float64_t, 6UL> e = {};
        std::array<std::float64_t, 2UL> u = {};
    };

    using Regulator = std::variant<PID, LQR>;

    struct Config {
        std::float64_t wheel_distance = {};
        std::float64_t tilt_fault_thresh_low = {};
        std::float64_t tilt_fault_thresh_high = {};
        std::float64_t imu_fault_thresh_low = {};
        std::float64_t imu_fault_thresh_high = {};
        std::float64_t wheel_fault_thresh_low = {};
        std::float64_t wheel_fault_thresh_high = {};
    };

    constexpr auto MS1_1 = GPIO::PA11;
    constexpr auto MS2_1 = GPIO::PA10;
    constexpr auto MS3_1 = GPIO::PA9;
    constexpr auto DIR_1 = GPIO::PB15;
    constexpr auto EN_1 = GPIO::NC;
    constexpr auto SLEEP_1 = GPIO::NC;
    constexpr auto RESET_1 = GPIO::NC;

    constexpr auto MS1_2 = GPIO::PA3;
    constexpr auto MS2_2 = GPIO::PA4;
    constexpr auto MS3_2 = GPIO::PA5;
    constexpr auto DIR_2 = GPIO::PA7;
    constexpr auto EN_2 = GPIO::NC;
    constexpr auto SLEEP_2 = GPIO::NC;
    constexpr auto RESET_2 = GPIO::NC;

    constexpr auto MPU6050_FREQ = 200UL;
    constexpr auto MPU6050_I2C_ADDRESS = std::to_underlying(DevAddress::AD0_LOW);
    constexpr auto MPU6050_GYRO_RANGE = GyroRange::GYRO_FS_250;
    constexpr auto MPU6050_ACCEL_RANGE = AccelRange::ACCEL_FS_2;
    constexpr auto MPU6050_DLPF = DLPF::BW_42;
    constexpr auto MPU6050_DHPF = DHPF::DHPF_RESET;

    constexpr auto ICM20948_FREQ = 200UL;
    constexpr auto ICM20948_I2C_ADDRESS = ICM_20948_I2C_ADDR_AD0;

    constexpr auto LQR_X_REF = std::array{0.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64};
    constexpr auto LQR_KI = std::array{1.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64};
    constexpr auto LQR_KX = std::array{-7.14F64, -1.900F64, -0.0007F64, -0.0015F64, -0.707F64, -0.8803F64};

    constexpr auto PID_Y_REF = 4.5F64;
    constexpr auto PID_KP = 5.0F64;
    constexpr auto PID_KI = 0.0F64;
    constexpr auto PID_KD = 0.0F64;
    constexpr auto PID_KC = 0.0F64;
    constexpr auto PID_TD = 0.0001F64;
    constexpr auto PID_SAT = 1200.0F64;

    constexpr auto DT = 1.0F64 / ICM20948_FREQ;

    constexpr auto STEPS_PER_360 = 200U;
    constexpr auto WHEEL_DIST = 10.0F64;
    constexpr auto WHEEL_RADIUS = 1.0F64;
    constexpr auto TILT_FAULT_THRESH_LOW = 45.0F64;
    constexpr auto TILT_FAULT_THRESH_HIGH = 30.0F64;
    constexpr auto IMU_FAULT_THRESH_LOW = 160.0F64;
    constexpr auto IMU_FAULT_THRESH_HIGH = 180.0F64;
    constexpr auto WHEEL_FAULT_THRESH_HIGH = 1200.0;
    constexpr auto WHEEL_FAULT_THRESH_LOW = 800.0;

}; // namespace segway

#endif // SEGWAY_CONFIG_HPP