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

namespace Segway {

    using namespace MPU6050;
    using namespace ICM20948;
    using namespace StepDriver;

    using A4988 = ::A4988::A4988;
    using StepDriver = ::StepDriver::StepDriver;
    using SFR = ::Utility::SFR<std::float32_t, 6UL, 2UL>;
    using SFO = ::Utility::SFO<std::float32_t, 6UL, 2UL>;
    using ICM20948_DMP = ::ICM20948::ICM20948_DMP;
    using MPU6050 = ::MPU6050::MPU6050;
    using MPU6050_DMP = ::MPU6050::MPU6050_DMP;
    using IMU = std::variant<ICM20948_DMP, MPU6050_DMP>;

    struct Config {
        std::array<std::float32_t, 6UL> Kx = {};
        std::array<std::float32_t, 6UL> Ki = {};

        std::array<std::float32_t, 6UL> prev_x = {};
        std::array<std::float32_t, 6UL> prev_e = {};
        std::array<std::float32_t, 6UL> int_e = {};
        std::array<std::float32_t, 6UL> x = {};
        std::array<std::float32_t, 6UL> e = {};
        std::array<std::float32_t, 2UL> u = {};
    };

    constexpr auto MS1_1 = GPIO::PB4;
    constexpr auto MS2_1 = GPIO::PB5;
    constexpr auto MS3_1 = GPIO::PB6;
    constexpr auto DIR_1 = GPIO::PB7;
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

    constexpr auto X_REF = std::array{0.0F32, 0.0F32, 0.0F32, 0.0F32, 0.0F32, 0.0F32};
    constexpr auto DT = 1.0F / ICM20948_FREQ;

    constexpr auto STEPS_PER_360 = 200U;

}; // namespace Segway

#endif // SEGWAY_CONFIG_HPP