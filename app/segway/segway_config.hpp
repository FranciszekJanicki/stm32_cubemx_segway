#ifndef SEGWAY_CONFIG_HPP
#define SEGWAY_CONFIG_HPP

#include "a4988.hpp"
#include "gpio.hpp"
#include "i2c_device.hpp"
#include "icm20948_dmp.hpp"
#include "imu.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "nvic.hpp"
#include "pwm_device.hpp"
#include "segway.hpp"
#include "sfo.hpp"
#include "sfr.hpp"
#include "step_driver.hpp"
#include "wheel.hpp"

namespace Segway {

    using namespace Utility;
    using namespace STM32_Utility;
    using namespace A4988;
    using namespace MPU6050;
    using namespace ICM20948;
    using namespace StepDriver;

    using A4988 = ::A4988::A4988;
    using StepDriver = ::StepDriver::StepDriver;

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

    constexpr auto DOT_TILT = 0.0F;
    constexpr auto TILT = 0.0F;
    constexpr auto DOT_ROTATION = 0.0F;
    constexpr auto ROTATION = 0.0F;
    constexpr auto POSITION = 0.0F;
    constexpr auto WHELLS_SPEED = 0.0F;
    constexpr auto DT = 1.0F / ICM20948_FREQ;

    constexpr auto STEPS_PER_360 = 200U;

}; // namespace Segway

#endif // SEGWAY_CONFIG_HPP