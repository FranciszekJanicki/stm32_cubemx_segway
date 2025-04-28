#ifndef SEGWAY_GLOBAL_CONFIG_HPP
#define SEGWAY_GLOBAL_CONFIG_HPP

#include "a4988.hpp"
#include "gpio.hpp"
#include "i2c_device.hpp"
#include "mpu6050.hpp"
#include "nvic.hpp"
#include "pid.hpp"
#include "pwm_device.hpp"
#include "sfo.hpp"
#include "sfr.hpp"
#include "step_driver.hpp"
#include "wheel.hpp"

namespace segway {

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

    constexpr auto LQR_X_REF = std::array{0.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64};
    constexpr auto LQR_KI = std::array{1.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64, 0.0F64};
    constexpr auto LQR_KX = std::array{-7.14F64, -1.900F64, -0.0007F64, -0.0015F64, -0.707F64, -0.8803F64};

    constexpr auto PID_Y_REF = 2.75F64;
    constexpr auto PID_KP = 15.0F64;
    constexpr auto PID_KI = 0.0F64;
    constexpr auto PID_KD = 0.0F64;
    constexpr auto PID_KC = 0.0F64;
    constexpr auto PID_TD = 0.0001F64;
    constexpr auto PID_SAT = 1000.0F64;

    constexpr auto TILT_FAULT_THRESH_LOW = 45.0F64;
    constexpr auto TILT_FAULT_THRESH_HIGH = 30.0F64;
    constexpr auto IMU_FAULT_THRESH_LOW = 160.0F64;
    constexpr auto IMU_FAULT_THRESH_HIGH = 180.0F64;
    constexpr auto WHEEL_FAULT_THRESH_HIGH = PID_SAT;
    constexpr auto WHEEL_FAULT_THRESH_LOW = PID_SAT * 0.8;

}; // namespace segway

#endif // SEGWAY_GLOBAL_CONFIG_HPP