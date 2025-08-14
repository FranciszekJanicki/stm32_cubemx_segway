#ifndef MPU6050_UTILITY_HPP
#define MPU6050_UTILITY_HPP

#include "mpu6050_config.hpp"

namespace mpu6050 {

    std::float64_t gyro_range_to_scale(GyroRange gyro_range) noexcept;
    std::float64_t accel_range_to_scale(AccelRange accel_range) noexcept;

    std::uint8_t get_sampling_divider(std::uint32_t sampling_rate, DLPF dlpf) noexcept;

    std::uint8_t slave_num_to_address(std::uint8_t num) noexcept;
    std::uint8_t slave_num_to_register(std::uint8_t num) noexcept;
    std::uint8_t slave_num_to_control(std::uint8_t num) noexcept;
    std::uint8_t slave_num_to_output_byte(std::uint8_t num) noexcept;

}; // namespace mpu6050

#endif // MPU6050_UTILITY_HPP