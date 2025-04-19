#ifndef MPU6050_DMP_CONFIG_HPP
#define MPU6050_DMP_CONFIG_HPP

#include "mpu6050_config.hpp"
#include <numbers>

namespace MPU6050 {

    constexpr auto DMP_MEMORY_BANKS = 8;
    constexpr auto DMP_MEMORY_BANK_SIZE = 256UL;
    constexpr auto DMP_MEMORY_CHUNK_SIZE = 16UL;
    constexpr auto FIFO_DEFAULT_TIMEOUT = 11000;
    constexpr auto FIFO_MAX_COUNT = 1024UL;
    constexpr auto QUAT_SCALE = static_cast<std::float64_t>(1ULL << 30ULL);

} // namespace MPU6050

#endif // MPU6050_DMP_CONFIG_HPP