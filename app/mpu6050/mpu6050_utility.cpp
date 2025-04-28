#include "mpu6050_utility.hpp"

namespace mpu6050 {

    std::float64_t gyro_range_to_scale(GyroRange const gyro_range) noexcept
    {
        switch (gyro_range) {
            case GyroRange::GYRO_FS_250:
                return 131.0f;
            case GyroRange::GYRO_FS_500:
                return 65.5f;
            case GyroRange::GYRO_FS_1000:
                return 32.8f;
            case GyroRange::GYRO_FS_2000:
                return 16.4f;
            default:
                return 0.0f;
        }
    }

    std::float64_t accel_range_to_scale(AccelRange const accel_range) noexcept
    {
        switch (accel_range) {
            case AccelRange::ACCEL_FS_2:
                return 16384.0f;
            case AccelRange::ACCEL_FS_4:
                return 8192.0f;
            case AccelRange::ACCEL_FS_8:
                return 4096.0f;
            case AccelRange::ACCEL_FS_16:
                return 2048.0f;
            default:
                return 0.0f;
        }
    }

    std::uint8_t get_sampling_divider(std::uint32_t const sampling_rate, DLPF const dlpf) noexcept
    {
        if (dlpf == DLPF::BW_256) {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_DIS_HZ / sampling_rate) - 1U);
        } else {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_EN_HZ / sampling_rate) - 1U);
        }
    }

    std::uint8_t slave_num_to_address(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RA::I2C_SLV0_ADDR);
            case 1:
                return std::to_underlying(RA::I2C_SLV1_ADDR);
            case 2:
                return std::to_underlying(RA::I2C_SLV2_ADDR);
            case 3:
                return std::to_underlying(RA::I2C_SLV3_ADDR);
            default:
                std::unreachable();
        }
    }

    std::uint8_t slave_num_to_register(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RA::I2C_SLV0_REG);
            case 1:
                return std::to_underlying(RA::I2C_SLV1_REG);
            case 2:
                return std::to_underlying(RA::I2C_SLV2_REG);
            case 3:
                return std::to_underlying(RA::I2C_SLV3_REG);
            default:
                std::unreachable();
        }
    }

    std::uint8_t slave_num_to_control(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RA::I2C_SLV0_CTRL);
            case 1:
                return std::to_underlying(RA::I2C_SLV1_CTRL);
            case 2:
                return std::to_underlying(RA::I2C_SLV2_CTRL);
            case 3:
                return std::to_underlying(RA::I2C_SLV3_CTRL);
            default:
                std::unreachable();
        }
    }

    std::uint8_t slave_num_to_output_byte(std::uint8_t const num) noexcept
    {
        switch (num) {
            case 0:
                return std::to_underlying(RA::I2C_SLV0_DO);
            case 1:
                return std::to_underlying(RA::I2C_SLV1_DO);
            case 2:
                return std::to_underlying(RA::I2C_SLV2_DO);
            case 3:
                return std::to_underlying(RA::I2C_SLV3_DO);
            default:
                std::unreachable();
        }
    }

}; // namespace mpu6050