#ifndef MPU6050_CONFIG_HPP
#define MPU6050_CONFIG_HPP

#include "i2c_device.hpp"
#include "mpu6050_registers.hpp"
#include "quaternion3d.hpp"
#include "vector3d.hpp"
#include <cstdint>

namespace MPU6050 {

    using I2CDevice = STM32_Utility::I2CDevice;

    template <typename T>
    using Vec3D = Utility::Vector3D<T>;

    template <typename T>
    using Quat3D = Utility::Quaternion3D<T>;

    enum struct DevAddress : std::uint16_t {
        AD0_LOW = 0x68,
        AD0_HIGH = 0x69,
    };

    enum struct GyroRange : std::uint8_t {
        GYRO_FS_250 = 0x00,
        GYRO_FS_500 = 0x01,
        GYRO_FS_1000 = 0x02,
        GYRO_FS_2000 = 0x03,
    };

    enum struct AccelRange : std::uint8_t {
        ACCEL_FS_2 = 0x00,
        ACCEL_FS_4 = 0x01,
        ACCEL_FS_8 = 0x02,
        ACCEL_FS_16 = 0x03,
    };

    enum struct ExtSync : std::uint8_t {
        DISABLED = 0x0,
        TEMP_OUT_L = 0x1,
        GYRO_XOUT_L = 0x2,
        GYRO_YOUT_L = 0x3,
        GYRO_ZOUT_L = 0x4,
        ACCEL_XOUT_L = 0x5,
        ACCEL_YOUT_L = 0x6,
        ACCEL_ZOUT_L = 0x7,
    };

    enum struct DLPF : std::uint8_t {
        BW_256 = 0x00,
        BW_188 = 0x01,
        BW_98 = 0x02,
        BW_42 = 0x03,
        BW_20 = 0x04,
        BW_10 = 0x05,
        BW_5 = 0x06,
    };

    enum struct DHPF : std::uint8_t {
        DHPF_RESET = 0x00,
        DHPF_5 = 0x01,
        DHPF_2P5 = 0x02,
        DHPF_1P25 = 0x03,
        DHPF_0P63 = 0x04,
        DHPF_HOLD = 0x07,
    };

    enum struct ClockDiv : std::uint8_t {
        DIV_500 = 0x9,
        DIV_471 = 0xA,
        DIV_444 = 0xB,
        DIV_421 = 0xC,
        DIV_400 = 0xD,
        DIV_381 = 0xE,
        DIV_364 = 0xF,
        DIV_348 = 0x0,
        DIV_333 = 0x1,
        DIV_320 = 0x2,
        DIV_308 = 0x3,
        DIV_296 = 0x4,
        DIV_286 = 0x5,
        DIV_276 = 0x6,
        DIV_267 = 0x7,
        DIV_258 = 0x8,
    };

    enum struct IntMode : std::uint8_t {
        ACTIVEHIGH = 0x00,
        ACTIVELOW = 0x01,
    };

    enum struct IntDrive : std::uint8_t {
        PUSHPULL = 0x00,
        OPENDRAIN = 0x01,
    };

    enum struct IntLatch : std::uint8_t {
        PULSE50US = 0x00,
        WAITCLEAR = 0x01,
    };

    enum struct IntClear : std::uint8_t {
        STATUSREAD = 0x00,
        ANYREAD = 0x01,
    };

    enum struct DetectDecrement : std::uint8_t {
        DECREMENT_RESET = 0x0,
        DECREMENT_1 = 0x1,
        DECREMENT_2 = 0x2,
        DECREMENT_4 = 0x3,
    };

    enum struct Delay : std::uint8_t {
        DELAY_3MS = 0b11,
        DELAY_2MS = 0b10,
        DELAY_1MS = 0b01,
        NO_DELAY = 0b00,
    };

    enum struct Clock : std::uint8_t {
        INTERNAL = 0x00,
        PLL_XGYRO = 0x01,
        PLL_YGYRO = 0x02,
        PLL_ZGYRO = 0x03,
        PLL_EXT32K = 0x04,
        PLL_EXT19M = 0x05,
        KEEP_RESET = 0x07,
    };

    enum struct WakeFreq : std::uint8_t {
        FREQ_1P25 = 0x0,
        FREQ_5 = 0x1,
        FREQ_20 = 0x2,
        FREQ_40 = 0x3,
    };

    constexpr auto PI = std::numbers::pi_v<std::float64_t>;

    constexpr auto GYRO_OUTPUT_RATE_DLPF_EN_HZ = 1000U;
    constexpr auto GYRO_OUTPUT_RATE_DLPF_DIS_HZ = 8000U;
    constexpr auto ACCEL_OUTPUT_RATE_HZ = 1000U;

    inline std::float64_t gyro_range_to_scale(GyroRange const gyro_range) noexcept
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

    inline std::float64_t accel_range_to_scale(AccelRange const accel_range) noexcept
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

    inline std::uint8_t get_sampling_divider(std::uint32_t const sampling_rate, DLPF const dlpf) noexcept
    {
        if (dlpf == DLPF::BW_256) {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_DIS_HZ / sampling_rate) - 1U);
        } else {
            return static_cast<std::uint8_t>((GYRO_OUTPUT_RATE_DLPF_EN_HZ / sampling_rate) - 1U);
        }
    }

    inline std::uint8_t slave_num_to_address(std::uint8_t const num) noexcept
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

    inline std::uint8_t slave_num_to_register(std::uint8_t const num) noexcept
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

    inline std::uint8_t slave_num_to_control(std::uint8_t const num) noexcept
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

    inline std::uint8_t slave_num_to_output_byte(std::uint8_t const num) noexcept
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

}; // namespace MPU6050

#endif // MPU6050_CONFIG_HPP