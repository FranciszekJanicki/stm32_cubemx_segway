#ifndef MPU6050_CONFIG_HPP
#define MPU6050_CONFIG_HPP

#include "mpu6050_registers.hpp"
#include "quaternion3d.hpp"
#include "vector3d.hpp"
#include <cstdint>
#include <stdfloat>

namespace mpu6050 {

    template <typename T>
    using Vec3D = utility::Vector3D<T>;

    template <typename T>
    using Quat3D = utility::Quaternion3D<T>;

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

    struct Config {
        std::uint32_t sampling_rate;
        GyroRange gyro_range;
        AccelRange accel_range;
        DLPF dlpf_setting;
        DHPF dhpf_setting;
    };

    struct Scale {
        std::float64_t gyro_scale;
        std::float64_t accel_scale;
    };

    struct Interface {
        void* user;
        void (*write_bytes)(void*, std::uint8_t, std::uint8_t*, std::size_t);
        void (*read_bytes)(void*, std::uint8_t, std::uint8_t*, std::size_t);
        void (*delay_ms)(void*, std::uint32_t);
    };

    constexpr auto PI = std::numbers::pi_v<std::float64_t>;

    constexpr auto GYRO_OUTPUT_RATE_DLPF_EN_HZ = 1000U;
    constexpr auto GYRO_OUTPUT_RATE_DLPF_DIS_HZ = 8000U;
    constexpr auto ACCEL_OUTPUT_RATE_HZ = 1000U;

}; // namespace mpu6050

#endif // MPU6050_CONFIG_HPP