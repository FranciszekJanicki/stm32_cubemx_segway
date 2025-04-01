#ifndef MPU6050_DMP_CONFIG_HPP
#define MPU6050_DMP_CONFIG_HPP

#include "mpu6050.hpp"
#include "mpu6050_config.hpp"

namespace MPU6050 {

    inline Vec3D<float> quaternion_to_gravity(Quat3D<float> const& quaternion) noexcept
    {
        return Vec3D<float>{2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y),
                            2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
                            quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y +
                                quaternion.z * quaternion.z};
    }

    inline float quaternion_to_roll(Quat3D<float> const& quaternion) noexcept
    {
        auto const gravity{quaternion_to_gravity(quaternion)};
        return std::atan2(gravity.y, gravity.z);
    }

    inline float quaternion_to_pitch(Quat3D<float> const& quaternion) noexcept
    {
        auto const gravity = quaternion_to_gravity(quaternion);
        auto const pitch = std::atan2(gravity.x, std::sqrt(gravity.y * gravity.y + gravity.z * gravity.z));
        return (gravity.z < 0) ? (pitch > 0 ? 3.1416F - pitch : -3.1416F - pitch) : pitch;
    }

    inline float quaternion_to_yaw(Quat3D<float> const& quaternion) noexcept
    {
        return std::atan2(2 * quaternion.x * quaternion.y - 2 * quaternion.w * quaternion.z,
                          2 * quaternion.w * quaternion.w + 2 * quaternion.x * quaternion.x - 1);
    }

    inline Vec3D<float> quaternion_to_roll_pitch_yaw(Quat3D<float> const& quaternion) noexcept
    {
        auto const gravity = quaternion_to_gravity(quaternion);
        auto const pitch = std::atan2(gravity.x, std::sqrt(gravity.y * gravity.y + gravity.z * gravity.z));

        return Vec3D<float>{std::atan2(gravity.y, gravity.z),
                            (gravity.z < 0) ? (pitch > 0 ? 3.1416F - pitch : -3.1416F - pitch) : pitch,
                            std::atan2(2 * quaternion.x * quaternion.y - 2 * quaternion.w * quaternion.z,
                                       2 * quaternion.w * quaternion.w + 2 * quaternion.x * quaternion.x - 1)};
    }

    constexpr auto DMP_MEMORY_BANKS = 8;
    constexpr auto DMP_MEMORY_BANK_SIZE = 256UL;
    constexpr auto DMP_MEMORY_CHUNK_SIZE = 16UL;
    constexpr auto FIFO_DEFAULT_TIMEOUT = 11000;
    constexpr auto FIFO_MAX_COUNT = 1024UL;

}; // namespace MPU6050

#endif // MPU6050_DMP_CONFIG_HPP