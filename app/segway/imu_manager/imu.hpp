#ifndef IMU_HPP
#define IMU_HPP

#include "icm20948_dmp.hpp"
#include "mpu6050_dmp.hpp"
#include <stdfloat>
#include <variant>

namespace segway {

    using namespace icm20948;
    using namespace mpu6050;

    using IMU = std::variant<ICM20948_DMP, MPU6050_DMP>;

    void initialize_imu(IMU& imu) noexcept;
    void deinitialize_imu(IMU& imu) noexcept;

    std::float64_t get_tilt_angle(IMU& imu) noexcept;
    std::float64_t get_rotation_angle(IMU& imu) noexcept;

}; // namespace segway

#endif // IMU_HPP