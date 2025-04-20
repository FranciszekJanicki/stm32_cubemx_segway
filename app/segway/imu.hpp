#ifndef IMU_HPP
#define IMU_HPP

#include "icm20948_dmp.hpp"
#include "mpu6050_dmp.hpp"
#include "segway_config.hpp"
#include <variant>

namespace Segway {

    using namespace ICM20948;
    using namespace MPU6050;
    using MPU6050 = ::MPU6050::MPU6050;

    struct IMU {
    public:
        std::float32_t get_tilt(this IMU& self) noexcept;
        std::float32_t get_rotation(this IMU& self) noexcept;

        std::variant<ICM20948_DMP, MPU6050_DMP> sensor = {};
    };

}; // namespace Segway

#endif // IMU_HPP