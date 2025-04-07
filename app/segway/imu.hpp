#include "icm20948_dmp.hpp"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include <variant>

namespace Segway {

    using namespace ICM20948;
    using ICM20948_DMP = ::ICM20948::ICM20948_DMP;

    using namespace MPU6050;
    using MPU6050 = ::MPU6050::MPU6050;
    using MPU6050_DMP = ::MPU6050::MPU6050_DMP;

    using IMU = std::variant<ICM20948_DMP, MPU6050_DMP>;

}; // namespace Segway