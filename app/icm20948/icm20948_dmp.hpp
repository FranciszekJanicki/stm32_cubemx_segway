#ifndef ICM20948_DMP_HPP
#define ICM20948_DMP_HPP

#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"
#include "ICM_20948_C.h"
#include "ICM_20948_DMP.h"
#include "ICM_20948_ENUMERATIONS.h"
#include "ICM_20948_REGISTERS.h"
#include "i2c_device.hpp"
#include "quaternion3d.hpp"
#include "utility.hpp"
#include "vector3d.hpp"
#include <optional>

namespace icm20948 {

    template <typename T>
    using Vec3D = utility::Vector3D<T>;

    template <typename T>
    using Quat3D = utility::Quaternion3D<T>;

    using I2CDevice = stm32_utility::I2CDevice;

    constexpr auto ACCEL_SCALE = 8.192F / 1000.0F * 9.81F;
    constexpr auto GYRO_SCALE = 16.4F;
    constexpr auto QUAT_SCALE = static_cast<std::float64_t>(1UL << 30UL);

    struct ICM20948_DMP {
    public:
        void initialize() noexcept;
        void deinitialize() noexcept;

        std::optional<Vec3D<std::float64_t>> get_roll_pitch_yaw() noexcept;
        std::optional<std::float64_t> get_roll() noexcept;
        std::optional<std::float64_t> get_pitch() noexcept;
        std::optional<std::float64_t> get_yaw() noexcept;

        std::optional<Quat3D<std::float64_t>> get_quaternion_scaled() noexcept;
        std::optional<Quat3D<std::int32_t>> get_quaternion_raw() noexcept;

        bool initialized = false;

        I2CDevice i2c_device = {};

        ICM_20948_Serif_t icm_20948_serif = {};
        ICM_20948_Device_t icm_20948_device = {};

    private:
        void initialize_dmp() noexcept;
    };

}; // namespace icm20948

#endif // ICM20948_DMP_HPP