#ifndef ICM20948_HPP
#define ICM20948_HPP

#include "../stm32_utility/i2c_device.hpp"
#include "../utility/utility.hpp"
#include "../utility/vector3d.hpp"
#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"
#include "ICM_20948_C.h"
#include "ICM_20948_DMP.h"
#include "ICM_20948_ENUMERATIONS.h"
#include "ICM_20948_REGISTERS.h"
#include <optional>

namespace ICM20948 {

    template <typename T>
    using Vec3D = Utility::Vector3D<T>;

    using I2CDevice = STM32_Utility::I2CDevice;

    constexpr auto ACCEL_SCALE = 8.192F / 1000.0F * 9.81F;
    constexpr auto GYRO_SCALE = 16.4F;
    constexpr auto QUAT_SCALE = static_cast<float>(2U << 30U);

    struct Config {};

    struct ICM20948 {
    public:
        ICM20948() noexcept = default;
        ICM20948(I2CDevice&& i2c_device) noexcept;

        ICM20948(ICM20948 const& other) = delete;
        ICM20948(ICM20948&& other) noexcept = default;

        ICM20948& operator=(ICM20948 const& other) = delete;
        ICM20948& operator=(ICM20948&& other) noexcept = default;

        ~ICM20948() noexcept;

        void initialize() noexcept;
        void initialize_dmp() noexcept;

        void deinitialize() noexcept;

        std::optional<std::float32_t> get_roll() noexcept;
        std::optional<std::float32_t> get_pitch() noexcept;
        std::optional<std::float32_t> get_yaw() noexcept;
        std::optional<Vec3D<std::float32_t>> get_roll_pitch_yaw() noexcept;

    private:
        bool initialized_{false};

        I2CDevice i2c_device_{};

        ICM_20948_Serif_t icm_20948_serif_{};
        ICM_20948_Device_t icm_20948_device_{};
    };

}; // namespace ICM20948

#endif // ICM20948_HPP