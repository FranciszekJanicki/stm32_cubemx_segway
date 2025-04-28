#ifndef MPU6050_DMP_HPP
#define MPU6050_DMP_HPP

#include "mpu6050.hpp"
#include "mpu6050_config.hpp"
#include "mpu6050_registers.hpp"
#include <array>
#include <cstdint>

namespace mpu6050 {

    struct MPU6050_DMP {
    public:
        void initialize(Config const& config) noexcept;
        void deinitialize() noexcept;

        std::optional<std::float64_t> get_roll() const noexcept;
        std::optional<std::float64_t> get_pitch() const noexcept;
        std::optional<std::float64_t> get_yaw() const noexcept;
        std::optional<Vec3D<std::float64_t>> get_roll_pitch_yaw() const noexcept;

        std::optional<Quat3D<std::float64_t>> get_quaternion_scaled() const noexcept;

        bool initialized = false;

        MPU6050 mpu6050 = {};

    private:
        void initialize_dmp() const noexcept;

        std::optional<Quat3D<std::int16_t>> get_quaternion_raw() const noexcept;

        bool get_otp_bank_valid() const noexcept;
        void set_otp_bank_valid(bool const enabled) const noexcept;

        void set_x_gyro_offset_tc(std::uint8_t const offset) const noexcept;
        void set_y_gyro_offset_tc(std::uint8_t const offset) const noexcept;
        void set_z_gyro_offset_tc(std::uint8_t const offset) const noexcept;

        void set_x_fine_gain(std::uint8_t const gain) const noexcept;
        void set_y_fine_gain(std::uint8_t const gain) const noexcept;
        void set_z_fine_gain(std::uint8_t const gain) const noexcept;

        void set_x_accel_offset(std::int16_t const offset) const noexcept;
        void set_y_accel_offset(std::int16_t const offset) const noexcept;
        void set_z_accel_offset(std::int16_t const offset) const noexcept;

        void set_x_gyro_offset(std::int16_t const offset) const noexcept;
        void set_y_gyro_offset(std::int16_t const offset) const noexcept;
        void set_z_gyro_offset(std::int16_t const offset) const noexcept;

        void set_int_pll_ready_enabled(bool const enabled) const noexcept;
        void set_int_dmp_enabled(bool const enabled) const noexcept;

        bool get_dmp_int_5_status() const noexcept;
        bool get_dmp_int_4_status() const noexcept;
        bool get_dmp_int_3_status() const noexcept;
        bool get_dmp_int_2_status() const noexcept;
        bool get_dmp_int_1_status() const noexcept;
        bool get_dmp_int_0_status() const noexcept;

        bool get_int_pll_ready_status() const noexcept;
        bool get_int_dmp_status() const noexcept;

        void set_dmp_enabled(bool const enabled) const noexcept;
        void reset_dmp() const noexcept;

        void set_memory_bank(std::uint8_t const bank,
                             bool const prefetch_enabled = false,
                             bool const user_bank = false) const noexcept;
        void set_memory_start_address(std::uint8_t const address) const noexcept;

        std::uint8_t read_memory_byte() const noexcept;
        void write_memory_byte(std::uint8_t write_data) const noexcept;
        void read_memory_block(std::uint8_t* read_data,
                               std::size_t const read_size,
                               std::uint8_t bank,
                               std::uint8_t address) const noexcept;
        void write_memory_block(std::uint8_t* write_data,
                                std::size_t const write_size,
                                std::uint8_t bank,
                                std::uint8_t address) const noexcept;
        void write_dmp_configuration_set(std::uint8_t* write_data, std::size_t const write_size) const noexcept;

        void set_dmp_config1(std::uint8_t const config) const noexcept;
        void set_dmp_config2(std::uint8_t const config) const noexcept;

        std::array<std::uint8_t, 42UL> get_dmp_packet() const noexcept;
    };

}; // namespace mpu6050

#endif // MPU6050_DMP_HPP