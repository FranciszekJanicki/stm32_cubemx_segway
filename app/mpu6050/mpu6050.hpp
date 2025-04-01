#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "mpu6050_config.hpp"

namespace MPU6050 {

    struct MPU6050 {
        MPU6050() noexcept = default;
        MPU6050(I2CDevice&& i2c_device,
                std::uint32_t const sampling_rate,
                GyroRange const gyro_range,
                AccelRange const accel_range,
                DLPF const dlpf,
                DHPF const dhpf) noexcept;

        MPU6050(MPU6050 const& other) noexcept = delete;
        MPU6050(MPU6050&& other) noexcept = default;

        MPU6050& operator=(MPU6050 const& other) noexcept = delete;
        MPU6050& operator=(MPU6050&& other) noexcept = default;

        ~MPU6050() noexcept;

        /* celsius */
        [[nodiscard]] float get_temperature_celsius() const noexcept;

        /* meters per square second */
        [[nodiscard]] Vec3D<float> get_acceleration_scaled() const noexcept;
        [[nodiscard]] float get_acceleration_x_scaled() const noexcept;
        [[nodiscard]] float get_acceleration_y_scaled() const noexcept;
        [[nodiscard]] float get_acceleration_z_scaled() const noexcept;

        /* radians */
        [[nodiscard]] Vec3D<float> get_rotation_scaled() const noexcept;
        [[nodiscard]] float get_rotation_x_scaled() const noexcept;
        [[nodiscard]] float get_rotation_y_scaled() const noexcept;
        [[nodiscard]] float get_rotation_z_scaled() const noexcept;

        /* degrees */
        [[nodiscard]] Vec3D<float> get_roll_pitch_yaw() const noexcept;
        [[nodiscard]] float get_roll() const noexcept;
        [[nodiscard]] float get_pitch() const noexcept;
        [[nodiscard]] float get_yaw() const noexcept;

        std::uint8_t read_byte(std::uint8_t const reg_address) const noexcept;

        void write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

        bool read_bit(std::uint8_t const reg_address, std::uint8_t const position) const noexcept;

        std::uint8_t
        read_bits(std::uint8_t const reg_address, std::uint8_t const position, std::uint8_t const size) const noexcept;

        void write_bit(std::uint8_t const reg_address, bool const bit, std::uint8_t const position) const noexcept;

        void write_bits(std::uint8_t const reg_address,
                        std::uint8_t const bit,
                        std::uint8_t const position,
                        std::uint8_t const size) const noexcept;

        void
        read_bytes(std::uint8_t const reg_address, std::uint8_t* const bytes, std::uint8_t const size) const noexcept;

        void
        write_bytes(std::uint8_t const reg_address, std::uint8_t* const bytes, std::uint8_t const size) const noexcept;

        bool is_valid_device_id() const noexcept;

        void initialize(std::uint32_t const sampling_rate,
                        GyroRange const gyro_range,
                        AccelRange const accel_range,
                        DLPF const dlpf,
                        DHPF const dhpf) noexcept;
        void initialize_base(GyroRange const gyro_range, AccelRange const accel_range) const noexcept;
        void initialize_advanced(std::uint32_t const sampling_rate, DLPF const dlpf, DHPF const dhpf) const noexcept;
        void initialize_interrupt() const noexcept;
        void initialize_data_ready_interrupt() const noexcept;
        void initialize_f_sync_interrupt() const noexcept;
        void initialize_motion_interrupt() const noexcept;
        void initialize_zero_motion_interrupt() const noexcept;
        void initialize_free_fall_interrupt() const noexcept;
        void deinitialize() noexcept;

        void set_sampling_rate(std::uint8_t const sampling_rate, DLPF const dlpf) const noexcept;
        void set_external_frame_sync(ExtSync const frame_sync) const noexcept;
        void set_dlpf_mode(DLPF const dlpf) const noexcept;
        void set_full_scale_gyro_range(GyroRange const range) const noexcept;
        void set_full_scale_accel_range(AccelRange const range) const noexcept;
        void set_dhpf_mode(DHPF const dhpf) const noexcept;

        void set_free_fall_detection_threshold(std::uint8_t const threshold) const noexcept;
        void set_free_fall_detection_duration(std::uint8_t const duration) const noexcept;
        void set_motion_detection_threshold(std::uint8_t const threshold) const noexcept;
        void set_motion_detection_duration(std::uint8_t const duration) const noexcept;
        void set_zero_motion_detection_threshold(std::uint8_t const threshold) const noexcept;
        void set_zero_motion_detection_duration(std::uint8_t const duration) const noexcept;

        void set_fifo_enabled(std::uint8_t const fifo_enabled) const noexcept;
        void set_temp_fifo_enabled(bool const enabled) const noexcept;
        void set_x_gyro_fifo_enabled(bool const enabled) const noexcept;
        void set_y_gyro_fifo_enabled(bool const enabled) const noexcept;
        void set_z_gyro_fifo_enabled(bool const enabled) const noexcept;
        void set_accel_fifo_enabled(bool const enabled) const noexcept;
        void set_slave2_fifo_enabled(bool const enabled) const noexcept;
        void set_slave1_fifo_enabled(bool const enabled) const noexcept;
        void set_slave0_fifo_enabled(bool const enabled) const noexcept;

        void set_multi_master_enabled(bool const enabled) const noexcept;
        void set_wait_for_external_sensor_enabled(bool const enabled) const noexcept;
        void set_slave3_fifo_enabled(bool const enabled) const noexcept;
        void set_slave_read_write_transition_enabled(bool const enabled) const noexcept;
        void set_master_clock_speed(std::uint8_t const speed) const noexcept;

        void set_slave_address(std::uint8_t const num, std::uint8_t const address) const noexcept;
        void set_slave_register(std::uint8_t const num, std::uint8_t const reg) const noexcept;
        void set_slave_enabled(std::uint8_t const num, bool const enabled) const noexcept;
        void set_slave_word_byte_swap(std::uint8_t const num, bool const enabled) const noexcept;
        void set_slave_write_mode(std::uint8_t const num, bool const mode) const noexcept;
        void set_slave_word_group_offset(std::uint8_t const num, bool const enabled) const noexcept;
        void set_slave_data_length(std::uint8_t const num, std::uint8_t const length) const noexcept;

        void set_slave4_address(std::uint8_t const address) const noexcept;
        void set_slave4_register(std::uint8_t const reg) const noexcept;
        void set_slave4_output_byte(std::uint8_t const data) const noexcept;
        void set_slave4_enabled(bool const enabled) const noexcept;
        void set_slave4_interrupt_enabled(bool const enabled) const noexcept;
        void set_slave4_write_mode(bool const mode) const noexcept;
        void set_slave4_master_delay(std::uint8_t const delay) const noexcept;
        std::uint8_t get_slave4_input_byte() const noexcept;

        bool get_passthrough_status() const noexcept;
        bool get_slave4_is_done() const noexcept;
        bool get_lost_arbitration() const noexcept;
        bool get_slave4_nack() const noexcept;
        bool get_slave3_nack() const noexcept;
        bool get_slave2_nack() const noexcept;
        bool get_slave1_nack() const noexcept;
        bool get_slave0_nack() const noexcept;

        void set_interrupt(std::uint8_t const interrupt) const noexcept;
        void set_interrupt_mode(IntMode const mode) const noexcept;
        void set_interrupt_drive(IntDrive const drive) const noexcept;
        void set_interrupt_latch(IntLatch const latch) const noexcept;
        void set_interrupt_latch_clear(IntClear const clear) const noexcept;
        void set_f_sync_interrupt_mode(IntMode const mode) const noexcept;
        void set_f_sync_interrupt_enabled(bool const enabled) const noexcept;
        void set_i2c_bypass_enabled(bool const enabled) const noexcept;
        void set_clock_output_enabled(bool const enabled) const noexcept;

        void set_int_enabled(std::uint8_t const int_enabled) const noexcept;
        void set_int_free_fall_enabled(bool const enabled) const noexcept;
        void set_int_motion_enabled(bool const enabled) const noexcept;
        void set_int_zero_motion_enabled(bool const enabled) const noexcept;
        void set_int_fifo_overflow_enabled(bool const enabled) const noexcept;
        void set_int_i2c_master_enabled(bool const enabled) const noexcept;
        void set_int_data_ready_enabled(bool const enabled) const noexcept;

        std::uint8_t get_int_status() const noexcept;
        bool get_int_free_fall_status() const noexcept;
        bool get_int_motion_status() const noexcept;
        bool get_int_zero_motion_status() const noexcept;
        bool get_int_fifo_overflow_status() const noexcept;
        bool get_int_i2c_master_status() const noexcept;
        bool get_int_data_ready_status() const noexcept;

        Vec3D<std::int16_t> get_acceleration_raw() const noexcept;
        std::int16_t get_acceleration_x_raw() const noexcept;
        std::int16_t get_acceleration_y_raw() const noexcept;
        std::int16_t get_acceleration_z_raw() const noexcept;

        std::int16_t get_temperature_raw() const noexcept;

        Vec3D<std::int16_t> get_rotation_raw() const noexcept;
        std::int16_t get_rotation_x_raw() const noexcept;
        std::int16_t get_rotation_y_raw() const noexcept;
        std::int16_t get_rotation_z_raw() const noexcept;

        std::uint8_t get_external_sensor_byte(std::uint8_t const position) const noexcept;
        std::uint16_t get_external_sensor_word(std::uint8_t const position) const noexcept;
        std::uint32_t get_external_sensor_dword(std::uint8_t const position) const noexcept;

        std::uint8_t get_motion_status() const noexcept;
        bool get_x_neg_motion_detected() const noexcept;
        bool get_x_pos_motion_detected() const noexcept;
        bool get_y_neg_motion_detected() const noexcept;
        bool get_y_pos_motion_detected() const noexcept;
        bool get_z_neg_motion_detected() const noexcept;
        bool get_z_pos_motion_detected() const noexcept;
        bool get_zero_motion_detected() const noexcept;

        void set_slave_output_byte(std::uint8_t const num, std::uint8_t const data) const noexcept;
        void set_external_shadow_delay_enabled(bool const enabled) const noexcept;
        void set_slave_delay_enabled(std::uint8_t const num, bool const enabled) const noexcept;

        void reset_gyro_path() const noexcept;
        void reset_accel_path() const noexcept;
        void reset_temperature_path() const noexcept;

        void set_motion_detection_control(std::uint8_t const control) const noexcept;
        void set_accel_power_on_delay(Delay const delay) const noexcept;
        void set_free_fall_detection_counter_decrement(DetectDecrement const decrement) const noexcept;
        void set_motion_detection_counter_decrement(DetectDecrement const decrement) const noexcept;

        void set_fifo_enabled(bool const enabled) const noexcept;
        void set_i2c_master_mode_enabled(bool const enabled) const noexcept;
        void reset_fifo() const noexcept;
        void reset_i2c_master() const noexcept;
        void reset_sensors() const noexcept;

        void device_reset() const noexcept;
        void device_wake_up() const noexcept;
        void set_clock_source(Clock const source) const noexcept;
        void set_sleep_enabled(bool const enabled) const noexcept;
        void set_wake_cycle_enabled(bool const enabled) const noexcept;
        void set_temperature_sensor_enabled(bool const enabled) const noexcept;

        void set_wake_up_frequency(WakeFreq const frequency) const noexcept;
        void set_x_accel_standby(bool const standby) const noexcept;
        void set_y_accel_standby(bool const standby) const noexcept;
        void set_z_accel_standby(bool const standby) const noexcept;
        void set_x_gyro_standby(bool const standby) const noexcept;
        void set_y_gyro_standby(bool const standby) const noexcept;
        void set_z_gyro_standby(bool const standby) const noexcept;

        std::uint16_t get_fifo_count() const noexcept;
        std::uint8_t get_fifo_byte() const noexcept;
        void get_current_fifo_packet(std::uint8_t* packet_data, std::size_t const packet_size) const noexcept;
        void get_fifo_bytes(std::uint8_t* read_data, std::size_t const read_size) const noexcept;
        void set_fifo_byte(std::uint8_t const write_data) const noexcept;
        void set_fifo_bytes(std::uint8_t* write_data, std::size_t const write_size) const noexcept;

        std::uint8_t get_device_id() const noexcept;

        bool initialized_{false};

        I2CDevice i2c_device_{};

        float gyro_scale_{};
        float accel_scale_{};
    };

}; // namespace MPU6050

#endif // MPU6050_HPP