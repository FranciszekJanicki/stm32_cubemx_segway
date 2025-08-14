#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "mpu6050_config.hpp"
#include "mpu6050_registers.hpp"
#include <optional>

namespace mpu6050 {

    struct MPU6050 {
    public:
        void initialize(Config const& config) noexcept;
        void deinitialize() noexcept;

        /* celsius */
        std::optional<std::float64_t> get_temperature_celsius() const noexcept;

        /* meters per square second */
        std::optional<Vec3D<std::float64_t>> get_acceleration_scaled() const noexcept;
        std::optional<std::float64_t> get_acceleration_x_scaled() const noexcept;
        std::optional<std::float64_t> get_acceleration_y_scaled() const noexcept;
        std::optional<std::float64_t> get_acceleration_z_scaled() const noexcept;

        /* radians */
        std::optional<Vec3D<std::float64_t>> get_rotation_scaled() const noexcept;
        std::optional<std::float64_t> get_rotation_x_scaled() const noexcept;
        std::optional<std::float64_t> get_rotation_y_scaled() const noexcept;
        std::optional<std::float64_t> get_rotation_z_scaled() const noexcept;

        /* degrees */
        std::optional<Vec3D<std::float64_t>> get_roll_pitch_yaw() const noexcept;
        std::optional<std::float64_t> get_roll() const noexcept;
        std::optional<std::float64_t> get_pitch() const noexcept;
        std::optional<std::float64_t> get_yaw() const noexcept;

        std::uint8_t read_byte(std::uint8_t reg_address) const noexcept;

        void write_byte(std::uint8_t reg_address, std::uint8_t byte) const noexcept;

        bool read_bit(std::uint8_t reg_address, std::uint8_t position) const noexcept;

        std::uint8_t read_bits(std::uint8_t reg_address,
                               std::uint8_t position,
                               std::uint8_t size) const noexcept;

        void write_bit(std::uint8_t reg_address, bool bit, std::uint8_t position) const noexcept;

        void write_bits(std::uint8_t reg_address,
                        std::uint8_t bit,
                        std::uint8_t position,
                        std::uint8_t size) const noexcept;

        void read_bytes(std::uint8_t reg_address,
                        std::uint8_t* bytes,
                        std::uint8_t size) const noexcept;

        void write_bytes(std::uint8_t reg_address,
                         std::uint8_t* bytes,
                         std::uint8_t size) const noexcept;

        void delay_ms(std::uint32_t ms) const noexcept;

        void set_sampling_rate(std::uint8_t sampling_rate, DLPF dlpf) const noexcept;
        void set_external_frame_sync(ExtSync frame_sync) const noexcept;
        void set_dlpf_mode(DLPF dlpf) const noexcept;
        void set_full_scale_gyro_range(GyroRange range) const noexcept;
        void set_full_scale_accel_range(AccelRange range) const noexcept;
        void set_dhpf_mode(DHPF dhpf) const noexcept;

        void set_free_fall_detection_threshold(std::uint8_t threshold) const noexcept;
        void set_free_fall_detection_duration(std::uint8_t duration) const noexcept;
        void set_motion_detection_threshold(std::uint8_t threshold) const noexcept;
        void set_motion_detection_duration(std::uint8_t duration) const noexcept;
        void set_zero_motion_detection_threshold(std::uint8_t threshold) const noexcept;
        void set_zero_motion_detection_duration(std::uint8_t duration) const noexcept;

        void set_fifo_enabled(std::uint8_t fifo_enabled) const noexcept;
        void set_temp_fifo_enabled(bool enabled) const noexcept;
        void set_x_gyro_fifo_enabled(bool enabled) const noexcept;
        void set_y_gyro_fifo_enabled(bool enabled) const noexcept;
        void set_z_gyro_fifo_enabled(bool enabled) const noexcept;
        void set_accel_fifo_enabled(bool enabled) const noexcept;
        void set_slave2_fifo_enabled(bool enabled) const noexcept;
        void set_slave1_fifo_enabled(bool enabled) const noexcept;
        void set_slave0_fifo_enabled(bool enabled) const noexcept;

        void set_multi_master_enabled(bool enabled) const noexcept;
        void set_wait_for_external_sensor_enabled(bool enabled) const noexcept;
        void set_slave3_fifo_enabled(bool enabled) const noexcept;
        void set_slave_read_write_transition_enabled(bool enabled) const noexcept;
        void set_master_clock_speed(std::uint8_t speed) const noexcept;

        void set_slave_address(std::uint8_t num, std::uint8_t address) const noexcept;
        void set_slave_register(std::uint8_t num, std::uint8_t reg) const noexcept;
        void set_slave_enabled(std::uint8_t num, bool enabled) const noexcept;
        void set_slave_word_byte_swap(std::uint8_t num, bool enabled) const noexcept;
        void set_slave_write_mode(std::uint8_t num, bool mode) const noexcept;
        void set_slave_word_group_offset(std::uint8_t num, bool enabled) const noexcept;
        void set_slave_data_length(std::uint8_t num, std::uint8_t length) const noexcept;

        void set_slave4_address(std::uint8_t address) const noexcept;
        void set_slave4_register(std::uint8_t reg) const noexcept;
        void set_slave4_output_byte(std::uint8_t data) const noexcept;
        void set_slave4_enabled(bool enabled) const noexcept;
        void set_slave4_interrupt_enabled(bool enabled) const noexcept;
        void set_slave4_write_mode(bool mode) const noexcept;
        void set_slave4_master_delay(std::uint8_t delay) const noexcept;
        std::uint8_t get_slave4_input_byte() const noexcept;

        bool get_passthrough_status() const noexcept;
        bool get_slave4_is_done() const noexcept;
        bool get_lost_arbitration() const noexcept;
        bool get_slave4_nack() const noexcept;
        bool get_slave3_nack() const noexcept;
        bool get_slave2_nack() const noexcept;
        bool get_slave1_nack() const noexcept;
        bool get_slave0_nack() const noexcept;

        void set_interrupt(std::uint8_t interrupt) const noexcept;
        void set_interrupt_mode(IntMode mode) const noexcept;
        void set_interrupt_drive(IntDrive drive) const noexcept;
        void set_interrupt_latch(IntLatch latch) const noexcept;
        void set_interrupt_latch_clear(IntClear clear) const noexcept;
        void set_f_sync_interrupt_mode(IntMode mode) const noexcept;
        void set_f_sync_interrupt_enabled(bool enabled) const noexcept;
        void set_i2c_bypass_enabled(bool enabled) const noexcept;
        void set_clock_output_enabled(bool enabled) const noexcept;

        void set_int_enabled(std::uint8_t int_enabled) const noexcept;
        void set_int_free_fall_enabled(bool enabled) const noexcept;
        void set_int_motion_enabled(bool enabled) const noexcept;
        void set_int_zero_motion_enabled(bool enabled) const noexcept;
        void set_int_fifo_overflow_enabled(bool enabled) const noexcept;
        void set_int_i2c_master_enabled(bool enabled) const noexcept;
        void set_int_data_ready_enabled(bool enabled) const noexcept;

        std::uint8_t get_int_status() const noexcept;
        bool get_int_free_fall_status() const noexcept;
        bool get_int_motion_status() const noexcept;
        bool get_int_zero_motion_status() const noexcept;
        bool get_int_fifo_overflow_status() const noexcept;
        bool get_int_i2c_master_status() const noexcept;
        bool get_int_data_ready_status() const noexcept;

        std::optional<Vec3D<std::int16_t>> get_acceleration_raw() const noexcept;
        std::optional<std::int16_t> get_acceleration_x_raw() const noexcept;
        std::optional<std::int16_t> get_acceleration_y_raw() const noexcept;
        std::optional<std::int16_t> get_acceleration_z_raw() const noexcept;

        std::optional<std::int16_t> get_temperature_raw() const noexcept;

        std::optional<Vec3D<std::int16_t>> get_rotation_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_x_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_y_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_z_raw() const noexcept;

        std::uint8_t get_external_sensor_byte(std::uint8_t position) const noexcept;
        std::uint16_t get_external_sensor_word(std::uint8_t position) const noexcept;
        std::uint32_t get_external_sensor_dword(std::uint8_t position) const noexcept;

        std::uint8_t get_motion_status() const noexcept;
        bool get_x_neg_motion_detected() const noexcept;
        bool get_x_pos_motion_detected() const noexcept;
        bool get_y_neg_motion_detected() const noexcept;
        bool get_y_pos_motion_detected() const noexcept;
        bool get_z_neg_motion_detected() const noexcept;
        bool get_z_pos_motion_detected() const noexcept;
        bool get_zero_motion_detected() const noexcept;

        void set_slave_output_byte(std::uint8_t num, std::uint8_t data) const noexcept;
        void set_external_shadow_delay_enabled(bool enabled) const noexcept;
        void set_slave_delay_enabled(std::uint8_t num, bool enabled) const noexcept;

        void reset_gyro_path() const noexcept;
        void reset_accel_path() const noexcept;
        void reset_temperature_path() const noexcept;

        void set_motion_detection_control(std::uint8_t control) const noexcept;
        void set_accel_power_on_delay(Delay delay) const noexcept;
        void set_free_fall_detection_counter_decrement(DetectDecrement decrement) const noexcept;
        void set_motion_detection_counter_decrement(DetectDecrement decrement) const noexcept;

        void set_fifo_enabled(bool enabled) const noexcept;
        void set_i2c_master_mode_enabled(bool enabled) const noexcept;
        void reset_fifo() const noexcept;
        void reset_i2c_master() const noexcept;
        void reset_sensors() const noexcept;

        void device_reset() const noexcept;
        void device_wake_up() const noexcept;
        void set_clock_source(Clock source) const noexcept;
        void set_sleep_enabled(bool enabled) const noexcept;
        void set_wake_cycle_enabled(bool enabled) const noexcept;
        void set_temperature_sensor_enabled(bool enabled) const noexcept;

        void set_wake_up_frequency(WakeFreq frequency) const noexcept;
        void set_x_accel_standby(bool standby) const noexcept;
        void set_y_accel_standby(bool standby) const noexcept;
        void set_z_accel_standby(bool standby) const noexcept;
        void set_x_gyro_standby(bool standby) const noexcept;
        void set_y_gyro_standby(bool standby) const noexcept;
        void set_z_gyro_standby(bool standby) const noexcept;

        std::uint16_t get_fifo_count() const noexcept;
        std::uint8_t get_fifo_byte() const noexcept;
        void get_current_fifo_packet(std::uint8_t* packet_data,
                                     std::size_t packet_size) const noexcept;
        void get_fifo_bytes(std::uint8_t* read_data, std::size_t read_size) const noexcept;
        void set_fifo_byte(std::uint8_t write_data) const noexcept;
        void set_fifo_bytes(std::uint8_t* write_data, std::size_t write_size) const noexcept;

        std::uint8_t get_device_id() const noexcept;

        bool initialized = {};

        Interface interface = {};
        Scale scale = {};
    };

}; // namespace mpu6050

#endif // MPU6050_HPP