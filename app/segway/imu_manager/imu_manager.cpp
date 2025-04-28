#include "imu_manager.hpp"
#include "event_group_manager.hpp"
#include "i2c.h"
#include "icm20948_dmp.hpp"
#include "log.hpp"
#include "mpu6050_dmp.hpp"
#include "queue_manager.hpp"
#include <variant>

namespace segway {

    namespace {

        constexpr auto TAG = "imu_manager";

        struct Context {
            std::variant<mpu6050::MPU6050_DMP, icm20948::ICM20948_DMP> imu = {};

            struct Config {
                std::float64_t imu_fault_thresh_low = {};
                std::float64_t imu_fault_thresh_high = {};
            } config;
        } ctx;

        void process_data_ready() noexcept
        {
            auto const& [r, p, y] =
                std::visit([](auto& sensor) { return sensor.get_roll_pitch_yaw().value(); }, ctx.imu);

            LOG(TAG, "r: %f, p: %f, y: %f", r, p, y);

            auto handle = get_queue(QueueType::CONTROL);

            auto event = ControlEvent{};
            event.type = ControlEventType::IMU_DATA;
            event.payload.imu_data.roll = r;
            event.payload.imu_data.pitch = p;
            event.payload.imu_data.yaw = y;

            xQueueSend(handle, &event, pdMS_TO_TICKS(10));
        }

        void process_i2c_error() noexcept
        {
            LOG(TAG, "I2C ERROR!!!");
        }

        void process_rx_complete() noexcept
        {
            LOG(TAG, "RX COMPLETE");
        }

        void process_tx_complete() noexcept
        {
            LOG(TAG, "TX COMPLETE");
        }

        void process_sampling_timer() noexcept
        {
            auto const& [r, p, y] =
                std::visit([](auto& sensor) { return sensor.get_roll_pitch_yaw().value(); }, ctx.imu);

            LOG(TAG, "r: %f, p: %f, y: %f", r, p, y);

            auto handle = get_queue(QueueType::CONTROL);

            auto event = ControlEvent{};
            event.type = ControlEventType::IMU_DATA;
            event.payload.imu_data.roll = r;
            event.payload.imu_data.pitch = p;
            event.payload.imu_data.yaw = y;

            xQueueSend(handle, &event, pdMS_TO_TICKS(10));
        }

        void process_event_group_bits() noexcept
        {
            auto event_group = get_event_group(EventGroupType::IMU);

            auto event_bits = xEventGroupWaitBits(event_group, IMUEventBit::ALL, pdTRUE, pdFALSE, pdMS_TO_TICKS(10));

            if ((event_bits & IMUEventBit::DATA_READY) == IMUEventBit::DATA_READY) {
                process_data_ready();
            }

            if ((event_bits & IMUEventBit::I2C_ERROR) == IMUEventBit::I2C_ERROR) {
                process_i2c_error();
            }

            if ((event_bits & IMUEventBit::RX_COMPLETE) == IMUEventBit::RX_COMPLETE) {
                process_rx_complete();
            }

            if ((event_bits & IMUEventBit::TX_COMPLETE) == IMUEventBit::TX_COMPLETE) {
                process_tx_complete();
            }

            if ((event_bits & IMUEventBit::SAMPLING_TIMER) == IMUEventBit::SAMPLING_TIMER) {
                process_sampling_timer();
            }
        }

    }; // namespace

    void imu_manager_init() noexcept
    {
        auto mpu6050_interface =
            mpu6050::Interface{.user = &hi2c1,
                               .write_bytes =
                                   [](void* user, std::uint8_t address, std::uint8_t* data, std::size_t size) {
                                       auto handle = static_cast<I2C_HandleTypeDef*>(user);
                                       HAL_I2C_Mem_Write(handle, 104, address, I2C_MEMADD_SIZE_8BIT, data, size, 100);
                                   },
                               .read_bytes =
                                   [](void* user, std::uint8_t address, std::uint8_t* data, std::size_t size) {
                                       auto handle = static_cast<I2C_HandleTypeDef*>(user);
                                       HAL_I2C_Mem_Read(handle, 104, address, I2C_MEMADD_SIZE_8BIT, data, size, 100);
                                   },
                               .delay_ms = [](void* user, std::uint32_t ms) { HAL_Delay(ms); }};

        auto mpu6050_config = mpu6050::Config{.sampling_rate = 200UL,
                                              .gyro_range = mpu6050::GyroRange::GYRO_FS_250,
                                              .accel_range = mpu6050::AccelRange::ACCEL_FS_2,
                                              .dlpf_setting = mpu6050::DLPF::BW_42,
                                              .dhpf_setting = mpu6050::DHPF::DHPF_RESET};

        auto mpu6050 = mpu6050::MPU6050{.interface = std::move(mpu6050_interface)};

        auto mpu6050_dmp = mpu6050::MPU6050_DMP{.mpu6050 = std::move(mpu6050)};
        mpu6050_dmp.initialize(mpu6050_config);

        ctx.imu = std::move(mpu6050_dmp);
    }

    void imu_manager_process() noexcept
    {
        process_event_group_bits();
    }

}; // namespace segway
