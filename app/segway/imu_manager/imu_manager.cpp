#include "imu_manager.hpp"
#include "event_group_manager.hpp"
#include "i2c.h"
#include "log.hpp"
#include "mpu6050_dmp.hpp"
#include "queue_manager.hpp"
#include "tim.h"

namespace segway {

    namespace {

        constexpr auto TAG = "imu_manager";

        constexpr auto IMU_FAULT_THRESH_LOW = 160.0F64;
        constexpr auto IMU_FAULT_THRESH_HIGH = 180.0F64;

        struct Context {
            mpu6050::MPU6050_DMP imu;

            struct Config {
                std::float64_t imu_fault_thresh_low;
                std::float64_t imu_fault_thresh_high;
                std::float64_t sampling_time;
            } config;
        } ctx;

        void start_sampling_timer() noexcept
        {
            HAL_TIM_Base_Start_IT(&htim2);
        }

        void stop_sampling_timer() noexcept
        {
            HAL_TIM_Base_Stop_IT(&htim2);
        }

        void process_data_ready() noexcept
        {
            LOG(TAG, "process_sampling_timer");

            auto rpy = ctx.imu.get_roll_pitch_yaw().value();

            auto handle = get_queue(QueueType::CONTROL);
            auto event = ControlEvent{.type = ControlEventType::IMU_DATA};
            event.payload.imu_data = {.roll = rpy.x, .pitch = rpy.y, .yaw = rpy.z, .dt = ctx.config.sampling_time};

            if (!xQueueSend(handle, &event, pdMS_TO_TICKS(10))) {
                LOG(TAG, "Failed sending to queue!");
            }

            start_sampling_timer();
        }

        void process_sampling_timer() noexcept
        {
            process_data_ready();
        }

        void process_i2c_error() noexcept
        {
            LOG(TAG, "process_i2c_error");
        }

        void process_rx_complete() noexcept
        {
            LOG(TAG, "process_rx_complete");
        }

        void process_tx_complete() noexcept
        {
            LOG(TAG, "process_tx_complete");
        }

        void process_event_group_bits() noexcept
        {
            LOG(TAG, "process_event_group_bits");

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
        LOG(TAG, "imu_manager_init");

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
        //   mpu6050_dmp.initialize(mpu6050_config);

        ctx.imu = std::move(mpu6050_dmp);

        start_sampling_timer();
    }

    void imu_manager_process() noexcept
    {
        process_event_group_bits();

        // auto* buf = static_cast<char*>(std::calloc(uxTaskGetNumberOfTasks() * 40UL, 1UL));
        // if (buf) {
        //     vTaskList(buf);
        //     LOG(TAG, buf);
        //     std::free(buf);
        // }

        // auto bytes_left = sizeof(StackType_t) * uxTaskGetStackHighWaterMark(nullptr);
        // LOG(TAG, "Bytes left: %d", bytes_left);
    }

}; // namespace segway
