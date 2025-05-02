#include "imu_manager.hpp"
#include "event_group_manager.hpp"
#include "i2c.h"
#include "log.hpp"
#include "mpu6050_dmp.hpp"
#include "queue_manager.hpp"
#include "task_manager.hpp"
#include "tim.h"
#include <cassert>

namespace segway {

    namespace {

        constexpr auto TAG = "imu_manager";

        constexpr auto FAULT_THRESH_LOW = 160.0F64;
        constexpr auto FAULT_THRESH_HIGH = 180.0F64;

        struct Context {
            mpu6050::MPU6050_DMP imu;

            struct Config {
                std::float64_t fault_thresh_low;
                std::float64_t fault_thresh_high;
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

            auto event = ControlEvent{.type = ControlEventType::IMU_DATA};
            event.payload.imu_data = {.roll = rpy.x,
                                      .pitch = rpy.y,
                                      .yaw = rpy.z,
                                      .dt = ctx.config.sampling_time};

            if (!xQueueSend(get_control_queue(), &event, pdMS_TO_TICKS(10))) {
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

        void process_imu_event_group_bits() noexcept
        {
            LOG(TAG, "process_imu_event_group_bits");

            auto event_bits = xEventGroupWaitBits(get_imu_event_group(),
                                                  IMUEventBit::ALL,
                                                  pdTRUE,
                                                  pdFALSE,
                                                  pdMS_TO_TICKS(10));

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

        void imu_task(void*) noexcept
        {
            LOG(TAG, "imu_task start");

            while (1) {
                process_imu_event_group_bits();
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            LOG(TAG, "imu_task end");
        }

        inline void imu_task_init() noexcept
        {
            constexpr auto IMU_TASK_PRIORITY = 1UL;
            constexpr auto IMU_TASK_STACK_DEPTH = 1024UL;
            constexpr auto IMU_TASK_NAME = "imu_task";
            constexpr auto IMU_TASK_ARG = nullptr;

            static auto imu_static_task = StaticTask_t{};
            static auto imu_task_stack = std::array<StackType_t, IMU_TASK_STACK_DEPTH>{};

            set_imu_task(xTaskCreateStatic(&imu_task,
                                           IMU_TASK_NAME,
                                           imu_task_stack.size(),
                                           IMU_TASK_ARG,
                                           IMU_TASK_PRIORITY,
                                           imu_task_stack.data(),
                                           &imu_static_task));
        }

        inline void imu_event_group_init() noexcept
        {
            static auto imu_static_event_group = StaticEventGroup_t{};

            set_imu_event_group(xEventGroupCreateStatic(&imu_static_event_group));
        }

    }; // namespace

    void imu_manager_init() noexcept
    {
        LOG(TAG, "manager_init");

        auto mpu6050_interface = mpu6050::Interface{
            .user = &hi2c1,
            .write_bytes =
                [](void* user, std::uint8_t address, std::uint8_t* data, std::size_t size) {
                    auto handle = static_cast<I2C_HandleTypeDef*>(user);
                    assert(HAL_OK == HAL_I2C_Mem_Write(handle, 104, address, 1, data, size, 100));
                },
            .read_bytes =
                [](void* user, std::uint8_t address, std::uint8_t* data, std::size_t size) {
                    auto handle = static_cast<I2C_HandleTypeDef*>(user);
                    assert(HAL_OK == HAL_I2C_Mem_Read(handle, 104, address, 1, data, size, 100));
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

        imu_event_group_init();
        imu_task_init();
    }

}; // namespace segway
