#include "imu_manager.hpp"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "i2c.h"
#include "log.hpp"
#include "message_buffer_manager.hpp"
#include "mpu6050_dmp.hpp"
#include "queue_manager.hpp"
#include "task_manager.hpp"
#include "tim.h"
#include "utility.hpp"
#include <cassert>

namespace segway {

    namespace {

        constexpr auto TAG = "imu_manager";

        struct IMUManagerCtx {
            mpu6050::MPU6050_DMP imu;

            struct Config {
                std::float64_t fault_thresh_low;
                std::float64_t fault_thresh_high;
                std::float64_t sampling_time;
            } config;

            bool is_running;
        } ctx;

        inline bool send_control_event(ControlEvent const& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueueSend(get_queue(QueueType::CONTROL), &event, pdMS_TO_TICKS(10));
#else
            return xMessageBufferSend(get_message_buffer(MessageBufferType::CONTROL),
                                      &event,
                                      sizeof(event),
                                      pdMS_TO_TICKS(10)) == sizeof(event);
#endif
        }

        inline bool receive_imu_event(IMUEvent& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueueReceive(get_queue(QueueType::IMU), &event, pdMS_TO_TICKS(10));
#else
            return xMessageBufferReceive(get_message_buffer(MessageBufferType::IMU),
                                         &event,
                                         sizeof(event),
                                         pdMS_TO_TICKS(10)) == sizeof(event);
#endif
        }

        inline std::uint32_t wait_imu_event_bits() noexcept
        {
#ifdef USE_EVENT_GROUPS
            return xEventGroupWaitBits(get_event_group(EventGroupType::IMU),
                                       IMUEventBit::ALL,
                                       pdTRUE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(10));
#else
            auto event_bits = 0UL;
            xTaskNotifyWait(0x00, IMUEventBit::ALL, &event_bits, pdMS_TO_TICKS(10));
            return event_bits;
#endif
        }

        inline void set_control_event_bits(std::uint32_t const event_bits) noexcept
        {
#ifdef USE_EVENT_GROUPS
            xEventGroupSetBits(get_event_group(EventGroupType::CONTROL), event_bits);
#else
            xTaskNotify(get_task(TaskType::CONTROL), event_bits, eNotifyAction::eSetBits);
#endif
        }

        void process_start() noexcept
        {
            if (ctx.is_running) {
                return;
            }

            LOG(TAG, "process_start");

            ctx.is_running = true;
            set_control_event_bits(ControlEventBit::START);

            HAL_TIM_Base_Start_IT(&htim2);
        }

        void process_stop() noexcept
        {
            if (ctx.is_running) {
                return;
            }

            LOG(TAG, "process_stop");

            ctx.is_running = false;
            set_control_event_bits(ControlEventBit::STOP);

            HAL_TIM_Base_Stop_IT(&htim2);
        }

        void process_data_ready() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_data_ready");

            auto rpy = ctx.imu.get_roll_pitch_yaw().value();

            auto event = ControlEvent{.type = ControlEventType::IMU_DATA};
            event.payload.imu_data.roll = utility::radians_to_degrees(rpy.x);
            event.payload.imu_data.pitch = utility::radians_to_degrees(rpy.y);
            event.payload.imu_data.yaw = utility::radians_to_degrees(rpy.z);
            event.payload.imu_data.dt = ctx.config.sampling_time;

            if (!send_control_event(event)) {
                LOG(TAG, "Failed send_control_event");
            }

            HAL_TIM_Base_Start_IT(&htim2);
        }

        void process_i2c_error() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_i2c_error");
        }

        void process_rx_complete() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_rx_complete");
        }

        void process_tx_complete() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_tx_complete");
        }

        void process_imu_event_group_bits() noexcept
        {
            LOG(TAG, "process_imu_event_group_bits");

            auto event_bits = wait_imu_event_bits();

            if ((event_bits & IMUEventBit::START) == IMUEventBit::START) {
                process_start();
            }

            if ((event_bits & IMUEventBit::STOP) == IMUEventBit::STOP) {
                process_stop();
            }

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
            constexpr auto IMU_TASK_PRIORITY = 2UL;
            constexpr auto IMU_TASK_STACK_DEPTH = 1024UL;
            constexpr auto IMU_TASK_NAME = "imu_task";
            constexpr auto IMU_TASK_ARG = nullptr;

            static auto imu_static_task = StaticTask_t{};
            static auto imu_task_stack = std::array<StackType_t, IMU_TASK_STACK_DEPTH>{};

            set_task(TaskType::IMU,
                     xTaskCreateStatic(&imu_task,
                                       IMU_TASK_NAME,
                                       imu_task_stack.size(),
                                       IMU_TASK_ARG,
                                       IMU_TASK_PRIORITY,
                                       imu_task_stack.data(),
                                       &imu_static_task));
        }

        inline void imu_event_group_init() noexcept
        {
#ifdef USE_EVENT_GROUPS
            static auto imu_static_event_group = StaticEventGroup_t{};

            set_event_group(EventGroupType::IMU, xEventGroupCreateStatic(&imu_static_event_group));
#endif
        }

        inline void imu_config_init() noexcept
        {
            constexpr auto FAULT_THRESH_LOW = 160.0F64;
            constexpr auto FAULT_THRESH_HIGH = 180.0F64;
            constexpr auto SAMPLING_TIME = 0.01F64;

            ctx.config.fault_thresh_high = FAULT_THRESH_HIGH;
            ctx.config.fault_thresh_low = FAULT_THRESH_LOW;
            ctx.config.sampling_time = SAMPLING_TIME;

            ctx.is_running = false;
        }

        inline void imu_periph_init() noexcept
        {
            auto mpu6050_interface = mpu6050::Interface{
                .user = &hi2c1,
                .write_bytes =
                    [](void* user, std::uint8_t address, std::uint8_t* data, std::size_t size) {
                        auto handle = static_cast<I2C_HandleTypeDef*>(user);
                        assert(HAL_OK ==
                               HAL_I2C_Mem_Write(handle, 0x68 << 1, address, 1, data, size, 100));
                    },
                .read_bytes =
                    [](void* user, std::uint8_t address, std::uint8_t* data, std::size_t size) {
                        auto handle = static_cast<I2C_HandleTypeDef*>(user);
                        assert(HAL_OK ==
                               HAL_I2C_Mem_Read(handle, 0x68 << 1, address, 1, data, size, 100));
                    },
                .delay_ms = [](void* user, std::uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }};

            auto mpu6050_config = mpu6050::Config{.sampling_rate = 200UL,
                                                  .gyro_range = mpu6050::GyroRange::GYRO_FS_250,
                                                  .accel_range = mpu6050::AccelRange::ACCEL_FS_2,
                                                  .dlpf_setting = mpu6050::DLPF::BW_42,
                                                  .dhpf_setting = mpu6050::DHPF::DHPF_RESET};

            auto mpu6050 = mpu6050::MPU6050{.interface = std::move(mpu6050_interface)};

            ctx.imu = mpu6050::MPU6050_DMP{.mpu6050 = std::move(mpu6050)};
            ctx.imu.initialize(mpu6050_config);
        }

    }; // namespace

    void imu_manager_init() noexcept
    {
        std::memset(&ctx, 0, sizeof(ctx));

        imu_config_init();
        imu_periph_init();
        imu_event_group_init();
        imu_task_init();
    }

}; // namespace segway
