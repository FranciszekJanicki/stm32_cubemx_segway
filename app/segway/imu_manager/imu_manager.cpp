#include "imu_manager.hpp"
#include "FreeRTOS.h"
#include "event_bits.hpp"
#include "event_group_manager.hpp"
#include "i2c.h"
#include "log.hpp"
#include "message_buf_manager.hpp"
#include "mpu6050_dmp.hpp"
#include "queue.h"
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
            std::float64_t sampling_time;
            bool is_running;
        } ctx;

        inline bool send_control_event(ControlEvent const& event) noexcept
        {
#ifdef USE_QUEUES
            return xQueueOverwrite(get_queue(QueueType::CONTROL), &event);
#else
            return xMessageBufferSend(get_message_buf(MessageBufType::CONTROL),
                                      &event,
                                      sizeof(event),
                                      pdMS_TO_TICKS(1)) == sizeof(event);
#endif
        }

        inline std::uint32_t wait_imu_event_bits() noexcept
        {
#ifdef USE_EVENT_GROUPS
            return xEventGroupWaitBits(get_event_group(EventGroupType::IMU),
                                       IMU_EVENT_BIT_ALL,
                                       pdTRUE,
                                       pdFALSE,
                                       pdMS_TO_TICKS(1));
#else
            auto event_bits = 0UL;
            xTaskNotifyWait(0x00, IMU_EVENT_BIT_ALL, &event_bits, pdMS_TO_TICKS(1));
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
            set_control_event_bits(CONTROL_EVENT_BIT_START);

            HAL_TIM_Base_Start_IT(&htim2);
        }

        void process_stop() noexcept
        {
            if (ctx.is_running) {
                return;
            }

            LOG(TAG, "process_stop");

            ctx.is_running = false;
            set_control_event_bits(CONTROL_EVENT_BIT_STOP);

            HAL_TIM_Base_Stop_IT(&htim2);
        }

        void process_data_ready() noexcept
        {
            if (!ctx.is_running) {
                return;
            }

            LOG(TAG, "process_data_ready");

            auto rpy = ctx.imu.get_roll_pitch_yaw().value();

            ControlEvent event = {.type = ControlEventType::IMU_DATA};
            event.payload.imu_data.roll = utility::radians_to_degrees(rpy.x);
            event.payload.imu_data.pitch = utility::radians_to_degrees(rpy.y);
            event.payload.imu_data.yaw = utility::radians_to_degrees(rpy.z);
            event.payload.imu_data.dt = ctx.sampling_time;

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

            if ((event_bits & IMU_EVENT_BIT_START) == IMU_EVENT_BIT_START) {
                process_start();
            }

            if ((event_bits & IMU_EVENT_BIT_STOP) == IMU_EVENT_BIT_STOP) {
                process_stop();
            }

            if ((event_bits & IMU_EVENT_BIT_DATA_READY) == IMU_EVENT_BIT_DATA_READY) {
                process_data_ready();
            }

            if ((event_bits & IMU_EVENT_BIT_I2C_ERROR) == IMU_EVENT_BIT_I2C_ERROR) {
                process_i2c_error();
            }

            if ((event_bits & IMU_EVENT_BIT_RX_COMPLETE) == IMU_EVENT_BIT_RX_COMPLETE) {
                process_rx_complete();
            }

            if ((event_bits & IMU_EVENT_BIT_TX_COMPLETE) == IMU_EVENT_BIT_TX_COMPLETE) {
                process_tx_complete();
            }
        }

        void imu_task(void*) noexcept
        {
            process_start();

            while (1) {
                process_imu_event_group_bits();
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        inline void imu_task_init() noexcept
        {
            constexpr auto IMU_TASK_PRIORITY = 2UL;
            constexpr auto IMU_TASK_STACK_DEPTH = 1024UL;
            constexpr auto IMU_TASK_NAME = "imu_task";
            constexpr auto IMU_TASK_ARG = nullptr;

            static StaticTask_t imu_task_buffer = {};
            static std::array<StackType_t, IMU_TASK_STACK_DEPTH> imu_task_stack = {};

            set_task(TaskType::IMU,
                     xTaskCreateStatic(&imu_task,
                                       IMU_TASK_NAME,
                                       imu_task_stack.size(),
                                       IMU_TASK_ARG,
                                       IMU_TASK_PRIORITY,
                                       imu_task_stack.data(),
                                       &imu_task_buffer));
        }

        inline void imu_event_group_init() noexcept
        {
#ifdef USE_EVENT_GROUPS
            static StaticEventGroup_t imu_event_group_buffer = {};

            set_event_group(EventGroupType::IMU, xEventGroupCreateStatic(&imu_event_group_buffer));
#endif
        }

        inline void imu_config_init() noexcept
        {
            constexpr auto SAMPLING_TIME = 0.005F64;

            ctx.sampling_time = SAMPLING_TIME;
            ctx.is_running = false;
        }

        inline void imu_periph_init() noexcept
        {
            mpu6050::Interface mpu6050_interface = {
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

            mpu6050::Config mpu6050_config = {.sampling_rate = 200UL,
                                              .gyro_range = mpu6050::GyroRange::GYRO_FS_250,
                                              .accel_range = mpu6050::AccelRange::ACCEL_FS_2,
                                              .dlpf_setting = mpu6050::DLPF::BW_42,
                                              .dhpf_setting = mpu6050::DHPF::DHPF_RESET};

            mpu6050::MPU6050 mpu6050 = {.interface = mpu6050_interface};

            ctx.imu = mpu6050::MPU6050_DMP{.mpu6050 = mpu6050};
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
