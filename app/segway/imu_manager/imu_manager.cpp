#include "imu_manager.hpp"
#include "event_group_manager.hpp"
#include "imu.hpp"

namespace segway {

    namespace {

        constexpr auto TAG = "imu_manager";

        struct Context {
            IMU imu = {};
        };

        void process_data_ready() noexcept
        {}

        void process_i2c_error() noexcept
        {}

        void process_rx_complete() noexcept
        {}

        void process_tx_complete() noexcept
        {}

        void process_sampling_timer() noexcept
        {}

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
    {}

    void imu_manager_process() noexcept
    {
        process_event_group_bits();
    }

}; // namespace segway
