#ifndef SEGWAY_EVENT_GROUP_MANAGER_HPP
#define SEGWAY_EVENT_GROUP_MANAGER_HPP

#include "FreeRTOS.h"
#include "event_group_bit.hpp"
#include "event_groups.h"
#include <cstdint>

namespace segway {

    enum struct EventGroupType : std::uint8_t {
        IMU,
        WHEEL,
        EVENT_GROUP_NUM,
    };

    void set_event_group(EventGroupType const type, EventGroupHandle_t const handle) noexcept;

    inline void set_imu_event_group(EventGroupHandle_t const handle) noexcept
    {
        set_event_group(EventGroupType::IMU, handle);
    }

    inline void set_wheel_event_group(EventGroupHandle_t const handle) noexcept
    {
        set_event_group(EventGroupType::WHEEL, handle);
    }

    EventGroupHandle_t get_event_group(EventGroupType const type) noexcept;

    inline EventGroupHandle_t get_imu_event_group() noexcept
    {
        return get_event_group(EventGroupType::IMU);
    }

    inline EventGroupHandle_t get_wheel_event_group() noexcept
    {
        return get_event_group(EventGroupType::WHEEL);
    }

}; // namespace segway

#endif // SEGWAY_EVENT_GROUP_MANAGER_HPP