#ifndef SEGWAY_EVENT_GROUP_MANAGER_HPP
#define SEGWAY_EVENT_GROUP_MANAGER_HPP

#include "FreeRTOS.h"
#include "event_group_bit.hpp"
#include "event_groups.h"
#include <cstdint>

namespace segway {

    enum struct EventGroupType : std::uint8_t {
        IMU_EVENT_GROUP,
        WHEEL_EVENT_GROUP,
        EVENT_GROUP_NUM,
    };

    void set_event_group_handle(EventGroupType const type, EventGroupHandle_t const handle) noexcept;

    EventGroupHandle_t get_event_group_handle(EventGroupType const type) noexcept;

}; // namespace segway

#endif // SEGWAY_EVENT_GROUP_MANAGER_HPP