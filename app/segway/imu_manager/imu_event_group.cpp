#include "imu_event_group.hpp"
#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"

namespace segway {

    namespace {

        inline auto imu_static_event_group = StaticEventGroup_t{};

    };

    void imu_event_group_init() noexcept
    {
        auto handle = xEventGroupCreateStatic(&imu_static_event_group);

        set_event_group_handle(EventGroupType::IMU_EVENT_GROUP, handle);
    }

}; // namespace segway