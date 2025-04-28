#include "wheel_event_group.hpp"
#include "FreeRTOS.h"
#include "event_group_manager.hpp"
#include "event_groups.h"

namespace segway {

    namespace {

        inline auto wheel_static_event_group = StaticEventGroup_t{};

    };

    void wheel_event_group_init() noexcept
    {
        auto handle = xEventGroupCreateStatic(&wheel_static_event_group);

        set_event_group(EventGroupType::WHEEL, handle);
    }

}; // namespace segway