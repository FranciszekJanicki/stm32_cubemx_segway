#include "event_group_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr std::size_t EVENT_GROUP_NUM = std::to_underlying(EventGroupType::EVENT_GROUP_NUM);

        std::array<EventGroupHandle_t, EVENT_GROUP_NUM> event_groups = {};

    }; // namespace

    void set_event_group(EventGroupType const type, EventGroupHandle_t const handle) noexcept
    {
        assert(handle);

        auto index = std::to_underlying(type);
        assert(index < EVENT_GROUP_NUM);
        assert(!event_groups[index]);

        event_groups[index] = handle;
    }

    EventGroupHandle_t get_event_group(EventGroupType const type) noexcept
    {
        auto index = std::to_underlying(type);
        assert(index < EVENT_GROUP_NUM);
        assert(event_groups[index]);

        return event_groups[index];
    }

}; // namespace segway