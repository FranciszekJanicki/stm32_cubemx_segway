#include "event_group_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto EVENT_GROUP_NUM = std::to_underlying(EventGroupType::EVENT_GROUP_NUM);

        auto event_group_handles = std::array<EventGroupHandle_t, EVENT_GROUP_NUM>{};

    }; // namespace

    void set_event_group_handle(EventGroupType const type, EventGroupHandle_t const handle) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < EVENT_GROUP_NUM);
        assert(event_group_handles[index] == nullptr);

        event_group_handles[index] = handle;
    }

    EventGroupHandle_t get_event_group_handle(EventGroupType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < EVENT_GROUP_NUM);
        assert(event_group_handles[index] != nullptr);

        return event_group_handles[index];
    }

}; // namespace segway