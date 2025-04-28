#include "task_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto TASK_NUM = std::to_underlying(TaskType::TASK_NUM);

        auto task_handles = std::array<TaskHandle_t, TASK_NUM>{};

    }; // namespace

    void set_task_handle(TaskType const type, TaskHandle_t const handle) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < TASK_NUM);
        assert(task_handles[index] == nullptr);

        task_handles[index] = handle;
    }

    TaskHandle_t get_task_handle(TaskType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < TASK_NUM);
        assert(task_handles[index] != nullptr);

        return task_handles[index];
    }

}; // namespace segway