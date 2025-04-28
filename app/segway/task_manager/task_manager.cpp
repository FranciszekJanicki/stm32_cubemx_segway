#include "task_manager.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto TASK_NUM = std::to_underlying(TaskType::TASK_NUM);

        auto tasks = std::array<TaskHandle_t, TASK_NUM>{};

    }; // namespace

    void set_task(TaskType const type, TaskHandle_t const handle) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < TASK_NUM);
        assert(tasks[index] == nullptr);

        tasks[index] = handle;
    }

    TaskHandle_t get_task(TaskType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < TASK_NUM);
        assert(tasks[index] != nullptr);

        return tasks[index];
    }

}; // namespace segway