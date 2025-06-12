#include "task_manager.hpp"
#include "log.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr std::size_t TASK_NUM = std::to_underlying(TaskType::TASK_NUM);

        std::array<TaskHandle_t, TASK_NUM> tasks = {};

    }; // namespace

    void set_task(TaskType const type, TaskHandle_t const handle) noexcept
    {
        assert(handle);

        auto index = std::to_underlying(type);
        assert(index < TASK_NUM);
        assert(!tasks[index]);

        tasks[index] = handle;
    }

    TaskHandle_t get_task(TaskType const type) noexcept
    {
        auto index = std::to_underlying(type);
        assert(index < TASK_NUM);
        assert(tasks[index]);

        return tasks[index];
    }

}; // namespace segway