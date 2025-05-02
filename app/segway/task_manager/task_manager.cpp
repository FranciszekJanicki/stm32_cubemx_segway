#include "task_manager.hpp"
#include "log.hpp"
#include <array>
#include <cassert>
#include <utility>

namespace segway {

    namespace {

        constexpr auto TASK_NUM = std::to_underlying(TaskType::TASK_NUM);

        auto tasks = std::array<TaskHandle_t, TASK_NUM>{};

    }; // namespace

    extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
    {
        LOG("RTOS", "Stack overflow in task %s", pcTaskName);
        while (1)
            ; // Stop here for debugging
    }

    void set_task(TaskType const type, TaskHandle_t const handle) noexcept
    {
        assert(handle);

        auto const index = std::to_underlying(type);
        assert(index < TASK_NUM);
        assert(!tasks[index]);

        tasks[index] = handle;
    }

    TaskHandle_t get_task(TaskType const type) noexcept
    {
        auto const index = std::to_underlying(type);

        assert(index < TASK_NUM);
        assert(tasks[index]);

        return tasks[index];
    }

}; // namespace segway