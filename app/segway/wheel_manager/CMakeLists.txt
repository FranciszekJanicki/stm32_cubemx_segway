add_library(wheel_manager STATIC)

target_sources(wheel_manager PRIVATE 
    "wheel_task.cpp"
    "wheel_manager.cpp"
    "wheel_queue.cpp"
    "wheel_event_group.cpp"
    "wheel.cpp"
)

target_include_directories(wheel_manager PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/segway/common
    ${CMAKE_SOURCE_DIR}/app/segway/task_manager
    ${CMAKE_SOURCE_DIR}/app/segway/queue_manager
    ${CMAKE_SOURCE_DIR}/app/segway/event_group_manager
    ${CMAKE_SOURCE_DIR}/app/step_driver
    ${CMAKE_SOURCE_DIR}/app/a4988
    ${CMAKE_SOURCE_DIR}/app/utility
)

target_link_libraries(wheel_manager PRIVATE
    common
    task_manager
    queue_manager
    event_group_manager
    step_driver
    a4988
    utility
)

target_compile_options(wheel_manager PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -Wconversion
    -Wshadow
    -Wpedantic
    -Wnarrowing
    -Waddress
    -pedantic
    -Wdeprecated
    -Wsign-conversion
    -Wduplicated-cond
    -Wduplicated-branches
    -Wlogical-op
    -Wnull-dereference
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wcast-align
    -fconcepts
)