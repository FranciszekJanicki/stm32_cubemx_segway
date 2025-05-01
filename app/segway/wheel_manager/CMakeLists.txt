add_library(wheel_manager STATIC)

target_sources(wheel_manager PRIVATE 
    "wheel_task.cpp"
    "wheel_manager.cpp"
    "wheel_queue.cpp"
    "wheel_event_group.cpp"
    "wheel.cpp"
)

target_include_directories(wheel_manager PUBLIC 
    ${SEGWAY_DIR}/common
    ${SEGWAY_DIR}/task_manager
    ${SEGWAY_DIR}/queue_manager
    ${SEGWAY_DIR}/event_group_manager
    ${APP_DIR}/step_driver
    ${APP_DIR}/a4988
    ${APP_DIR}/utility
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