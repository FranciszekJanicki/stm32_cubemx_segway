add_library(control_manager STATIC)

target_sources(control_manager PRIVATE 
    "control_task.cpp"
    "control_queue.cpp"
    "control_manager.cpp"
)

target_include_directories(control_manager PUBLIC
    ${SEGWAY_DIR}/common
    ${SEGWAY_DIR}/task_manager
    ${SEGWAY_DIR}/queue_manager
    ${APP_DIR}/utility
)

target_link_libraries(control_manager PRIVATE
    common
    task_manager
    queue_manager
    utility
)

target_compile_options(control_manager PUBLIC
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