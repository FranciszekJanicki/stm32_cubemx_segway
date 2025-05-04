add_library(control_manager STATIC)

target_sources(control_manager PRIVATE 
    "control_manager.cpp"
)

target_include_directories(control_manager PUBLIC
    ${SEGWAY_DIR}/common
    ${SEGWAY_DIR}/task_manager
    ${SEGWAY_DIR}/queue_manager
    ${APP_DIR}/utility
    ${STM32CUBEMX_DIR}
    ${SEGWAY_DIR}/event_group_manager
    ${SEGWAY_DIR}/message_buffer_manager
)

target_link_libraries(control_manager PRIVATE
    common
    task_manager
    queue_manager
    utility
    stm32cubemx
    event_group_manager
    message_buffer_manager
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