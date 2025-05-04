add_library(main_manager STATIC)

target_sources(main_manager PRIVATE 
    "main_manager.cpp"
)

target_include_directories(main_manager PUBLIC 
    ${STM32CUBEMX_DIR}
    ${SEGWAY_DIR}/event_group_manager
    ${SEGWAY_DIR}/common
    ${SEGWAY_DIR}/task_manager
)

target_link_libraries(main_manager PRIVATE
    stm32cubemx
    event_group_manager
    common
    task_manager
)

target_compile_options(main_manager PUBLIC
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