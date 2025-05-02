add_library(task_manager STATIC)

target_sources(task_manager PRIVATE 
    "task_manager.cpp"
)

target_include_directories(task_manager PUBLIC 
    ${STM32CUBEMX_DIR}
    ${SEGWAY_DIR}/common
)

target_link_libraries(task_manager PRIVATE
    stm32cubemx
    common
)

target_compile_options(task_manager PUBLIC
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