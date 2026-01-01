add_library(wheel_manager STATIC)

target_sources(wheel_manager PRIVATE 
    "wheel_manager.cpp"
    "wheel.cpp"
)

target_include_directories(wheel_manager PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(wheel_manager PUBLIC
    common
    task_manager
    queue_manager
    event_group_manager
    step_driver
    a4988
    utility
    cubemx
    message_buf_manager
    gpio
)

target_compile_options(wheel_manager PUBLIC
    -std=c23
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
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
)