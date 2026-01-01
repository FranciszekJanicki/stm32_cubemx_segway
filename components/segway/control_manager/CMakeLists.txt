add_library(control_manager STATIC)

target_sources(control_manager PRIVATE 
    "control_manager.cpp"
)

target_include_directories(control_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR} 
)

target_link_libraries(control_manager PRIVATE
    common
    task_manager
    queue_manager
    utility
    cubemx
    event_group_manager
    message_buf_manager
)

target_compile_options(control_manager PUBLIC
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