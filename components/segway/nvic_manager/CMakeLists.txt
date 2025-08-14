add_library(nvic_manager STATIC)

target_sources(nvic_manager PRIVATE 
    "nvic_manager.cpp"
)

target_include_directories(nvic_manager PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(nvic_manager PUBLIC
    stm32cubemx
    event_group_manager
    task_manager
    common
)

target_compile_options(nvic_manager PUBLIC
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