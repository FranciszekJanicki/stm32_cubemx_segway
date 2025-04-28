add_library(nvic_manager STATIC)

target_sources(nvic_manager PRIVATE 
    "nvic_manager.cpp"
)

target_include_directories(nvic_manager PUBLIC 
    ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx
    ${CMAKE_SOURCE_DIR}/app/segway/event_group_manager
)

target_link_libraries(nvic_manager PRIVATE
    stm32cubemx
    event_group_manager
)

target_compile_options(nvic_manager PUBLIC
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