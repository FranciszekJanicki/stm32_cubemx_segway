add_library(event_group_manager STATIC)

target_sources(event_group_manager PRIVATE 
    "event_group_manager.cpp"
)

target_include_directories(event_group_manager PUBLIC 
    ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx
)

target_link_libraries(event_group_manager PRIVATE
    stm32cubemx
)

target_compile_options(event_group_manager PUBLIC
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