add_library(gpio STATIC)

target_sources(gpio PRIVATE 
    "gpio.cpp"
)

target_include_directories(gpio PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(gpio PUBLIC
    cubemx
)

target_compile_options(gpio PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -fexceptions
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