add_library(hardware STATIC)

target_sources(hardware PRIVATE 
    "gpio.cpp"
    "uart.cpp"
    "tim.cpp"
    "i2c.cpp"
)

target_include_directories(hardware PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}/hardware
)

target_link_libraries(hardware PRIVATE
    stm32cubemx
    utility
)

target_compile_options(hardware PUBLIC
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