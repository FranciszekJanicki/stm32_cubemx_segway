add_library(segway STATIC)

target_sources(segway PRIVATE 
    "segway.cpp"
    "unit_test.cpp"
    "balance_segway.cpp"
)

target_include_directories(segway PUBLIC 
    "."
    ${CMAKE_CURRENT_SOURCE_DIR}/segway
)

target_link_libraries(segway PRIVATE
    stm32cubemx
    icm20948
    step_driver
    hardware
    stm32_utility
)

target_compile_options(segway PUBLIC
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