add_library(segway STATIC)

target_sources(segway PRIVATE 
    "segway.cpp"
    "unit_test.cpp"
    "nvic.cpp"
    "wheel.cpp"
)

target_include_directories(segway PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/icm20948
    ${CMAKE_SOURCE_DIR}/app/a4988
    ${CMAKE_SOURCE_DIR}/app/step_driver
    ${CMAKE_SOURCE_DIR}/app/stm32_utility
    ${CMAKE_SOURCE_DIR}/app/mpu6050
    ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx
    ${CMAKE_SOURCE_DIR}/app/utility
)

target_link_libraries(segway PRIVATE
    icm20948
    step_driver
    stm32cubemx
    mpu6050
    stm32_utility
    a4988
)

target_compile_options(segway PUBLIC
    -std=c++20
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