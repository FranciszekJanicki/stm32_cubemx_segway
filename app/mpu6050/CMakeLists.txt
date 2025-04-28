add_library(mpu6050 STATIC)

target_sources(mpu6050 PRIVATE 
    "mpu6050.cpp"
    "mpu6050_dmp.cpp"
    "mpu6050_utility.cpp"
)

target_include_directories(mpu6050 PUBLIC 
    ${CMAKE_SOURCE_DIR}/cmake/stm32cubemx
    ${CMAKE_SOURCE_DIR}/app/stm32_utility
    ${CMAKE_SOURCE_DIR}/app/utility
)

target_link_libraries(mpu6050 PRIVATE
    stm32cubemx
    stm32_utility
    utility
)

target_compile_options(mpu6050 PUBLIC
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
