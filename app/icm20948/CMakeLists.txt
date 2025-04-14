add_library(icm20948)

target_sources(icm20948 PUBLIC 
    #"icm20948_dmp.cpp"
    "icm_sensor.c"
    "util/ICM_20948_C.c"
)

target_include_directories(icm20948 PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/icm20948
    ${CMAKE_SOURCE_DIR}/app/icm20948/util
)

target_link_libraries(icm20948 PRIVATE
    stm32cubemx
    stm32_utility
)

target_compile_options(icm20948 PUBLIC
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
