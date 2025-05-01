add_library(mpu6050 STATIC)

target_sources(mpu6050 PRIVATE 
    "mpu6050.cpp"
    "mpu6050_dmp.cpp"
    "mpu6050_utility.cpp"
)

target_include_directories(mpu6050 PUBLIC 
    ${APP_DIR}/utility
)

target_link_libraries(mpu6050 PRIVATE
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
