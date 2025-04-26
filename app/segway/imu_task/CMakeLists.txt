add_library(imu_task STATIC)

target_sources(imu_task PRIVATE 
    "imu_task.cpp"
)

target_include_directories(imu_task PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/segway/common
)

target_link_libraries(imu_task PRIVATE
    common
)

target_compile_options(imu_task PUBLIC
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