add_library(imu_manager STATIC)

target_sources(imu_manager PRIVATE 
    "imu_task.cpp"
    "imu_event_group.cpp"
    "imu_manager.cpp"
)

target_include_directories(imu_manager PUBLIC
    ${CMAKE_SOURCE_DIR}/app/segway/common
    ${CMAKE_SOURCE_DIR}/app/segway/task_manager
    ${CMAKE_SOURCE_DIR}/app/segway/event_group_manager
    ${CMAKE_SOURCE_DIR}/app/mpu6050
    ${CMAKE_SOURCE_DIR}/app/segway/queue_manager
)

target_link_libraries(imu_manager PRIVATE
    common
    task_manager
    event_group_manager
    mpu6050
    queue_manager
)

target_compile_options(imu_manager PUBLIC
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