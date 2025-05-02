add_library(imu_manager STATIC)

target_sources(imu_manager PRIVATE
    "imu_manager.cpp"
)

target_include_directories(imu_manager PUBLIC
    ${SEGWAY_DIR}/common
    ${SEGWAY_DIR}/task_manager
    ${SEGWAY_DIR}/event_group_manager
    ${APP_DIR}/mpu6050
    ${SEGWAY_DIR}/queue_manager
    ${SEGWAY_DIR}/nvic_manager
    ${SEGWAY_DIR}/task_manager
    ${STM32CUBEMX_DIR}
)

target_link_libraries(imu_manager PRIVATE
    common
    task_manager
    event_group_manager
    mpu6050
    queue_manager
    nvic_manager
    task_manager
    stm32cubemx
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