add_library(imu_manager STATIC)

target_sources(imu_manager PRIVATE
    "imu_manager.cpp"
)

target_include_directories(imu_manager PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(imu_manager PUBLIC
    common
    task_manager
    event_group_manager
    mpu6050
    queue_manager
    nvic_manager
    stm32cubemx
    utility
    message_buf_manager
)

target_compile_options(imu_manager PUBLIC
    -std=c23
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
    -Wformat=2
    -Wformat-security
    -Wmissing-prototypes
    -Wmissing-declarations
    -Wstrict-prototypes
    -Wold-style-definition
    -Wundef
    -Wvla
    -Wpointer-arith
    -Wstrict-aliasing=2
)