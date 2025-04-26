add_library(main_task STATIC)

target_sources(main_task PRIVATE 
    "main_task.cpp"
)

target_include_directories(main_task PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/segway/control_task
    ${CMAKE_SOURCE_DIR}/app/segway/imu_task
    ${CMAKE_SOURCE_DIR}/app/segway/wheel_task
    ${CMAKE_SOURCE_DIR}/app/segway/common
)

target_link_libraries(main_task PRIVATE
    control_task
    imu_task
    wheel_task
    stm32cubemx
    common
)

target_compile_options(main_task PUBLIC
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