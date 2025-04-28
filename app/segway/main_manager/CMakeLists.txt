add_library(main_manager STATIC)

target_sources(main_manager PRIVATE 
    "main_task.cpp"
)

target_include_directories(main_manager PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/segway/control_manager
    ${CMAKE_SOURCE_DIR}/app/segway/imu_manager
    ${CMAKE_SOURCE_DIR}/app/segway/wheel_manager
    ${CMAKE_SOURCE_DIR}/app/segway/common
)

target_link_libraries(main_manager PRIVATE
    control_manager
    imu_manager
    wheel_manager
    common
)

target_compile_options(main_manager PUBLIC
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