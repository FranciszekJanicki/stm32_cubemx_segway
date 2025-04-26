add_library(wheel_task STATIC)

target_sources(wheel_task PRIVATE 
    "wheel_task.cpp"
)

target_include_directories(wheel_task PUBLIC
    ${CMAKE_SOURCE_DIR}/app/segway/common
)

target_link_libraries(wheel_task PRIVATE
    common
)

target_compile_options(wheel_task PUBLIC
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