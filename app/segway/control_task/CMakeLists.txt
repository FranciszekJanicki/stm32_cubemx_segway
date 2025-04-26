add_library(control_task STATIC)

target_sources(control_task PRIVATE 
    "control_task.cpp"
)

target_include_directories(control_task PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/segway/common
)

target_link_libraries(control_task PRIVATE
    common
)

target_compile_options(control_task PUBLIC
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