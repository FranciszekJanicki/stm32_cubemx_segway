add_library(segway STATIC)

target_sources(segway PRIVATE 
    "segway.cpp"
    "unit_test.cpp"
    "nvic.cpp"
)

target_include_directories(segway PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/segway
    ${CMAKE_CURRENT_SOURCE_DIR}/icm20948
    ${CMAKE_CURRENT_SOURCE_DIR}/a4988
)

target_link_libraries(segway PRIVATE
    icm20948
    step_driver
)

target_compile_options(segway PUBLIC
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