add_library(step_driver STATIC)

target_sources(step_driver PRIVATE 
    "step_driver.cpp"
)

target_include_directories(step_driver PRIVATE 
    ${CMAKE_SOURCE_DIR}/app/a4988
    ${CMAKE_SOURCE_DIR}/app/utility
)

target_link_libraries(step_driver PRIVATE
    a4988
    utility
)

target_compile_options(step_driver PUBLIC
    -std=c++23
    -Wall
    -Wextra
    -fexceptions
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