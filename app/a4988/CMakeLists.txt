add_library(a4988 STATIC)

target_sources(a4988 PRIVATE 
    "a4988.cpp"
)

target_include_directories(a4988 PUBLIC 
    ${CMAKE_SOURCE_DIR}/app/stm32_utility
    ${CMAKE_SOURCE_DIR}/app/utility
)

target_link_libraries(a4988 PRIVATE
    stm32_utility
    utility
)

target_compile_options(a4988 PUBLIC
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