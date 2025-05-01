add_library(common INTERFACE)

target_sources(common INTERFACE 
    "log.hpp"
)

target_link_libraries(common INTERFACE
    stm32cubemx
)

target_include_directories(common INTERFACE 
    ${CMAKE_DIR}/stm32cubemx
)

target_compile_options(common INTERFACE
    -std=c++23
    -fexceptions
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
