add_library(utility INTERFACE)

target_sources(utility INTERFACE 
    "pid.hpp"
    "utility.hpp"
    "vector3d.hpp"
    "quaternion3d.hpp"
    "sfr.hpp"
    "sfo.hpp"
    "matrix.hpp"
    "rotation3d.hpp"
    "kalman.hpp"
)

target_include_directories(utility INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(utility INTERFACE
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
