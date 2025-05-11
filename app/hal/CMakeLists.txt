add_library(hal STATIC)

target_sources(hal PRIVATE 
    "gpio.cpp"
)

target_include_directories(hal PRIVATE 
    ${CMAKE_DIR}/stm32cubemx 
)

target_link_libraries(hal PRIVATE
    stm32cubemx
)

target_compile_options(hal PUBLIC
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