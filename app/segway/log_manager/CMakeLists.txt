add_library(log_manager STATIC)

target_sources(log_manager PRIVATE 
    "log_manager.cpp"
)

target_include_directories(log_manager PUBLIC 
    ${STM32CUBEMX_DIR}
    ${SEGWAY_DIR}/queue_manager
)

target_link_libraries(log_manager PRIVATE
    stm32cubemx
    queue_manager
)

target_compile_options(log_manager PUBLIC
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