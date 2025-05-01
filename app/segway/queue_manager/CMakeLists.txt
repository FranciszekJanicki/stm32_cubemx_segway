add_library(queue_manager STATIC)

target_sources(queue_manager PRIVATE 
    "queue_manager.cpp"
)

target_include_directories(queue_manager PUBLIC 
    ${STM32CUBEMX_DIR}
)

target_link_libraries(queue_manager PRIVATE
    stm32cubemx
)

target_compile_options(queue_manager PUBLIC
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