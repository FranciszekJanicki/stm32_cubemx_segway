add_library(message_buf_manager STATIC)

target_sources(message_buf_manager PRIVATE 
    "message_buf_manager.cpp"
)

target_include_directories(message_buf_manager PUBLIC 
    ${STM32CUBEMX_DIR}
)

target_link_libraries(message_buf_manager PRIVATE
    stm32cubemx
)

target_compile_options(message_buf_manager PUBLIC
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