add_executable(main)

target_sources(main PRIVATE 
    "main.cpp"
)

target_include_directories(main PRIVATE 
    ${CMAKE_DIR}/stm32cubemx
    ${SEGWAY_DIR}/main_manager
)

target_link_libraries(main PUBLIC
    stm32cubemx
    main_manager
)

target_compile_options(main PUBLIC
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
