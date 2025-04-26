cmake_minimum_required(VERSION 3.30)

include("cmake/gcc-arm-none-eabi.cmake")

project(stm32_cubemx_segway LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

message("Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

add_subdirectory(cmake/stm32cubemx)
add_subdirectory(app/utility)
add_subdirectory(app/stm32_utility)
add_subdirectory(app/a4988)
add_subdirectory(app/mpu6050)
add_subdirectory(app/icm20948)
add_subdirectory(app/step_driver)
add_subdirectory(app/segway/imu_task)
add_subdirectory(app/segway/main_task)
add_subdirectory(app/segway/control_task)
add_subdirectory(app/segway/wheel_task)
add_subdirectory(app/segway/common)
add_subdirectory(app/main)

target_compile_options(stm32cubemx INTERFACE 
    -w
)