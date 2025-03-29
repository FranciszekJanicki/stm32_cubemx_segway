#!/bin/bash

# Define paths for the missing files and their destination directories
hal_conf_file="Core/Inc/stm32f4xx_hal_conf.h"
stm32f4xx_file="Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
destination_hal_conf="Drivers/STM32F4xx_HAL_Driver/Inc"
destination_stm32f4xx="Drivers/CMSIS/Device/ST/STM32F4xx/Include"

project_root=$(pwd)  # Get current working directory (assumes this script is run from the root of the project)

# Check if 'stm32f4xx_hal_conf.h' exists in Core/Inc and copy it if missing
if [ -f "${project_root}/${hal_conf_file}" ]; then
    echo "'stm32f4xx_hal_conf.h' found in '${hal_conf_file}'."
    
    # Ensure the destination directory exists
    if [ -d "${project_root}/${destination_hal_conf}" ]; then
        cp "${project_root}/${hal_conf_file}" "${project_root}/${destination_hal_conf}/"
        echo "File copied to '${destination_hal_conf}/stm32f4xx_hal_conf.h'."
    else
        echo "Destination directory '${destination_hal_conf}' does not exist."
        exit 1
    fi
else
    echo "'stm32f4xx_hal_conf.h' not found in '${hal_conf_file}'."
    exit 1
fi

# Check if 'stm32f4xx.h' exists in Drivers/CMSIS/Device/ST/STM32F4xx/Include and copy it if missing
if [ -f "${project_root}/${stm32f4xx_file}" ]; then
    echo "'stm32f4xx.h' found in '${stm32f4xx_file}'."
    
    # Ensure the destination directory exists
    if [ -d "${project_root}/${destination_stm32f4xx}" ]; then
        cp "${project_root}/${stm32f4xx_file}" "${project_root}/${destination_stm32f4xx}/"
        echo "File copied to '${destination_stm32f4xx}/stm32f4xx.h'."
    else
        echo "Destination directory '${destination_stm32f4xx}' does not exist."
        exit 1
    fi
else
    echo "'stm32f4xx.h' not found in '${stm32f4xx_file}'."
    exit 1
fi
