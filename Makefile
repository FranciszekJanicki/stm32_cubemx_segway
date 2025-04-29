PROJECT_DIR := $(shell pwd)
BUILD_DIR := ${PROJECT_DIR}/build
APP_DIR := ${PROJECT_DIR}/app
DRIVERS_DIR := ${PROJECT_DIR}/Drivers
REQUIREMENTS_DIR := ${PROJECT_DIR}/requirements
STM32CUBEMX_DIR := ${PROJECT_DIR}/cmake/stm32cubemx
STM32_UTILITY_DIR := ${APP_DIR}/stm32_utility
UTILITY_DIR := ${APP_DIR}/utility

.PHONY: build
build: 
	cd ${BUILD_DIR} && make

.PHONY: clean
clean: 
	rm -rf ${BUILD_DIR}

.PHONY: cmake
cmake:
	cd ${PROJECT_DIR} && make clean && mkdir build && cmake -S . -B build

.PHONY: flash_uart
flash_uart: 
	STM32_Programmer_CLI -c port=swd -d ${BUILD_DIR}/app/main/main.elf -rst

.PHONY: flash_usb
flash_usb:
	STM32_Programmer_CLI -c port=/dev/ttyACM0 -d ${BUILD_DIR}/app/main/main.elf -rst

.PHONY: monitor_uart
monitor_uart:
	minicom -D /dev/ttyUSB0 -b 115200

.PHONY: monitor_usb
monitor_usb:
	minicom -D /dev/ttyACM0 -b 115200

.PHONY: clang_format
clang_format:
	for ext in h c cpp hpp; do \
		find $(SOURCE_DIR) -iname "*.$$ext" -print0 | xargs -0 -r clang-format -i; \
	done

.PHONY: cubemx
cubemx: 
	stm32cubemx

.PHONY: cubeprog
cubeprog:
	/opt/stm32cubeprog/bin/STM32CubeProgrammer

.PHONY: add_stm32_utility
add_stm32_utility:
	git submodule add -f https://github.com/franciszekjanicki/stm32_utility.git ${STM32_UTILITY_DIR}

.PHONY: remove_stm32_utility
remove_stm32_utility:
	git submodule deinit -f ${STM32_UTILITY_DIR}
	git rm -rf ${STM32_UTILITY_DIR}
	rm -rf ${STM32_UTILITY_DIR}
	rm -rf .git/modules/app/stm32_utility

.PHONY: add_utility
add_utility:
	git submodule add -f https://github.com/franciszekjanicki/utility.git ${UTILITY_DIR}

.PHONY: remove_utility
remove_utility:
	git submodule deinit -f ${UTILITY_DIR}
	git rm -rf ${UTILITY_DIR}
	rm -rf ${UTILITY_DIR}
	rm -rf .git/modules/app/utility

.PHONY: all
all:
	make build && make flash_uart && make monitor_uart
