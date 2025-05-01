include make/build.mk
include make/clang.mk
include make/cubemx.mk
include make/flash.mk
include make/monitor.mk
include make/third_party.mk

.PHONY: flash_monitor_uart
flash_monitor_uart: flash_uart monitor_uart

.PHONY: flash_monitor_usb
flash_monitor_usb: flash_usb monitor_usb

.PHONY: all
all:
	$(MAKE) build && $(MAKE) flash_uart && $(MAKE) monitor_uart
