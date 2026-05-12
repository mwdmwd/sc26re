# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(steam_controller)

target_sources(app PRIVATE
	src/battery.c
	src/calibration.c
	src/hardware.c
	src/haptics.c
	src/ibex_settings_registry.c
	src/main.c
	src/power.c
	src/radio_personality.c
	src/shell.c
	src/transport.c
	src/valve_feature.c
	src/valve_hid_report_map.c
	src/valve_identity.c
	src/valve_settings.c
)

target_sources_ifdef(CONFIG_HAPTICS_PWM_NRF app PRIVATE src/haptics_pwm_nrf.c)
target_sources_ifdef(CONFIG_IBEX_ANALOG_INPUTS app PRIVATE src/analog.c)
target_sources_ifdef(CONFIG_IBEX_OLYMPUS app PRIVATE src/olympus.c)
target_sources_ifdef(CONFIG_IBEX_BLE app PRIVATE src/transport_ble.c)
target_sources_ifdef(CONFIG_IBEX_USB_HID app PRIVATE src/transport_usb.c)
target_sources_ifdef(CONFIG_IBEX_ESB app PRIVATE src/transport_esb.c)
target_sources_ifdef(CONFIG_SC26RE_ESB_BACKEND_NCS app PRIVATE src/esb_backend_ncs.c)
target_sources_ifdef(CONFIG_SC26RE_ESB_BACKEND_DIRECT app PRIVATE src/esb_backend_direct.c)
target_sources_ifdef(CONFIG_IBEX_RGBW_LED app PRIVATE src/rgbw_led.c)
target_sources_ifdef(CONFIG_GPIO_GREENPAK app PRIVATE drivers/gpio/gpio_greenpak.c)

if(CONFIG_BOARD_STEAM_CONTROLLER_IBEX)
	target_sources(app PRIVATE src/valve_bootloader.c)

	string(REGEX REPLACE "^0[xX]" "" IBEX_FW_TIMESTAMP "${CONFIG_IBEX_BUILD_TIMESTAMP}")
	string(TOUPPER "${IBEX_FW_TIMESTAMP}" IBEX_FW_TIMESTAMP)
	set(IBEX_FW_NAME "IBEX_CUSTOM_${IBEX_FW_TIMESTAMP}.fw")
	set(IBEX_FW_PATH "${PROJECT_BINARY_DIR}/zephyr/${IBEX_FW_NAME}")

	add_custom_target(ibex_fw ALL
		COMMAND ${PYTHON_EXECUTABLE}
			${CMAKE_CURRENT_SOURCE_DIR}/../scripts/package-ibex-fw.py
			--input ${PROJECT_BINARY_DIR}/zephyr/${KERNEL_BIN_NAME}
			--elf ${PROJECT_BINARY_DIR}/zephyr/${KERNEL_ELF_NAME}
			--output ${IBEX_FW_PATH}
		DEPENDS zephyr_final
		BYPRODUCTS
			${IBEX_FW_PATH}
		COMMENT "Packaging Valve Ibex firmware container"
		VERBATIM
	)
endif()
