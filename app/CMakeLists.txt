# SPDX-License-Identifier: AGPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(steam_controller)

target_sources(app PRIVATE
	src/battery.c
	src/calibration.c
	src/charge_mode.c
	src/hardware.c
	src/haptics.c
	src/ibex_settings_registry.c
	src/imu.c
	src/lizard.c
	src/main.c
	src/power.c
	src/puck_interface.c
	src/radio_personality.c
	src/shell.c
	src/transport.c
	src/valve_feature.c
	src/valve_hid_report_map.c
	src/valve_identity.c
	src/valve_settings.c
)

target_sources_ifdef(CONFIG_HAPTICS_PWM_NRF app PRIVATE src/haptics_pwm_nrf.c)
if(CONFIG_IBEX_HAPTICS_PWM)
	set(IBEX_HAPTICS_WAVEFORMS_H ${PROJECT_BINARY_DIR}/generated/haptics_waveforms.h)

	add_custom_command(
		OUTPUT ${IBEX_HAPTICS_WAVEFORMS_H}
		COMMAND ${PYTHON_EXECUTABLE}
			${CMAKE_CURRENT_SOURCE_DIR}/../scripts/haptics-wav-to-header.py
			--output ${IBEX_HAPTICS_WAVEFORMS_H}
			--input haptics_waveform_tick=${CMAKE_CURRENT_SOURCE_DIR}/assets/haptics/tick.wav
			--input haptics_waveform_click=${CMAKE_CURRENT_SOURCE_DIR}/assets/haptics/click.wav
			--input haptics_waveform_scream=${CMAKE_CURRENT_SOURCE_DIR}/assets/haptics/scream.wav
		DEPENDS
			${CMAKE_CURRENT_SOURCE_DIR}/../scripts/haptics-wav-to-header.py
			${CMAKE_CURRENT_SOURCE_DIR}/assets/haptics/tick.wav
			${CMAKE_CURRENT_SOURCE_DIR}/assets/haptics/click.wav
			${CMAKE_CURRENT_SOURCE_DIR}/assets/haptics/scream.wav
		COMMENT "Generating Ibex haptics waveform header"
		VERBATIM
	)
	add_custom_target(ibex_haptics_waveforms DEPENDS ${IBEX_HAPTICS_WAVEFORMS_H})
	add_dependencies(app ibex_haptics_waveforms)
	target_include_directories(app PRIVATE ${PROJECT_BINARY_DIR}/generated)
	target_sources(app PRIVATE src/haptics_pwm_ibex.c)
endif()
target_sources_ifdef(CONFIG_IBEX_ANALOG_INPUTS app PRIVATE src/analog.c)
target_sources_ifdef(CONFIG_IBEX_OLYMPUS app PRIVATE src/olympus.c)
target_sources_ifdef(CONFIG_IBEX_BLE app PRIVATE src/transport_ble.c)
target_sources_ifdef(CONFIG_IBEX_USB_HID app PRIVATE src/transport_usb.c)
target_sources_ifdef(CONFIG_IBEX_ESB app PRIVATE src/transport_esb.c)
target_sources_ifdef(CONFIG_IBEX_ESB app PRIVATE src/esb_backend_direct.c)
target_sources_ifdef(CONFIG_IBEX_RGBW_LED app PRIVATE src/rgbw_led.c)
target_sources_ifdef(CONFIG_GPIO_GREENPAK app PRIVATE drivers/gpio/gpio_greenpak.c)
target_sources_ifdef(CONFIG_BOARD_STEAM_CONTROLLER_IBEX app PRIVATE src/valve_nvs.c)

if(CONFIG_BOARD_STEAM_CONTROLLER_IBEX)
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
