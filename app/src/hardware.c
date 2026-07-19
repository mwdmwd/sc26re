/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "analog.h"
#include "calibration.h"
#include "controller.h"
#include "haptics.h"
#include "imu.h"
#include "olympus.h"
#include "rgbw_led.h"

LOG_MODULE_REGISTER(hardware);

#define HARDWARE_REPORT_POLL_MS 4

struct hardware_button
{
	struct gpio_dt_spec gpio;
	uint32_t report_mask;
};

#define BUTTON_ENTRY(alias, name) \
	{ .gpio = GPIO_DT_SPEC_GET(DT_ALIAS(alias), gpios), \
	  .report_mask = BIT(CONTROLLER_BUTTON_##name) }

static const struct hardware_button buttons[] = {
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_a), okay)
	BUTTON_ENTRY(button_a, A),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_b), okay)
	BUTTON_ENTRY(button_b, B),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_x), okay)
	BUTTON_ENTRY(button_x, X),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_y), okay)
	BUTTON_ENTRY(button_y, Y),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_qam), okay)
	BUTTON_ENTRY(button_qam, QAM),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_right_stick), okay)
	BUTTON_ENTRY(button_right_stick, RIGHT_STICK),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_view), okay)
	BUTTON_ENTRY(button_view, VIEW),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_right_paddle1), okay)
	BUTTON_ENTRY(button_right_paddle1, RIGHT_PADDLE1),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_right_paddle2), okay)
	BUTTON_ENTRY(button_right_paddle2, RIGHT_PADDLE2),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_right_shoulder), okay)
	BUTTON_ENTRY(button_right_shoulder, RIGHT_SHOULDER),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_dpad_down), okay)
	BUTTON_ENTRY(button_dpad_down, DPAD_DOWN),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_dpad_right), okay)
	BUTTON_ENTRY(button_dpad_right, DPAD_RIGHT),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_dpad_left), okay)
	BUTTON_ENTRY(button_dpad_left, DPAD_LEFT),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_dpad_up), okay)
	BUTTON_ENTRY(button_dpad_up, DPAD_UP),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_menu), okay)
	BUTTON_ENTRY(button_menu, MENU),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_left_stick), okay)
	BUTTON_ENTRY(button_left_stick, LEFT_STICK),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_steam), okay)
	BUTTON_ENTRY(button_steam, STEAM),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_left_paddle1), okay)
	BUTTON_ENTRY(button_left_paddle1, LEFT_PADDLE1),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_left_paddle2), okay)
	BUTTON_ENTRY(button_left_paddle2, LEFT_PADDLE2),
#endif
#if DT_NODE_HAS_STATUS(DT_ALIAS(button_left_shoulder), okay)
	BUTTON_ENTRY(button_left_shoulder, LEFT_SHOULDER),
#endif
};
static struct gpio_callback button_callbacks[ARRAY_SIZE(buttons)];
K_SEM_DEFINE(input_changed, 0, 1);
static uint8_t imu_scan_error_logs;

static void button_changed(const struct device *port, struct gpio_callback *callback,
                           gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);
	hardware_signal_input_changed();
}

uint32_t hardware_read_buttons(void)
{
	uint32_t pressed = 0;

	for(size_t i = 0; i < ARRAY_SIZE(buttons); ++i)
	{
		if(gpio_pin_get_dt(&buttons[i].gpio) > 0)
		{
			pressed |= buttons[i].report_mask;
		}
	}

	return pressed;
}

int hardware_init(void)
{
	int err;

	if(IS_ENABLED(CONFIG_SETTINGS) && IS_ENABLED(CONFIG_BOARD_STEAM_CONTROLLER_IBEX))
	{
		(void)calibration_import_imu_from_valve_storage();
	}

	for(size_t i = 0; i < ARRAY_SIZE(buttons); ++i)
	{
		if(!gpio_is_ready_dt(&buttons[i].gpio))
		{
			LOG_ERR("button %u GPIO is not ready", i);
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&buttons[i].gpio, GPIO_INPUT);
		if(err)
		{
			return err;
		}

		err = gpio_pin_interrupt_configure_dt(&buttons[i].gpio, GPIO_INT_EDGE_BOTH);
		if(err)
		{
			return err;
		}

		gpio_init_callback(&button_callbacks[i], button_changed, BIT(buttons[i].gpio.pin));
		err = gpio_add_callback(buttons[i].gpio.port, &button_callbacks[i]);
		if(err)
		{
			return err;
		}
	}

	err = imu_init();
	if(err)
	{
		LOG_WRN("IMU unavailable: %d", err);
	}

#if CONFIG_IBEX_ANALOG_INPUTS
	err = analog_init();
	if(err)
	{
		LOG_WRN("analog inputs unavailable: %d", err);
	}
#endif

	err = haptics_init();
	if(err)
	{
		LOG_WRN("haptics unavailable: %d", err);
	}

#if CONFIG_IBEX_OLYMPUS
	err = olympus_init();
	if(err)
	{
		LOG_WRN("Olympus touchpads unavailable: %d", err);
	}
#endif

#if CONFIG_IBEX_RGBW_LED
	err = rgbw_led_init();
	if(err)
	{
		LOG_WRN("RGBW LED unavailable: %d", err);
	}
#endif

	return 0;
}

void hardware_read_report(struct controller_report *report)
{
	memset(report, 0, sizeof(*report));

	report->buttons = hardware_read_buttons();

	int err = imu_read_report(report);
	if(err && imu_scan_error_logs < 8)
	{
		imu_scan_error_logs++;
		LOG_WRN("IMU scan failed: %d", err);
	}

#if CONFIG_IBEX_ANALOG_INPUTS
	if(analog_read_report(report) != 0)
	{
		LOG_WRN("Ibex analog scan failed");
	}
#endif

#if CONFIG_IBEX_OLYMPUS
	olympus_read_report(report);
#endif
}

void hardware_wait_for_change(void)
{
	(void)k_sem_take(&input_changed, K_MSEC(HARDWARE_REPORT_POLL_MS));
}

void hardware_signal_input_changed(void)
{
	k_sem_give(&input_changed);
}

bool hardware_pairing_chord_pressed(void)
{
	uint32_t pressed = hardware_read_buttons();

	return (pressed & (BIT(CONTROLLER_BUTTON_A) | BIT(CONTROLLER_BUTTON_B))) ==
	       (BIT(CONTROLLER_BUTTON_A) | BIT(CONTROLLER_BUTTON_B));
}
