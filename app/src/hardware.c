/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "analog.h"
#include "controller.h"
#include "haptics.h"
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
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_a))
	BUTTON_ENTRY(button_a, A),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_b))
	BUTTON_ENTRY(button_b, B),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_x))
	BUTTON_ENTRY(button_x, X),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_y))
	BUTTON_ENTRY(button_y, Y),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_qam))
	BUTTON_ENTRY(button_qam, QAM),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_right_stick))
	BUTTON_ENTRY(button_right_stick, RIGHT_STICK),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_view))
	BUTTON_ENTRY(button_view, VIEW),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_right_paddle1))
	BUTTON_ENTRY(button_right_paddle1, RIGHT_PADDLE1),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_right_paddle2))
	BUTTON_ENTRY(button_right_paddle2, RIGHT_PADDLE2),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_right_shoulder))
	BUTTON_ENTRY(button_right_shoulder, RIGHT_SHOULDER),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_dpad_down))
	BUTTON_ENTRY(button_dpad_down, DPAD_DOWN),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_dpad_right))
	BUTTON_ENTRY(button_dpad_right, DPAD_RIGHT),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_dpad_left))
	BUTTON_ENTRY(button_dpad_left, DPAD_LEFT),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_dpad_up))
	BUTTON_ENTRY(button_dpad_up, DPAD_UP),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_menu))
	BUTTON_ENTRY(button_menu, MENU),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_left_stick))
	BUTTON_ENTRY(button_left_stick, LEFT_STICK),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_steam))
	BUTTON_ENTRY(button_steam, STEAM),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_left_paddle1))
	BUTTON_ENTRY(button_left_paddle1, LEFT_PADDLE1),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_left_paddle2))
	BUTTON_ENTRY(button_left_paddle2, LEFT_PADDLE2),
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(button_left_shoulder))
	BUTTON_ENTRY(button_left_shoulder, LEFT_SHOULDER),
#endif
};
static struct gpio_callback button_callbacks[ARRAY_SIZE(buttons)];
K_SEM_DEFINE(input_changed, 0, 1);

#if CONFIG_IBEX_ACCELEROMETER && DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(accel0))
static const struct device *const accelerometer = DEVICE_DT_GET(DT_ALIAS(accel0));
#endif

static void button_changed(const struct device *port, struct gpio_callback *callback,
                           gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);
	hardware_signal_input_changed();
}

static int axis_to_report(const struct sensor_value *value)
{
	int64_t micro_ms2 = sensor_value_to_micro(value);
	int64_t scaled = micro_ms2 * 16384 / SENSOR_G;

	return CLAMP(scaled, INT16_MIN, INT16_MAX);
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

#if CONFIG_IBEX_ACCELEROMETER && DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(accel0))
	if(!device_is_ready(accelerometer))
	{
		LOG_WRN("accelerometer is not ready; reports will contain zeroes");
	}
#endif

#if CONFIG_IBEX_ANALOG_INPUTS
	err = analog_init();
	if(err)
	{
		LOG_WRN("analog inputs unavailable: %d", err);
	}
#endif

#if CONFIG_IBEX_OLYMPUS
	err = olympus_init();
	if(err)
	{
		LOG_WRN("Olympus touchpads unavailable: %d", err);
	}
#endif

	err = haptics_init();
	if(err)
	{
		LOG_WRN("haptics unavailable: %d", err);
	}

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

#if CONFIG_IBEX_ACCELEROMETER && DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(accel0))
	if(device_is_ready(accelerometer) && sensor_sample_fetch(accelerometer) == 0)
	{
		struct sensor_value xyz[3];

		if(sensor_channel_get(accelerometer, SENSOR_CHAN_ACCEL_XYZ, xyz) == 0)
		{
			report->accel_x = axis_to_report(&xyz[0]);
			report->accel_y = axis_to_report(&xyz[1]);
			report->accel_z = axis_to_report(&xyz[2]);
		}
	}
#endif

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
