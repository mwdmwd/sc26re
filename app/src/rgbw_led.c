/* SPDX-License-Identifier: AGPL-3.0-or-later */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_gpio.h>

#include "rgbw_led.h"

LOG_MODULE_REGISTER(led);

#define PWMRGBLEDS_NODE DT_NODELABEL(pwmrgbleds)
#define LED_OFF_LEVEL 1

enum ibex_rgbw_channel
{
	IBEX_RGBW_RED,
	IBEX_RGBW_GREEN,
	IBEX_RGBW_BLUE,
	IBEX_RGBW_WHITE,
};

static const struct device *const rgbw_leds = DEVICE_DT_GET(PWMRGBLEDS_NODE);

static struct rgbw_color current = { 0 };

static void rgbw_led_force_pin_off(uint32_t pin)
{
	nrf_gpio_pin_write(pin, LED_OFF_LEVEL);
	nrf_gpio_cfg_output(pin);
}

static int rgbw_led_set_channel(enum ibex_rgbw_channel channel, uint8_t brightness)
{
	uint8_t percent = (uint16_t)brightness * 100U / 255U;

	return led_set_brightness(rgbw_leds, channel, percent);
}

int rgbw_led_init(void)
{
	if(!device_is_ready(rgbw_leds))
	{
		LOG_ERR("RGBW LED device is not ready");
		return -ENODEV;
	}

	rgbw_led_off();
	LOG_DBG("RGBW LED initialized");
	return 0;
}

void rgbw_led_set(struct rgbw_color color)
{
	current = color;

	if(rgbw_led_set_channel(IBEX_RGBW_RED, current.r) ||
	   rgbw_led_set_channel(IBEX_RGBW_GREEN, current.g) ||
	   rgbw_led_set_channel(IBEX_RGBW_BLUE, current.b) ||
	   rgbw_led_set_channel(IBEX_RGBW_WHITE, current.w))
	{
		LOG_WRN("failed to set one or more LED channels");
	}
}

void rgbw_led_off(void)
{
	rgbw_led_set((struct rgbw_color){ 0 });
}

void rgbw_led_prepare_poweroff(void)
{
	rgbw_led_off();

	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(1, 9));
	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(0, 22));
	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(1, 0));
	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(1, 8));
}

struct rgbw_color rgbw_led_get(void)
{
	return current;
}
