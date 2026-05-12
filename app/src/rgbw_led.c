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

static uint8_t current_r, current_g, current_b, current_w;

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

void rgbw_led_set(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
	current_r = r;
	current_g = g;
	current_b = b;
	current_w = w;

	if(rgbw_led_set_channel(IBEX_RGBW_RED, r) ||
	   rgbw_led_set_channel(IBEX_RGBW_GREEN, g) ||
	   rgbw_led_set_channel(IBEX_RGBW_BLUE, b) ||
	   rgbw_led_set_channel(IBEX_RGBW_WHITE, w))
	{
		LOG_WRN("failed to set one or more LED channels");
	}
}

void rgbw_led_off(void)
{
	rgbw_led_set(0, 0, 0, 0);
}

void rgbw_led_prepare_poweroff(void)
{
	rgbw_led_off();

	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(1, 9));
	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(0, 22));
	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(1, 0));
	rgbw_led_force_pin_off(NRF_GPIO_PIN_MAP(1, 8));
}

void rgbw_led_get(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w)
{
	*r = current_r;
	*g = current_g;
	*b = current_b;
	*w = current_w;
}
