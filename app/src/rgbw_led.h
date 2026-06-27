/* SPDX-License-Identifier: AGPL-3.0-or-later */

#pragma once

#include <stdint.h>

/**
 * Initialize the RGBW LED control wrapper.
 *
 * Validates that the devicetree LED device is ready and turns the LEDs off.
 * Returns 0 on success or a negative errno on failure.
 */
int rgbw_led_init(void);

struct rgbw_color
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t w;
};

/**
 * Set the LED color as normalized 0-255 per channel.
 *
 * Each component maps to the in-tree Zephyr LED API brightness scale. The white
 * channel is independent from RGB.
 */
void rgbw_led_set(struct rgbw_color color);

/**
 * Turn all LED channels off.
 */
void rgbw_led_off(void);

/**
 * Drive LED pins to their inactive state before SYSTEMOFF.
 */
void rgbw_led_prepare_poweroff(void);

/**
 * Get the current RGBW values (0-255 per channel).
 */
struct rgbw_color rgbw_led_get(void);
