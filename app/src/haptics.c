/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>

#include <zephyr/toolchain.h>

#include "controller.h"
#include "haptics.h"

__weak int haptics_backend_init(void)
{
	return 0;
}

__weak int haptics_backend_pulse(void)
{
	return -ENOTSUP;
}

__weak int haptics_backend_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
	ARG_UNUSED(frequency_hz);
	ARG_UNUSED(duration_ms);
	return -ENOTSUP;
}

int haptics_init(void)
{
	return haptics_backend_init();
}

void hardware_haptic_pulse(void)
{
	(void)haptics_backend_pulse();
}

int hardware_haptic_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
	return haptics_backend_tone(frequency_hz, duration_ms);
}
