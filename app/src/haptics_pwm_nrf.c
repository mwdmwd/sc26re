/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include "haptics.h"

static const struct pwm_dt_spec haptic = PWM_DT_SPEC_GET(DT_ALIAS(haptic0));
K_MUTEX_DEFINE(haptic_mutex);

int haptics_backend_init(void)
{
	return pwm_is_ready_dt(&haptic) ? 0 : -ENODEV;
}

int haptics_backend_pulse(void)
{
	return haptics_backend_tone(2000, 15);
}

int haptics_backend_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
	uint32_t period_us;
	int err;

	if(!pwm_is_ready_dt(&haptic))
	{
		return -ENODEV;
	}
	if(frequency_hz == 0 || duration_ms == 0)
	{
		return -EINVAL;
	}

	period_us = 1000000U / frequency_hz;
	if(period_us < 2)
	{
		return -EINVAL;
	}

	k_mutex_lock(&haptic_mutex, K_FOREVER);
	err = pwm_set_dt(&haptic, PWM_USEC(period_us), PWM_USEC(period_us / 2));
	if(!err)
	{
		k_msleep(duration_ms);
		err = pwm_set_dt(&haptic, 0, 0);
	}
	k_mutex_unlock(&haptic_mutex);

	return err;
}
