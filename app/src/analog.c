/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <limits.h>
#include <stdlib.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "analog.h"
#include "calibration.h"
#include "ibex_settings_registry.h"

LOG_MODULE_REGISTER(analog);

#define ANALOG_NODE DT_PATH(ibex_analog_inputs)
#define ANALOG_CHANNEL_COUNT 6
#define ANALOG_SAMPLE_COUNT 4
#define ANALOG_POWER_SETTLE_US 200
#define STICK_CENTER_MV 1650
#define STICK_ENDPOINT_MARGIN_PERCENT 2
#define TRIGGER_FULL_SCALE_MV 1650
#define TRIGGER_ENDPOINT_TRIM_PERCENT 2
#define TRIGGER_RANGE_MARGIN_PERCENT 15
#define TRIGGER_CLICK_THRESHOLD_SETTING SETTING_TRIGGER_THRESHOLD_PERCENT
#define TRIGGER_CLICK_RELEASE_HYSTERESIS_PERCENT 5
#define PUCK_PILOT_SAMPLE_COUNT 3
#define PUCK_PILOT_PRESENT_MIN_MV 2000

enum analog_channel
{
	ANALOG_STICK_LEFT_X,
	ANALOG_STICK_LEFT_Y,
	ANALOG_STICK_RIGHT_X,
	ANALOG_PUCK_PILOT = ANALOG_STICK_RIGHT_X,
	ANALOG_STICK_RIGHT_Y,
	ANALOG_TRIGGER_LEFT,
	ANALOG_TRIGGER_RIGHT,
};

static const struct adc_dt_spec analog_channels[] = {
	ADC_DT_SPEC_GET_BY_NAME(ANALOG_NODE, stick_left_x),
	ADC_DT_SPEC_GET_BY_NAME(ANALOG_NODE, stick_left_y),
	ADC_DT_SPEC_GET_BY_NAME(ANALOG_NODE, stick_right_x),
	ADC_DT_SPEC_GET_BY_NAME(ANALOG_NODE, stick_right_y),
	ADC_DT_SPEC_GET_BY_NAME(ANALOG_NODE, trigger_left),
	ADC_DT_SPEC_GET_BY_NAME(ANALOG_NODE, trigger_right),
};
static const struct gpio_dt_spec analog_enable = GPIO_DT_SPEC_GET(ANALOG_NODE, enable_gpios);
static bool trigger_left_click_latched;
static bool trigger_right_click_latched;
static bool analog_ready;
K_MUTEX_DEFINE(analog_lock);

BUILD_ASSERT(ARRAY_SIZE(analog_channels) == ANALOG_CHANNEL_COUNT);

static int16_t scale_stick_axis(int32_t sample, uint16_t min, uint16_t max, uint16_t center_min,
                                uint16_t center_max)
{
	if(min == 0 && max == 0)
	{
		return CLAMP((sample - STICK_CENTER_MV) * 20, INT16_MIN, INT16_MAX);
	}

	if(sample < center_min)
	{
		int32_t denom = center_min - min;

		if(denom <= 0)
		{
			return 0;
		}
		denom = MAX((denom * (100 - STICK_ENDPOINT_MARGIN_PERCENT)) / 100, 1);
		int64_t val = ((int64_t)(sample - center_min) * 32768) / denom;

		return CLAMP(val, INT16_MIN, 0);
	}
	else if(sample > center_max)
	{
		int32_t denom = max - center_max;

		if(denom <= 0)
		{
			return 0;
		}
		denom = MAX((denom * (100 - STICK_ENDPOINT_MARGIN_PERCENT)) / 100, 1);
		int64_t val = ((int64_t)(sample - center_max) * 32767) / denom;

		return CLAMP(val, 0, INT16_MAX);
	}
	else
	{
		return 0;
	}
}

static int16_t trigger_to_report(int32_t sample, const struct trigger_calibration *cal)
{
	if(!calibration_trigger_valid(cal))
	{
		return CLAMP((sample * INT16_MAX) / TRIGGER_FULL_SCALE_MV, 0, INT16_MAX);
	}

	int32_t low = MIN(cal->pressed, cal->idle);
	int32_t high = MAX(cal->pressed, cal->idle);

	if(cal->inverted)
	{
		low = (low * 100 + (100 - TRIGGER_ENDPOINT_TRIM_PERCENT) - 1) /
		      (100 - TRIGGER_ENDPOINT_TRIM_PERCENT);
	}
	else
	{
		high = (high * (100 - TRIGGER_ENDPOINT_TRIM_PERCENT)) / 100;
	}

	int32_t full_range = high - low;
	if(full_range <= 0)
	{
		return 0;
	}

	int32_t margin = (full_range * TRIGGER_RANGE_MARGIN_PERCENT) / 100;
	int32_t active_low = low + margin;
	int32_t active_high = high - margin;
	int32_t active_range = active_high - active_low;

	if(active_range <= 0)
	{
		return 0;
	}

	int64_t normalized = ((int64_t)(sample - active_low) * INT16_MAX) / active_range;

	if(cal->inverted)
	{
		normalized = INT16_MAX - normalized;
	}

	return CLAMP(normalized, 0, INT16_MAX);
}

static bool trigger_click_active(int16_t value, bool *latched)
{
	int16_t threshold_percent = 90;

	(void)ibex_setting_get(TRIGGER_CLICK_THRESHOLD_SETTING, &threshold_percent);
	threshold_percent = CLAMP(threshold_percent, 0, 100);

	int16_t release_percent = MAX(threshold_percent - TRIGGER_CLICK_RELEASE_HYSTERESIS_PERCENT, 0);
	int32_t press_threshold = ((int32_t)INT16_MAX * threshold_percent) / 100;
	int32_t release_threshold = ((int32_t)INT16_MAX * release_percent) / 100;

	if(*latched)
	{
		*latched = value >= release_threshold;
	}
	else
	{
		*latched = value >= press_threshold;
	}

	return *latched;
}

int analog_init(void)
{
	int err;

	if(!adc_is_ready_dt(&analog_channels[0]))
	{
		LOG_ERR("Ibex SAADC is not ready");
		return -ENODEV;
	}
	if(!gpio_is_ready_dt(&analog_enable))
	{
		LOG_ERR("Ibex analog enable GPIO is not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&analog_enable, GPIO_OUTPUT_INACTIVE);
	if(err)
	{
		LOG_ERR("analog enable GPIO setup failed: %d", err);
		return err;
	}

	for(size_t i = 0; i < ARRAY_SIZE(analog_channels); ++i)
	{
		err = adc_channel_setup_dt(&analog_channels[i]);
		if(err)
		{
			LOG_ERR("SAADC channel %u setup failed: %d", i, err);
			return err;
		}
	}

	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		err = calibration_load_settings();
		if(err)
		{
			LOG_WRN("failed to load analog calibration settings: %d", err);
		}
	}

	if(IS_ENABLED(CONFIG_SETTINGS) && (!calibration_trigger_loaded(CALIBRATION_LEFT) ||
	                                   !calibration_trigger_loaded(CALIBRATION_RIGHT) ||
	                                   !calibration_stick_loaded(CALIBRATION_LEFT) ||
	                                   !calibration_stick_loaded(CALIBRATION_RIGHT)))
	{
		(void)calibration_import_valve_storage();
	}

	if(!calibration_trigger_loaded(CALIBRATION_LEFT) ||
	   !calibration_trigger_loaded(CALIBRATION_RIGHT) ||
	   !calibration_stick_loaded(CALIBRATION_LEFT) ||
	   !calibration_stick_loaded(CALIBRATION_RIGHT))
	{
		LOG_WRN("Using raw analog fallback for missing calibration: trg_l=%u trg_r=%u joy_l=%u "
		        "joy_r=%u",
		        calibration_trigger_loaded(CALIBRATION_LEFT),
		        calibration_trigger_loaded(CALIBRATION_RIGHT),
		        calibration_stick_loaded(CALIBRATION_LEFT),
		        calibration_stick_loaded(CALIBRATION_RIGHT));
	}
	analog_ready = true;
	return 0;
}

static int analog_take(void)
{
	int err;

	k_mutex_lock(&analog_lock, K_FOREVER);
	err = gpio_pin_set_dt(&analog_enable, 1);
	if(err)
	{
		k_mutex_unlock(&analog_lock);
		return err;
	}
	k_usleep(ANALOG_POWER_SETTLE_US);
	return 0;
}

static int analog_put(void)
{
	int err;

	__ASSERT_NO_MSG(analog_lock.owner == k_current_get());

	err = gpio_pin_set_dt(&analog_enable, 0);
	k_mutex_unlock(&analog_lock);
	return err;
}

int analog_puck_pilot_present(bool *present)
{
	const struct adc_dt_spec *channel = &analog_channels[ANALOG_PUCK_PILOT];
	int16_t raw;
	int32_t mv;
	int present_samples = 0;
	int err;

	if(present == NULL)
	{
		return -EINVAL;
	}
	*present = false;

	if(!analog_ready)
	{
		return -ENODEV;
	}

	err = analog_take();
	if(err)
	{
		return err;
	}

	for(size_t i = 0; i < PUCK_PILOT_SAMPLE_COUNT; ++i)
	{
		struct adc_sequence sequence = {
			.channels = BIT(channel->channel_id),
			.buffer = &raw,
			.buffer_size = sizeof(raw),
			.resolution = 12,
		};

		err = adc_read(channel->dev, &sequence);
		if(err)
		{
			goto out_disable;
		}

		mv = raw;
		err = adc_raw_to_millivolts_dt(channel, &mv);
		if(err)
		{
			goto out_disable;
		}

		present_samples += mv >= PUCK_PILOT_PRESENT_MIN_MV;
		k_msleep(100);
	}

	*present = present_samples > PUCK_PILOT_SAMPLE_COUNT / 2;
	err = 0;

out_disable:
	analog_put();
	return err;
}

int analog_read_report(struct controller_report *report)
{
	int16_t samples[ANALOG_CHANNEL_COUNT * ANALOG_SAMPLE_COUNT];
	int32_t averaged[ANALOG_CHANNEL_COUNT] = { 0 };
	struct adc_sequence_options options = {
		.extra_samplings = ANALOG_SAMPLE_COUNT - 1,
	};
	struct adc_sequence sequence = {
		.options = &options,
		.channels = BIT_MASK(ANALOG_CHANNEL_COUNT),
		.buffer = samples,
		.buffer_size = sizeof(samples),
		.resolution = 12,
	};
	int err;

	if(!analog_ready)
	{
		return -ENODEV;
	}

	err = analog_take();
	if(err)
	{
		return err;
	}

	err = adc_read(analog_channels[0].dev, &sequence);
	analog_put();
	if(err)
	{
		return err;
	}

	for(size_t sample = 0; sample < ANALOG_SAMPLE_COUNT; ++sample)
	{
		for(size_t channel = 0; channel < ANALOG_CHANNEL_COUNT; ++channel)
		{
			averaged[channel] += samples[sample * ANALOG_CHANNEL_COUNT + channel];
		}
	}
	for(size_t channel = 0; channel < ANALOG_CHANNEL_COUNT; ++channel)
	{
		averaged[channel] /= ANALOG_SAMPLE_COUNT;
		err = adc_raw_to_millivolts_dt(&analog_channels[channel], &averaged[channel]);
		if(err)
		{
			LOG_WRN("SAADC channel %u raw-to-mV conversion failed: %d", channel, err);
			return err;
		}
	}

	const struct stick_calibration *left_stick = calibration_stick(CALIBRATION_LEFT);
	const struct stick_calibration *right_stick = calibration_stick(CALIBRATION_RIGHT);
	const struct trigger_calibration *left_trigger = calibration_trigger(CALIBRATION_LEFT);
	const struct trigger_calibration *right_trigger = calibration_trigger(CALIBRATION_RIGHT);

	report->stick_left_x =
	    scale_stick_axis(averaged[ANALOG_STICK_LEFT_X], left_stick->x_min, left_stick->x_max,
	                     left_stick->x_center_min, left_stick->x_center_max);
	report->stick_left_y =
	    scale_stick_axis(averaged[ANALOG_STICK_LEFT_Y], left_stick->y_min, left_stick->y_max,
	                     left_stick->y_center_min, left_stick->y_center_max);
	report->stick_right_x =
	    scale_stick_axis(averaged[ANALOG_STICK_RIGHT_X], right_stick->x_min, right_stick->x_max,
	                     right_stick->x_center_min, right_stick->x_center_max);
	report->stick_right_y =
	    scale_stick_axis(averaged[ANALOG_STICK_RIGHT_Y], right_stick->y_min, right_stick->y_max,
	                     right_stick->y_center_min, right_stick->y_center_max);
	report->trigger_left = trigger_to_report(averaged[ANALOG_TRIGGER_LEFT], left_trigger);
	report->trigger_right = trigger_to_report(averaged[ANALOG_TRIGGER_RIGHT], right_trigger);
	if(trigger_click_active(report->trigger_left, &trigger_left_click_latched))
	{
		report->buttons |= BIT(CONTROLLER_BUTTON_LEFT_TRIGGER_CLICK);
	}
	if(trigger_click_active(report->trigger_right, &trigger_right_click_latched))
	{
		report->buttons |= BIT(CONTROLLER_BUTTON_RIGHT_TRIGGER_CLICK);
	}

	err = 0;

	return err;
}
