/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "controller.h"
#include "haptics.h"
#include "ibex_settings_registry.h"
#include "sdl/controller_structs.h"

LOG_MODULE_REGISTER(haptics);

#define HAPTICS_THREAD_STACK_SIZE 768
#define HAPTICS_THREAD_PRIORITY 9
#define HAPTICS_QUEUE_DEPTH 20
#define HAPTICS_PCM_QUEUE_DEPTH 32
#define HAPTICS_REPORT_LOG_LIMIT 128
#define HAPTICS_REPORT_HEXDUMP_BYTES 24
#define HAPTICS_REPORT_PAYLOAD_BYTES(report_bytes) ((report_bytes) - 1U)
#define HAPTICS_SCRIPT_CHANNEL_COUNT 4
#define HAPTICS_SCRIPT_DELAY_ZERO_MS 1
#define HAPTICS_LIZARD_MOVE_THRESHOLD 1500
#define HAPTICS_LIZARD_TICK_GAIN_DB 2
#define HAPTICS_LIZARD_CLICK_GAIN_DB 6

/* Keep host report fields in the representable range; the backend handles output limits. */
#define HAPTICS_DEFAULT_PULSE_FREQUENCY_HZ 882
#define HAPTICS_CLICK_FREQUENCY_HZ 699
#define HAPTICS_RUMBLE_CARRIER_FREQUENCY_HZ 65
#define HAPTICS_RUMBLE_DURATION_MS 100
#define HAPTICS_RUMBLE_RIGHT_RANDOM_RATE_MIN_HZ 4.0f
#define HAPTICS_RUMBLE_RIGHT_RANDOM_RATE_RANGE_HZ 73.0f
#define HAPTICS_RUMBLE_LEFT_LFO_RATE_MIN_HZ 2.0f
#define HAPTICS_RUMBLE_LEFT_LFO_RATE_RANGE_HZ 15.0f
#define HAPTICS_RUMBLE_MAX_RATE_HZ 60.0f
#define HAPTICS_RUMBLE_LEFT_GAIN_OFFSET_DB 5
#define HAPTICS_RUMBLE_LFO_DEPTH 100
#define HAPTICS_SETTING_RUMBLE_GAIN_DB_OFFSET 27

/* OFW haptics table entries 0/1 are 70- and 93-sample fixed waveforms at 4 kHz. */
#define HAPTICS_TOUCHPAD_SAMPLE_RATE_HZ 4000U
#define HAPTICS_TICK_WAVEFORM_SAMPLES 70U
#define HAPTICS_CLICK_WAVEFORM_SAMPLES 93U
#define HAPTICS_WAVEFORM_DURATION_MS(samples) \
	DIV_ROUND_UP((samples) * 1000U, HAPTICS_TOUCHPAD_SAMPLE_RATE_HZ)
#define HAPTICS_TICK_DURATION_MS HAPTICS_WAVEFORM_DURATION_MS(HAPTICS_TICK_WAVEFORM_SAMPLES)
#define HAPTICS_CLICK_DURATION_MS HAPTICS_WAVEFORM_DURATION_MS(HAPTICS_CLICK_WAVEFORM_SAMPLES)
#define HAPTICS_PULSE_ONE_SHOT_GAIN_DB 2
#define HAPTICS_PULSE_CLICK_GAIN_DB 4

#define VALVE_HAPTIC_REPORT_CLASS 0x86
#define VALVE_HAPTIC_REPORT_PCM_MONO 0x87
#define VALVE_HAPTIC_REPORT_PCM_STEREO 0x88
#define VALVE_HAPTIC_REPORT_PCM_MONO_WITH_LENGTH 0x89
#define VALVE_HAPTIC_REPORT_GYRO_BIAS 0x8A
#define VALVE_HAPTIC_PCM_STEREO_SAMPLES 31

enum haptics_script_step_type
{
	HAPTICS_SCRIPT_STEP_END,
	HAPTICS_SCRIPT_STEP_NOOP,
	HAPTICS_SCRIPT_STEP_PULSE,
	HAPTICS_SCRIPT_STEP_TONE,
	HAPTICS_SCRIPT_STEP_LFO_TONE,
	HAPTICS_SCRIPT_STEP_LOG_SWEEP,
	HAPTICS_SCRIPT_STEP_SCREAM,
};

struct haptics_script_step
{
	uint16_t delay_ms;
	enum haptics_script_step_type type;
	uint8_t channels;
	int8_t gain_db;
	bool fixed_gain;
	uint16_t frequency_hz;
	uint16_t end_frequency_hz;
	uint16_t duration_ms;
	uint16_t lfo_frequency_hz;
	uint16_t on_us;
	uint16_t off_us;
	uint16_t repeat_count;
	uint8_t lfo_depth;
	uint8_t invert_channels;
};

struct haptics_script
{
	const struct haptics_script_step *steps;
};

struct haptics_script_state
{
	struct k_work_delayable work;
	const struct haptics_script *script;
	uint8_t channel;
	uint8_t index;
	int8_t gain_db;
	bool active;
};

struct haptics_touchpad_state
{
	int16_t x;
	int16_t y;
	uint32_t move_count;
	bool touched;
	bool pressure_active;
};

#define HAPTICS_SCRIPT_CURRENT_CHANNEL 0
#define HAPTICS_SCRIPT_END(delay) \
	{ .delay_ms = (delay), .type = HAPTICS_SCRIPT_STEP_END }
#define HAPTICS_SCRIPT_NOOP(delay) \
	{ .delay_ms = (delay), .type = HAPTICS_SCRIPT_STEP_NOOP }
#define HAPTICS_SCRIPT_PULSE(delay, on_us_, off_us_, repeat_count_) \
	{ \
		.delay_ms = (delay), \
		.type = HAPTICS_SCRIPT_STEP_PULSE, \
		.channels = HAPTICS_SCRIPT_CURRENT_CHANNEL, \
		.on_us = (on_us_), \
		.off_us = (off_us_), \
		.repeat_count = (repeat_count_), \
	}
#define HAPTICS_SCRIPT_TONE(delay, freq, duration, gain_offset) \
	{ \
		.delay_ms = (delay), \
		.type = HAPTICS_SCRIPT_STEP_TONE, \
		.channels = HAPTICS_SCRIPT_CURRENT_CHANNEL, \
		.gain_db = (gain_offset), \
		.frequency_hz = (freq), \
		.duration_ms = (duration), \
	}
#define HAPTICS_SCRIPT_LFO_TONE(delay, channels_, freq, duration, gain_offset, lfo_freq, depth) \
	{ \
		.delay_ms = (delay), \
		.type = HAPTICS_SCRIPT_STEP_LFO_TONE, \
		.channels = (channels_), \
		.gain_db = (gain_offset), \
		.frequency_hz = (freq), \
		.duration_ms = (duration), \
		.lfo_frequency_hz = (lfo_freq), \
		.lfo_depth = (depth), \
	}
#define HAPTICS_SCRIPT_LOG_SWEEP(delay, duration, start_freq, end_freq, gain_offset, invert) \
	{ \
		.delay_ms = (delay), \
		.type = HAPTICS_SCRIPT_STEP_LOG_SWEEP, \
		.channels = HAPTICS_SCRIPT_CURRENT_CHANNEL, \
		.gain_db = (gain_offset), \
		.frequency_hz = (start_freq), \
		.end_frequency_hz = (end_freq), \
		.duration_ms = (duration), \
		.invert_channels = (invert), \
	}
#define HAPTICS_SCRIPT_LOG_SWEEP_FIXED_GAIN(delay, duration, start_freq, end_freq, gain) \
	{ \
		.delay_ms = (delay), \
		.type = HAPTICS_SCRIPT_STEP_LOG_SWEEP, \
		.channels = HAPTICS_SCRIPT_CURRENT_CHANNEL, \
		.gain_db = (gain), \
		.fixed_gain = true, \
		.frequency_hz = (start_freq), \
		.end_frequency_hz = (end_freq), \
		.duration_ms = (duration), \
	}
#define HAPTICS_SCRIPT_SCREAM(delay) \
	{ \
		.delay_ms = (delay), \
		.type = HAPTICS_SCRIPT_STEP_SCREAM, \
		.channels = HAPTICS_SCRIPT_CURRENT_CHANNEL, \
		.fixed_gain = true, \
	}

/* OFW script table at 0x5CC68. */
/* clang-format off */
static const struct haptics_script_step haptics_script_1[] = {
	HAPTICS_SCRIPT_PULSE(0, 4000, 0, 1),
	HAPTICS_SCRIPT_TONE(30, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(81, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(81, 882, 80, -3),
	HAPTICS_SCRIPT_END(81),
};

static const struct haptics_script_step haptics_script_2[] = {
	HAPTICS_SCRIPT_PULSE(0, 4000, 0, 1),
	HAPTICS_SCRIPT_TONE(30, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(81, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(81, 882, 80, -3),
	HAPTICS_SCRIPT_TONE(81, 1176, 160, -22),
	HAPTICS_SCRIPT_END(160),
};

static const struct haptics_script_step haptics_script_3[] = {
	HAPTICS_SCRIPT_TONE(0, 900, 80, 0),
	HAPTICS_SCRIPT_TONE(120, 985, 80, 0),
	HAPTICS_SCRIPT_TONE(120, 900, 80, 0),
	HAPTICS_SCRIPT_TONE(120, 985, 80, 0),
	HAPTICS_SCRIPT_END(120),
};

static const struct haptics_script_step haptics_script_4[] = {
	HAPTICS_SCRIPT_TONE(0, 985, 80, 0),
	HAPTICS_SCRIPT_TONE(120, 900, 80, 0),
	HAPTICS_SCRIPT_TONE(120, 985, 80, 0),
	HAPTICS_SCRIPT_TONE(120, 900, 80, 0),
	HAPTICS_SCRIPT_END(120),
};

static const struct haptics_script_step haptics_script_5[] = {
	HAPTICS_SCRIPT_TONE(0, 882, 80, -3),
	HAPTICS_SCRIPT_TONE(81, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(81, 588, 80, 0),
	HAPTICS_SCRIPT_END(81),
};

static const struct haptics_script_step haptics_script_6[] = {
	HAPTICS_SCRIPT_TONE(0, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(81, 882, 80, -3),
	HAPTICS_SCRIPT_END(81),
};

static const struct haptics_script_step haptics_script_7[] = {
	HAPTICS_SCRIPT_TONE(0, 882, 80, -3),
	HAPTICS_SCRIPT_TONE(81, 588, 80, 0),
	HAPTICS_SCRIPT_END(81),
};

static const struct haptics_script_step haptics_script_8[] = {
	HAPTICS_SCRIPT_TONE(0, 588, 75, 0),
	HAPTICS_SCRIPT_TONE(150, 785, 75, 0),
	HAPTICS_SCRIPT_TONE(150, 882, 75, 0),
	HAPTICS_SCRIPT_TONE(150, 935, 75, 0),
	HAPTICS_SCRIPT_END(150),
};

static const struct haptics_script_step haptics_script_9[] = {
	HAPTICS_SCRIPT_TONE(0, 588, 75, 0),
	HAPTICS_SCRIPT_TONE(150, 935, 75, 0),
	HAPTICS_SCRIPT_TONE(150, 882, 75, 0),
	HAPTICS_SCRIPT_TONE(150, 785, 75, 0),
	HAPTICS_SCRIPT_END(150),
};

static const struct haptics_script_step haptics_script_10[] = {
	HAPTICS_SCRIPT_LOG_SWEEP(0, 300, 400, 1000, 0, 0),
	HAPTICS_SCRIPT_LOG_SWEEP(300, 300, 400, 1000, 0, 0),
	HAPTICS_SCRIPT_LOG_SWEEP(300, 300, 400, 1000, 0, 0),
	HAPTICS_SCRIPT_END(300),
};

static const struct haptics_script_step haptics_script_11[] = {
	HAPTICS_SCRIPT_LOG_SWEEP_FIXED_GAIN(0, 250, 935, 588, -6),
	HAPTICS_SCRIPT_LFO_TONE(250, HAPTICS_SCRIPT_CURRENT_CHANNEL, 50, 500, 0, 100, 100),
	HAPTICS_SCRIPT_END(500),
};

static const struct haptics_script_step haptics_script_12[] = {
	HAPTICS_SCRIPT_TONE(0, 65, 200, -3),
	HAPTICS_SCRIPT_TONE(30, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(61, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(61, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(61, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(61, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(61, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(61, 65, 200, -3),
	HAPTICS_SCRIPT_TONE(120, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(61, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(61, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(61, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(61, 699, 80, -3),
	HAPTICS_SCRIPT_TONE(61, 588, 80, 0),
	HAPTICS_SCRIPT_TONE(61, 65, 200, -3),
	HAPTICS_SCRIPT_END(81),
};

static const struct haptics_script_step haptics_script_13[] = {
	HAPTICS_SCRIPT_TONE(0, 65, 200, -3),
	HAPTICS_SCRIPT_LFO_TONE(30, HAPTICS_CHANNEL_RIGHT_1, 440, 1600, -3, 32, 100),
	HAPTICS_SCRIPT_LFO_TONE(1800, HAPTICS_CHANNEL_RIGHT_1, 440, 1600, -3, 32, 100),
	HAPTICS_SCRIPT_END(1200),
};

static const struct haptics_script_step haptics_script_14[] = {
	HAPTICS_SCRIPT_TONE(0, 65, 200, -3),
	HAPTICS_SCRIPT_TONE(30, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 500, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 620, 700, -6),
	HAPTICS_SCRIPT_TONE(70, 65, 200, -3),
	HAPTICS_SCRIPT_END(81),
};

static const struct haptics_script_step haptics_script_15[] = {
	HAPTICS_SCRIPT_TONE(0, 65, 200, -3),
	HAPTICS_SCRIPT_TONE(70, 1000, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1200, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1000, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1200, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1000, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1200, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1000, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1200, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1000, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1200, 70, -8),
	HAPTICS_SCRIPT_TONE(70, 1000, 70, -8),
	HAPTICS_SCRIPT_END(70),
};

static const struct haptics_script_step haptics_script_16[] = {
	HAPTICS_SCRIPT_SCREAM(0),
	HAPTICS_SCRIPT_END(240),
};

#define HAPTICS_SCRIPT_ENTRY(id) [((id)-1)] = { .steps = haptics_script_##id }
static const struct haptics_script haptics_scripts[] = {
	HAPTICS_SCRIPT_ENTRY(1),
	HAPTICS_SCRIPT_ENTRY(2),
	HAPTICS_SCRIPT_ENTRY(3),
	HAPTICS_SCRIPT_ENTRY(4),
	HAPTICS_SCRIPT_ENTRY(5),
	HAPTICS_SCRIPT_ENTRY(6),
	HAPTICS_SCRIPT_ENTRY(7),
	HAPTICS_SCRIPT_ENTRY(8),
	HAPTICS_SCRIPT_ENTRY(9),
	HAPTICS_SCRIPT_ENTRY(10),
	HAPTICS_SCRIPT_ENTRY(11),
	HAPTICS_SCRIPT_ENTRY(12),
	HAPTICS_SCRIPT_ENTRY(13),
	HAPTICS_SCRIPT_ENTRY(14),
	HAPTICS_SCRIPT_ENTRY(15),
	HAPTICS_SCRIPT_ENTRY(16),
};
/* clang-format on */

static struct k_thread haptics_thread;
static K_THREAD_STACK_DEFINE(haptics_stack, HAPTICS_THREAD_STACK_SIZE);
K_MSGQ_DEFINE(haptics_msgq, sizeof(struct haptics_effect), HAPTICS_QUEUE_DEPTH, 4);
static struct k_thread haptics_pcm_thread;
static K_THREAD_STACK_DEFINE(haptics_pcm_stack, HAPTICS_THREAD_STACK_SIZE);
K_MSGQ_DEFINE(haptics_pcm_msgq, sizeof(struct haptics_effect), HAPTICS_PCM_QUEUE_DEPTH, 4);
static atomic_t haptics_started;
static atomic_t haptics_ready;
static atomic_t haptics_report_log_count;
static atomic_t haptics_script_work_ready;
static atomic_t haptics_settings_callback_ready;
static atomic_t haptics_reports_seen;
static atomic_t haptics_effects_submitted;
static atomic_t haptics_effects_disabled;
static atomic_t haptics_effects_no_channel;
static atomic_t haptics_effects_queue_busy;
static atomic_t haptics_backend_calls;
static atomic_t haptics_backend_errors;
static atomic_t haptics_last_submit_err;
static atomic_t haptics_last_backend_err;
static atomic_t haptics_last_report_id;
static atomic_t haptics_last_effect_type;
static atomic_t haptics_last_effect_channels;
static K_MUTEX_DEFINE(haptics_script_mutex);
static struct haptics_script_state haptics_script_states[HAPTICS_SCRIPT_CHANNEL_COUNT] = {
	{ .channel = HAPTICS_CHANNEL_LEFT_0 },
	{ .channel = HAPTICS_CHANNEL_RIGHT_0 },
	{ .channel = HAPTICS_CHANNEL_LEFT_1 },
	{ .channel = HAPTICS_CHANNEL_RIGHT_1 },
};
static struct haptics_touchpad_state haptics_touchpad_states[2];

static void haptics_thread_main(void *arg1, void *arg2, void *arg3);
static void haptics_script_work_handler(struct k_work *work);

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

__weak int haptics_backend_effect(const struct haptics_effect *effect)
{
	switch(effect->type)
	{
		case HAPTICS_EFFECT_STOP:
		case HAPTICS_EFFECT_STOP_PCM:
			return 0;
		case HAPTICS_EFFECT_BUTTON_PULSE:
			return haptics_backend_pulse();
		case HAPTICS_EFFECT_TICK:
			return haptics_backend_tone(HAPTICS_DEFAULT_PULSE_FREQUENCY_HZ,
			                            HAPTICS_TICK_DURATION_MS);
		case HAPTICS_EFFECT_CLICK:
			return haptics_backend_tone(HAPTICS_CLICK_FREQUENCY_HZ, HAPTICS_CLICK_DURATION_MS);
		case HAPTICS_EFFECT_TONE:
		case HAPTICS_EFFECT_LFO_TONE:
		case HAPTICS_EFFECT_RANDOM_LFO:
			return haptics_backend_tone(effect->frequency_hz, effect->duration_ms);
		case HAPTICS_EFFECT_LOG_SWEEP:
			return haptics_backend_tone(effect->frequency_hz, effect->duration_ms);
		case HAPTICS_EFFECT_PCM_CONFIG:
		case HAPTICS_EFFECT_PCM_S8:
		case HAPTICS_EFFECT_SCREAM:
			return -ENOTSUP;
		case HAPTICS_EFFECT_PULSE_ONE_SHOT:
		case HAPTICS_EFFECT_PULSE_CLICK:
		case HAPTICS_EFFECT_PULSE_PERIODIC:
			return haptics_backend_tone(HAPTICS_DEFAULT_PULSE_FREQUENCY_HZ, effect->duration_ms);
		default:
			return -EINVAL;
	}
}

__weak int haptics_backend_get_debug(struct haptics_backend_debug *debug)
{
	if(debug == NULL)
	{
		return -EINVAL;
	}
	memset(debug, 0, sizeof(*debug));
	return -ENOTSUP;
}

__weak int haptics_backend_set_master_gain_db(int16_t gain_db)
{
	ARG_UNUSED(gain_db);
	return 0;
}

__weak int haptics_backend_set_amplifier_mode(bool forced_on)
{
	ARG_UNUSED(forced_on);
	return 0;
}

static void haptics_setting_changed(uint8_t id, int16_t value)
{
	if(id == SETTING_HAPTIC_MASTER_GAIN_DB)
	{
		(void)haptics_backend_set_master_gain_db(value);
	}
	else if(id == SETTING_TEST_CONTROL)
	{
		(void)haptics_backend_set_amplifier_mode(value != 0);
	}
}

int haptics_init(void)
{
	int err;

	if(atomic_get(&haptics_ready) != 0)
	{
		return 0;
	}

	err = haptics_backend_init();
	if(err)
	{
		return err;
	}
	if(atomic_cas(&haptics_settings_callback_ready, 0, 1))
	{
		err = ibex_settings_register_callback(haptics_setting_changed);
		if(err)
		{
			atomic_set(&haptics_settings_callback_ready, 0);
			return err;
		}
	}

	if(atomic_cas(&haptics_started, 0, 1))
	{
		k_thread_create(&haptics_thread, haptics_stack, K_THREAD_STACK_SIZEOF(haptics_stack),
		                haptics_thread_main, &haptics_msgq, NULL, NULL,
		                K_PRIO_PREEMPT(HAPTICS_THREAD_PRIORITY), K_FP_REGS, K_NO_WAIT);
		k_thread_create(&haptics_pcm_thread, haptics_pcm_stack,
		                K_THREAD_STACK_SIZEOF(haptics_pcm_stack), haptics_thread_main,
		                &haptics_pcm_msgq, NULL, NULL, K_PRIO_PREEMPT(HAPTICS_THREAD_PRIORITY),
		                K_FP_REGS, K_NO_WAIT);
	}
	if(atomic_cas(&haptics_script_work_ready, 0, 1))
	{
		for(size_t i = 0; i < ARRAY_SIZE(haptics_script_states); ++i)
		{
			k_work_init_delayable(&haptics_script_states[i].work, haptics_script_work_handler);
		}
	}

	atomic_set(&haptics_ready, 1);
	return 0;
}

static bool haptics_local_enabled(void)
{
	int16_t enabled = 1;

	if(ibex_setting_get(SETTING_HAPTICS_ENABLED, &enabled))
	{
		return enabled == 1;
	}
	return true;
}

static uint16_t safe_frequency(uint32_t frequency_hz)
{
	return MIN(frequency_hz, (uint32_t)UINT16_MAX);
}

static uint16_t safe_duration(uint32_t duration_ms)
{
	return CLAMP(duration_ms, 1U, (uint32_t)UINT16_MAX);
}

static uint16_t safe_pulse_duration(uint32_t on_us, uint32_t repeat_count)
{
	uint32_t duration_ms = DIV_ROUND_UP(on_us, 1000U) * repeat_count;

	return safe_duration(duration_ms);
}

static uint8_t haptics_command_channels(uint8_t side)
{
	switch(side & 0x7f)
	{
		case 0:
			return HAPTICS_CHANNEL_LEFT_0;
		case 1:
			return HAPTICS_CHANNEL_RIGHT_0;
		case 2:
		case 6:
			return HAPTICS_CHANNEL_LEFT_0 | HAPTICS_CHANNEL_RIGHT_0;
		case 3:
			return HAPTICS_CHANNEL_LEFT_1;
		case 4:
			return HAPTICS_CHANNEL_RIGHT_1;
		case 5:
		case 7:
			return HAPTICS_CHANNEL_LEFT_1 | HAPTICS_CHANNEL_RIGHT_1;
		default:
			return 0;
	}
}

static uint8_t haptics_pulse_channels(uint8_t side)
{
	/* OFW handles raw pulse sides 0/1 before applying the shared side table. */
	if(side == 0)
	{
		return HAPTICS_CHANNEL_RIGHT_0;
	}
	if(side == 1)
	{
		return HAPTICS_CHANNEL_LEFT_0;
	}
	return haptics_command_channels(side);
}

static int8_t clamp_gain_db(int gain_db)
{
	return (int8_t)CLAMP(gain_db, INT8_MIN, INT8_MAX);
}

static int rumble_base_gain_db(uint16_t intensity)
{
	int16_t offset_db = 0;
	float gain_db = ((float)(0x8000 - (int32_t)intensity) * 32.0f / 32768.0f) - 40.0f;

	(void)ibex_setting_get(HAPTICS_SETTING_RUMBLE_GAIN_DB_OFFSET, &offset_db);
	return (int)gain_db + offset_db;
}

static float rumble_modulation_rate_hz(uint16_t speed, float base_hz, float range_hz)
{
	float rate = ((float)speed * range_hz / 64000.0f) + base_hz;

	return CLAMP(rate, 0.0f, HAPTICS_RUMBLE_MAX_RATE_HZ);
}

static const char *haptics_channels_name(uint8_t channels)
{
	switch(channels)
	{
		case HAPTICS_CHANNEL_LEFT_0:
			return "left0";
		case HAPTICS_CHANNEL_RIGHT_0:
			return "right0";
		case HAPTICS_CHANNEL_LEFT_0 | HAPTICS_CHANNEL_RIGHT_0:
			return "primary";
		case HAPTICS_CHANNEL_LEFT_1:
			return "left1";
		case HAPTICS_CHANNEL_RIGHT_1:
			return "right1";
		case HAPTICS_CHANNEL_LEFT_1 | HAPTICS_CHANNEL_RIGHT_1:
			return "secondary";
		case HAPTICS_CHANNELS_ALL:
			return "all";
		default:
			return channels == 0 ? "none" : "mixed";
	}
}

static const char *haptics_effect_name(enum haptics_effect_type type)
{
	switch(type)
	{
		case HAPTICS_EFFECT_STOP:
			return "stop";
		case HAPTICS_EFFECT_STOP_PCM:
			return "stop_pcm";
		case HAPTICS_EFFECT_BUTTON_PULSE:
			return "button_pulse";
		case HAPTICS_EFFECT_TICK:
			return "tick";
		case HAPTICS_EFFECT_CLICK:
			return "click";
		case HAPTICS_EFFECT_TONE:
			return "tone";
		case HAPTICS_EFFECT_PULSE_ONE_SHOT:
			return "pulse_one_shot";
		case HAPTICS_EFFECT_PULSE_CLICK:
			return "pulse_click";
		case HAPTICS_EFFECT_PULSE_PERIODIC:
			return "pulse_periodic";
		case HAPTICS_EFFECT_LFO_TONE:
			return "lfo_tone";
		case HAPTICS_EFFECT_RANDOM_LFO:
			return "random_lfo";
		case HAPTICS_EFFECT_LOG_SWEEP:
			return "log_sweep";
		case HAPTICS_EFFECT_PCM_CONFIG:
			return "pcm_config";
		case HAPTICS_EFFECT_PCM_S8:
			return "pcm_s8";
		case HAPTICS_EFFECT_SCREAM:
			return "scream";
		default:
			return "?";
	}
}

static int haptics_submit_effect(const struct haptics_effect *effect)
{
	struct k_msgq *msgq = &haptics_msgq;
	int err;

	LOG_DBG("haptics effect=%s channels=%s gain_db=%d duration_ms=%u",
	        haptics_effect_name(effect->type), haptics_channels_name(effect->channels),
	        effect->gain_db, effect->duration_ms);
	atomic_set(&haptics_last_effect_type, effect->type);
	atomic_set(&haptics_last_effect_channels, effect->channels);
	if(atomic_get(&haptics_ready) == 0)
	{
		atomic_set(&haptics_last_submit_err, -ENODEV);
		return -ENODEV;
	}
	if(effect->channels == 0)
	{
		atomic_inc(&haptics_effects_no_channel);
		atomic_set(&haptics_last_submit_err, 0);
		return 0;
	}

	if(effect->type == HAPTICS_EFFECT_STOP_PCM ||
	   effect->type == HAPTICS_EFFECT_PCM_CONFIG ||
	   effect->type == HAPTICS_EFFECT_PCM_S8)
	{
		msgq = &haptics_pcm_msgq;
	}

	err = k_msgq_put(msgq, effect, K_NO_WAIT);
	if(err)
	{
		atomic_inc(&haptics_effects_queue_busy);
		atomic_set(&haptics_last_submit_err, err);
		return -EBUSY;
	}
	atomic_inc(&haptics_effects_submitted);
	atomic_set(&haptics_last_submit_err, 0);
	return 0;
}

static int haptics_submit_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_TONE,
		.channels = HAPTICS_CHANNELS_PRIMARY,
		.frequency_hz = safe_frequency(frequency_hz),
		.duration_ms = safe_duration(duration_ms),
	};

	if(frequency_hz == 0 || duration_ms == 0)
	{
		return 0;
	}
	return haptics_submit_effect(&command);
}

static int haptics_submit_pulse(void)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_BUTTON_PULSE,
		.channels = HAPTICS_CHANNELS_PRIMARY,
	};

	return haptics_submit_effect(&command);
}

static uint32_t haptics_approx_hypot(int32_t x, int32_t y)
{
	uint32_t ax = abs(x);
	uint32_t ay = abs(y);
	uint32_t high = MAX(ax, ay);
	uint32_t low = MIN(ax, ay);
	uint32_t value = 441U * low + 1007U * high;

	if(high < 16U * low)
	{
		value -= 40U * high;
	}
	return (value + 512U) >> 10;
}

void haptics_touchpad_update(bool right, bool pressure_active, bool touch_active, int16_t x,
                             int16_t y)
{
	struct haptics_touchpad_state *state = &haptics_touchpad_states[right ? 1 : 0];
	uint8_t channel = right ? HAPTICS_CHANNEL_RIGHT_0 : HAPTICS_CHANNEL_LEFT_0;
	int16_t enabled = 1;
	int16_t lizard_mode = 1;
	int16_t increment = 5500;
	int16_t click_pressure = 40;

	(void)ibex_setting_get(SETTING_HAPTICS_ENABLED, &enabled);
	(void)ibex_setting_get(SETTING_LIZARD_MODE, &lizard_mode);
	if(enabled != 1 || lizard_mode == 0)
	{
		state->touched = false;
		return;
	}

	(void)ibex_setting_get(SETTING_HAPTIC_INCREMENT, &increment);
	if(touch_active)
	{
		if(state->touched)
		{
			if(haptics_approx_hypot((int32_t)x - state->x, (int32_t)y - state->y) >
			   HAPTICS_LIZARD_MOVE_THRESHOLD)
			{
				state->x = x;
				state->y = y;
				state->move_count++;
			}
			if(HAPTICS_LIZARD_MOVE_THRESHOLD * state->move_count > (uint32_t)MAX(increment, 0))
			{
				struct haptics_effect effect = {
					.type = HAPTICS_EFFECT_TICK,
					.channels = channel,
					.gain_db = HAPTICS_LIZARD_TICK_GAIN_DB,
					.duration_ms = HAPTICS_TICK_DURATION_MS,
				};

				state->move_count = 0;
				(void)haptics_submit_effect(&effect);
			}
		}
		else
		{
			state->x = x;
			state->y = y;
			state->move_count = 0;
		}
	}
	state->touched = touch_active;

	(void)ibex_setting_get(right ? SETTING_RIGHT_TRACKPAD_CLICK_PRESSURE
	                             : SETTING_LEFT_TRACKPAD_CLICK_PRESSURE,
	                       &click_pressure);
	if(click_pressure != -1 && pressure_active != state->pressure_active)
	{
		struct haptics_effect effect = {
			.type = HAPTICS_EFFECT_CLICK,
			.channels = channel,
			.gain_db = HAPTICS_LIZARD_CLICK_GAIN_DB,
			.duration_ms = HAPTICS_CLICK_DURATION_MS,
		};

		(void)haptics_submit_effect(&effect);
	}
	state->pressure_active = pressure_active;
}

static int haptics_call_backend_effect(const struct haptics_effect *effect)
{
	int err = haptics_backend_effect(effect);

	atomic_inc(&haptics_backend_calls);
	atomic_set(&haptics_last_backend_err, err);
	if(err && err != -ENOTSUP)
	{
		atomic_inc(&haptics_backend_errors);
		LOG_DBG("haptics backend command failed: %d", err);
	}
	return err;
}

static int haptics_submit_pcm_config(uint8_t channels, uint8_t pcm_format)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_PCM_CONFIG,
		.channels = channels,
		.pcm_format = pcm_format,
	};

	return haptics_submit_effect(&command);
}

static int haptics_submit_pcm(uint8_t channels, const uint8_t *samples, size_t sample_count)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_PCM_S8,
		.channels = channels,
	};

	if(sample_count == 0)
	{
		return 0;
	}

	command.sample_count = MIN(sample_count, (size_t)HAPTICS_PCM_MAX_SAMPLES);
	memcpy(command.samples, samples, command.sample_count);
	return haptics_submit_effect(&command);
}

static k_timeout_t haptics_script_timeout(uint16_t delay_ms)
{
	return K_MSEC(delay_ms == 0 ? HAPTICS_SCRIPT_DELAY_ZERO_MS : delay_ms);
}

static int8_t haptics_script_gain(int8_t base_gain_db, const struct haptics_script_step *step)
{
	int gain_db = step->fixed_gain ? step->gain_db : base_gain_db + step->gain_db;

	return (int8_t)CLAMP(gain_db, INT8_MIN, INT8_MAX);
}

static void haptics_script_run_step(uint8_t channel, int8_t gain_db,
                                    const struct haptics_script_step *step)
{
	struct haptics_effect effect;
	uint8_t channels = step->channels == HAPTICS_SCRIPT_CURRENT_CHANNEL ? channel : step->channels;

	switch(step->type)
	{
		case HAPTICS_SCRIPT_STEP_NOOP:
			return;
		case HAPTICS_SCRIPT_STEP_PULSE:
			effect = (struct haptics_effect){
				.type = HAPTICS_EFFECT_PULSE_PERIODIC,
				.channels = channels,
				.duration_ms = safe_pulse_duration(step->on_us, step->repeat_count),
				.on_us = step->on_us,
				.off_us = step->off_us,
				.repeat_count = step->repeat_count,
			};
			(void)haptics_submit_effect(&effect);
			return;
		case HAPTICS_SCRIPT_STEP_TONE:
			effect = (struct haptics_effect){
				.type = HAPTICS_EFFECT_TONE,
				.channels = channels,
				.gain_db = haptics_script_gain(gain_db, step),
				.frequency_hz = safe_frequency(step->frequency_hz),
				.duration_ms = safe_duration(step->duration_ms),
			};
			(void)haptics_submit_effect(&effect);
			return;
		case HAPTICS_SCRIPT_STEP_LFO_TONE:
			effect = (struct haptics_effect){
				.type = HAPTICS_EFFECT_LFO_TONE,
				.channels = channels,
				.gain_db = haptics_script_gain(gain_db, step),
				.frequency_hz = safe_frequency(step->frequency_hz),
				.duration_ms = safe_duration(step->duration_ms),
				.lfo_frequency_hz = step->lfo_frequency_hz,
				.lfo_depth = step->lfo_depth,
			};
			(void)haptics_submit_effect(&effect);
			return;
		case HAPTICS_SCRIPT_STEP_LOG_SWEEP:
			effect = (struct haptics_effect){
				.type = HAPTICS_EFFECT_LOG_SWEEP,
				.channels = channels,
				.gain_db = haptics_script_gain(gain_db, step),
				.frequency_hz = safe_frequency(step->frequency_hz),
				.end_frequency_hz = safe_frequency(step->end_frequency_hz),
				.duration_ms = safe_duration(step->duration_ms),
				.invert_channels = step->invert_channels,
			};
			(void)haptics_submit_effect(&effect);
			return;
		case HAPTICS_SCRIPT_STEP_SCREAM:
			effect = (struct haptics_effect){
				.type = HAPTICS_EFFECT_SCREAM,
				.channels = channels,
				.gain_db = haptics_script_gain(gain_db, step),
			};
			(void)haptics_submit_effect(&effect);
			return;
		case HAPTICS_SCRIPT_STEP_END:
		default:
			return;
	}
}

static int haptics_start_script_on_channel(uint8_t channel, uint8_t script_id, int8_t gain_db)
{
	struct haptics_script_state *state = NULL;
	const struct haptics_script *script;
	uint16_t delay_ms;
	int err;

	if(script_id == 0 || script_id > ARRAY_SIZE(haptics_scripts))
	{
		LOG_DBG("unknown haptics script %u", script_id);
		return -EINVAL;
	}

	script = &haptics_scripts[script_id - 1];
	if(script->steps == NULL)
	{
		LOG_DBG("unknown haptics script %u", script_id);
		return -EINVAL;
	}

	for(size_t i = 0; i < ARRAY_SIZE(haptics_script_states); ++i)
	{
		if(haptics_script_states[i].channel == channel)
		{
			state = &haptics_script_states[i];
			break;
		}
	}
	if(state == NULL)
	{
		return 0;
	}

	k_mutex_lock(&haptics_script_mutex, K_FOREVER);
	if(state->active)
	{
		k_mutex_unlock(&haptics_script_mutex);
		LOG_DBG("haptics script already active on channel 0x%02x", channel);
		return 0;
	}

	state->script = script;
	state->index = 0;
	state->gain_db = gain_db;
	state->active = true;
	delay_ms = script->steps[0].delay_ms;
	k_mutex_unlock(&haptics_script_mutex);

	err = k_work_reschedule(&state->work, haptics_script_timeout(delay_ms));
	if(err < 0)
	{
		k_mutex_lock(&haptics_script_mutex, K_FOREVER);
		state->active = false;
		state->script = NULL;
		k_mutex_unlock(&haptics_script_mutex);
	}
	return err < 0 ? err : 0;
}

static int haptics_start_script(uint8_t channels, uint8_t script_id, int8_t gain_db)
{
	int first_err = 0;

	if(atomic_get(&haptics_ready) == 0 || atomic_get(&haptics_script_work_ready) == 0)
	{
		return -ENODEV;
	}

	for(size_t i = 0; i < ARRAY_SIZE(haptics_script_states); ++i)
	{
		int err;

		if((channels & haptics_script_states[i].channel) == 0)
		{
			continue;
		}

		err = haptics_start_script_on_channel(haptics_script_states[i].channel, script_id, gain_db);
		if(first_err == 0 && err)
		{
			first_err = err;
		}
	}

	return first_err;
}

static void haptics_script_work_handler(struct k_work *work)
{
	struct haptics_script_state *state = CONTAINER_OF(work, struct haptics_script_state, work.work);
	const struct haptics_script_step *step;
	uint8_t channel;
	int8_t gain_db;
	uint16_t delay_ms;

	k_mutex_lock(&haptics_script_mutex, K_FOREVER);
	if(!state->active || state->script == NULL)
	{
		k_mutex_unlock(&haptics_script_mutex);
		return;
	}

	step = &state->script->steps[state->index];
	if(step->type == HAPTICS_SCRIPT_STEP_END)
	{
		state->active = false;
		state->script = NULL;
		k_mutex_unlock(&haptics_script_mutex);
		return;
	}

	channel = state->channel;
	gain_db = state->gain_db;
	++state->index;
	delay_ms = state->script->steps[state->index].delay_ms;
	k_mutex_unlock(&haptics_script_mutex);

	haptics_script_run_step(channel, gain_db, step);
	(void)k_work_reschedule(&state->work, haptics_script_timeout(delay_ms));
}

static void haptics_thread_main(void *arg1, void *arg2, void *arg3)
{
	struct k_msgq *msgq = arg1;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for(;;)
	{
		struct haptics_effect command;

		k_msgq_get(msgq, &command, K_FOREVER);
		(void)haptics_call_backend_effect(&command);
	}
}

static const char *haptics_report_name(uint8_t report_id)
{
	switch(report_id)
	{
		case ID_OUT_REPORT_HAPTIC_RUMBLE:
			return "rumble";
		case ID_OUT_REPORT_HAPTIC_PULSE:
			return "pulse";
		case ID_OUT_REPORT_HAPTIC_COMMAND:
			return "command";
		case ID_OUT_REPORT_HAPTIC_LFO_TONE:
			return "lfo_tone";
		case ID_OUT_REPORT_HAPTIC_LOG_SWEEP:
			return "log_sweep";
		case ID_OUT_REPORT_HAPTIC_SCRIPT:
			return "script";
		case VALVE_HAPTIC_REPORT_CLASS:
			return "class";
		case VALVE_HAPTIC_REPORT_PCM_MONO:
			return "pcm_mono";
		case VALVE_HAPTIC_REPORT_PCM_STEREO:
			return "pcm_stereo";
		case VALVE_HAPTIC_REPORT_PCM_MONO_WITH_LENGTH:
			return "pcm_mono_len";
		case VALVE_HAPTIC_REPORT_GYRO_BIAS:
			return "gyro_bias";
		default:
			return "?";
	}
}

static const char *haptics_command_name(uint8_t command)
{
	switch(command)
	{
		case HAPTIC_TYPE_OFF:
			return "off";
		case HAPTIC_TYPE_TICK:
			return "tick";
		case HAPTIC_TYPE_CLICK:
			return "click";
		case HAPTIC_TYPE_TONE:
			return "tone";
		case HAPTIC_TYPE_RUMBLE:
			return "rumble";
		case HAPTIC_TYPE_NOISE:
			return "noise";
		case HAPTIC_TYPE_SCRIPT:
			return "script";
		case HAPTIC_TYPE_LOG_SWEEP:
			return "log_sweep";
		default:
			return "?";
	}
}

static bool haptics_take_report_log_slot(void)
{
	atomic_val_t old = atomic_inc(&haptics_report_log_count);

	if(old == HAPTICS_REPORT_LOG_LIMIT)
	{
		LOG_INF("haptics output logging suppressed after %u reports", HAPTICS_REPORT_LOG_LIMIT);
	}
	return old < HAPTICS_REPORT_LOG_LIMIT;
}

static void haptics_log_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	LOG_INF("haptics output 0x%02x %s len=%u", report_id, haptics_report_name(report_id), len);
	if(len > 0)
	{
		LOG_HEXDUMP_INF(data, MIN(len, (size_t)HAPTICS_REPORT_HEXDUMP_BYTES), "haptics payload");
	}
}

static int handle_haptic_rumble(const uint8_t *data, size_t len, bool log)
{
	struct haptics_effect command;
	uint16_t intensity;
	uint16_t left_speed;
	uint16_t right_speed;
	int base_gain_db;
	int first_err = 0;
	int err;

	if(len < HAPTICS_REPORT_PAYLOAD_BYTES(HID_RUMBLE_OUTPUT_REPORT_BYTES))
	{
		return -EMSGSIZE;
	}

	intensity = sys_get_le16(&data[1]);
	left_speed = sys_get_le16(&data[3]);
	right_speed = sys_get_le16(&data[6]);
	base_gain_db = rumble_base_gain_db(intensity);
	if(log)
	{
		LOG_INF("haptics rumble type=%u intensity=%u left_speed=%u left_gain=%d "
		        "right_speed=%u right_gain=%d",
		        data[0], intensity, left_speed, (int8_t)data[5], right_speed, (int8_t)data[8]);
	}

	if(right_speed == 0)
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_STOP,
			.channels = HAPTICS_CHANNEL_RIGHT_1,
		};
	}
	else
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_RANDOM_LFO,
			.channels = HAPTICS_CHANNEL_RIGHT_1,
			.gain_db = clamp_gain_db((int8_t)data[8] + base_gain_db),
			.frequency_hz = HAPTICS_RUMBLE_CARRIER_FREQUENCY_HZ,
			.duration_ms = HAPTICS_RUMBLE_DURATION_MS,
			.lfo_frequency_hz =
			    rumble_modulation_rate_hz(right_speed, HAPTICS_RUMBLE_RIGHT_RANDOM_RATE_MIN_HZ,
			                              HAPTICS_RUMBLE_RIGHT_RANDOM_RATE_RANGE_HZ),
		};
	}
	err = haptics_submit_effect(&command);
	if(err)
	{
		first_err = err;
	}

	if(left_speed == 0)
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_STOP,
			.channels = HAPTICS_CHANNEL_LEFT_1,
		};
	}
	else
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_LFO_TONE,
			.channels = HAPTICS_CHANNEL_LEFT_1,
			.gain_db =
			    clamp_gain_db((int8_t)data[5] + HAPTICS_RUMBLE_LEFT_GAIN_OFFSET_DB + base_gain_db),
			.frequency_hz = HAPTICS_RUMBLE_CARRIER_FREQUENCY_HZ,
			.duration_ms = HAPTICS_RUMBLE_DURATION_MS,
			.lfo_frequency_hz =
			    rumble_modulation_rate_hz(left_speed, HAPTICS_RUMBLE_LEFT_LFO_RATE_MIN_HZ,
			                              HAPTICS_RUMBLE_LEFT_LFO_RATE_RANGE_HZ),
			.lfo_depth = HAPTICS_RUMBLE_LFO_DEPTH,
		};
	}
	err = haptics_submit_effect(&command);
	if(first_err == 0 && err)
	{
		first_err = err;
	}

	return first_err;
}

static int handle_haptic_pulse(const uint8_t *data, size_t len, bool log)
{
	struct haptics_effect command;
	uint8_t channels;
	uint32_t on_us;
	uint32_t off_us;
	uint32_t repeat_count;

	if(len < HAPTICS_REPORT_PAYLOAD_BYTES(HID_HAPTIC_PULSE_OUTPUT_REPORT_BYTES))
	{
		return -EMSGSIZE;
	}

	channels = haptics_pulse_channels(data[0]);
	on_us = sys_get_le16(&data[1]);
	off_us = sys_get_le16(&data[3]);
	repeat_count = sys_get_le16(&data[5]);
	if(log)
	{
		LOG_INF("haptics pulse side=0x%02x channels=%s on_us=%u off_us=%u repeats=%u", data[0],
		        haptics_channels_name(channels), on_us, off_us, repeat_count);
	}
	if(on_us == 0)
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_STOP,
			.channels = channels,
		};
		return haptics_submit_effect(&command);
	}
	if(off_us == 0)
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_PULSE_ONE_SHOT,
			.channels = channels,
			.gain_db = HAPTICS_PULSE_ONE_SHOT_GAIN_DB,
			.frequency_hz = HAPTICS_DEFAULT_PULSE_FREQUENCY_HZ,
			.duration_ms = HAPTICS_TICK_DURATION_MS,
			.on_us = on_us,
			.repeat_count = repeat_count,
		};
		return haptics_submit_effect(&command);
	}
	if(repeat_count == 3 || repeat_count == 4)
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_PULSE_CLICK,
			.channels = channels,
			.gain_db = HAPTICS_PULSE_CLICK_GAIN_DB,
			.frequency_hz = HAPTICS_CLICK_FREQUENCY_HZ,
			.duration_ms = HAPTICS_CLICK_DURATION_MS,
			.on_us = on_us,
			.off_us = off_us,
			.repeat_count = repeat_count,
		};
		return haptics_submit_effect(&command);
	}
	if(repeat_count == 0)
	{
		return 0;
	}

	command = (struct haptics_effect){
		.type = HAPTICS_EFFECT_PULSE_PERIODIC,
		.channels = channels,
		.duration_ms = safe_pulse_duration(on_us, repeat_count),
		.on_us = on_us,
		.off_us = off_us,
		.repeat_count = repeat_count,
	};
	return haptics_submit_effect(&command);
}

static int handle_haptic_command(const uint8_t *data, size_t len, bool log)
{
	struct haptics_effect command;
	uint8_t channels;

	if(len < HAPTICS_REPORT_PAYLOAD_BYTES(HID_HAPTIC_COMMAND_REPORT_BYTES))
	{
		return -EMSGSIZE;
	}

	channels = haptics_command_channels(data[0]);
	if(log)
	{
		LOG_INF("haptics command side=0x%02x channels=%s command=%u %s gain_db=%d", data[0],
		        haptics_channels_name(channels), data[1], haptics_command_name(data[1]),
		        (int8_t)data[2]);
	}
	switch(data[1])
	{
		case HAPTIC_TYPE_OFF:
			command = (struct haptics_effect){
				.type = HAPTICS_EFFECT_STOP,
				.channels = channels,
			};
			return haptics_submit_effect(&command);
		case HAPTIC_TYPE_TICK:
			command = (struct haptics_effect){
				.type = HAPTICS_EFFECT_TICK,
				.channels = channels,
				.gain_db = (int8_t)data[2],
				.frequency_hz = HAPTICS_DEFAULT_PULSE_FREQUENCY_HZ,
				.duration_ms = HAPTICS_TICK_DURATION_MS,
			};
			return haptics_submit_effect(&command);
		case HAPTIC_TYPE_CLICK:
			command = (struct haptics_effect){
				.type = HAPTICS_EFFECT_CLICK,
				.channels = channels,
				.gain_db = (int8_t)data[2],
				.frequency_hz = HAPTICS_CLICK_FREQUENCY_HZ,
				.duration_ms = HAPTICS_CLICK_DURATION_MS,
			};
			return haptics_submit_effect(&command);
		default:
			return -EINVAL;
	}
}

static int handle_haptic_lfo_tone(const uint8_t *data, size_t len, bool log)
{
	struct haptics_effect command;
	uint8_t channels;
	uint16_t frequency_hz;
	uint16_t duration_ms;

	if(len < HAPTICS_REPORT_PAYLOAD_BYTES(HID_HAPTIC_LFO_TONE_REPORT_BYTES))
	{
		return -EMSGSIZE;
	}

	channels = haptics_command_channels(data[0]);
	frequency_hz = sys_get_le16(&data[2]);
	duration_ms = sys_get_le16(&data[4]);
	if(log)
	{
		LOG_INF("haptics lfo_tone side=0x%02x channels=%s gain_db=%d freq=%u duration_ms=%u "
		        "lfo_freq=%u lfo_depth=%u",
		        data[0], haptics_channels_name(channels), (int8_t)data[1], frequency_hz,
		        duration_ms, sys_get_le16(&data[6]), data[8]);
	}
	command = (struct haptics_effect){
		.type = HAPTICS_EFFECT_LFO_TONE,
		.channels = channels,
		.gain_db = (int8_t)data[1],
		.frequency_hz = safe_frequency(frequency_hz),
		.duration_ms = duration_ms,
		.lfo_frequency_hz = sys_get_le16(&data[6]),
		.lfo_depth = data[8],
	};
	return haptics_submit_effect(&command);
}

static int handle_haptic_log_sweep(const uint8_t *data, size_t len, bool log)
{
	struct haptics_effect command;
	uint8_t channels;
	uint16_t duration_ms;
	uint16_t start_frequency_hz;
	uint16_t end_frequency_hz;
	uint8_t invert_channels = 0;

	if(len < HAPTICS_REPORT_PAYLOAD_BYTES(HID_HAPTIC_LOG_SWEEP_REPORT_BYTES))
	{
		return -EMSGSIZE;
	}

	channels = haptics_command_channels(data[0]);
	duration_ms = sys_get_le16(&data[2]);
	start_frequency_hz = sys_get_le16(&data[4]);
	end_frequency_hz = sys_get_le16(&data[6]);
	if((data[0] & 0x7f) == 6)
	{
		invert_channels = HAPTICS_CHANNEL_RIGHT_0;
	}
	else if((data[0] & 0x7f) == 7)
	{
		invert_channels = HAPTICS_CHANNEL_RIGHT_1;
	}
	if(log)
	{
		LOG_INF("haptics log_sweep side=0x%02x channels=%s gain_db=%d duration_ms=%u "
		        "start_freq=%u end_freq=%u",
		        data[0], haptics_channels_name(channels), (int8_t)data[1], duration_ms,
		        start_frequency_hz, end_frequency_hz);
	}
	command = (struct haptics_effect){
		.type = HAPTICS_EFFECT_LOG_SWEEP,
		.channels = channels,
		.gain_db = (int8_t)data[1],
		.frequency_hz = safe_frequency(start_frequency_hz),
		.end_frequency_hz = safe_frequency(end_frequency_hz),
		.duration_ms = duration_ms,
		.invert_channels = invert_channels,
	};
	return haptics_submit_effect(&command);
}

static int handle_haptic_script(const uint8_t *data, size_t len, bool log)
{
	uint8_t channels;

	if(len < HAPTICS_REPORT_PAYLOAD_BYTES(HID_HAPTIC_SCRIPT_REPORT_BYTES))
	{
		return -EMSGSIZE;
	}

	channels = haptics_command_channels(data[0]);
	if(log)
	{
		LOG_INF("haptics script side=0x%02x channels=%s script_id=%u gain_db=%d", data[0],
		        haptics_channels_name(channels), data[1], (int8_t)data[2]);
	}
	(void)haptics_start_script(channels, data[1], (int8_t)data[2]);
	return 0;
}

static const char *haptics_stream_channel_name(uint8_t channel)
{
	switch(channel)
	{
		case 0:
			return "left0";
		case 1:
			return "right0";
		case 2:
			return "left1";
		case 3:
			return "right1";
		default:
			return "?";
	}
}

static uint8_t haptics_stream_channel_mask(uint8_t channel)
{
	switch(channel)
	{
		case 0:
			return HAPTICS_CHANNEL_LEFT_0;
		case 1:
			return HAPTICS_CHANNEL_RIGHT_0;
		case 2:
			return HAPTICS_CHANNEL_LEFT_1;
		case 3:
			return HAPTICS_CHANNEL_RIGHT_1;
		default:
			return 0;
	}
}

static const char *haptics_pcm_route_name(uint8_t selector)
{
	switch(selector)
	{
		case 0:
			return "left1";
		case 2:
		case 0x80:
			return "secondary";
		case 3:
			return "left0";
		case 4:
			return "right1";
		case 5:
			return "primary";
		default:
			return "none";
	}
}

static uint8_t haptics_pcm_route_channels(uint8_t selector)
{
	switch(selector)
	{
		case 0:
			return HAPTICS_CHANNEL_LEFT_1;
		case 2:
		case 0x80:
			return HAPTICS_CHANNEL_LEFT_1 | HAPTICS_CHANNEL_RIGHT_1;
		case 3:
			return HAPTICS_CHANNEL_LEFT_0;
		case 4:
			return HAPTICS_CHANNEL_RIGHT_1;
		case 5:
			return HAPTICS_CHANNEL_LEFT_0 | HAPTICS_CHANNEL_RIGHT_0;
		default:
			return 0;
	}
}

static int handle_haptic_stream_op(uint8_t channel, const uint8_t *data, size_t len, bool log)
{
	struct haptics_effect command;

	if(len == 0)
	{
		return -EINVAL;
	}
	if(data[0] == 2 && len < 3)
	{
		return -EINVAL;
	}
	if(log)
	{
		LOG_INF("haptics stream channel=%s op=%u arg=%u", haptics_stream_channel_name(channel),
		        data[0], len > 2 ? data[2] : 0);
	}
	if(data[0] == 1)
	{
		command = (struct haptics_effect){
			.type = HAPTICS_EFFECT_STOP_PCM,
			.channels = haptics_stream_channel_mask(channel),
		};
		return haptics_submit_effect(&command);
	}
	if(data[0] == 2)
	{
		return haptics_submit_pcm_config(haptics_stream_channel_mask(channel), data[2]);
	}
	return 0;
}

static int handle_haptic_class(const uint8_t *data, size_t len, bool log)
{
	if(len < 2)
	{
		return -EMSGSIZE;
	}

	switch(data[1])
	{
		case 1:
		case 0x40:
			return handle_haptic_stream_op(3, data, len, log);
		case 2:
		case 0x80:
			(void)handle_haptic_stream_op(3, data, len, log);
			return handle_haptic_stream_op(2, data, len, log);
		case 3:
			return handle_haptic_stream_op(0, data, len, log);
		case 4:
			return handle_haptic_stream_op(1, data, len, log);
		case 5:
			(void)handle_haptic_stream_op(1, data, len, log);
			return handle_haptic_stream_op(0, data, len, log);
		default:
			return data[1] == 0 ? handle_haptic_stream_op(2, data, len, log) : 0;
	}
}

static int handle_haptic_pcm_route(const uint8_t *data, size_t len, bool log)
{
	uint8_t channels;

	if(len == 0)
	{
		return -EMSGSIZE;
	}
	channels = haptics_pcm_route_channels(data[0]);
	if(log)
	{
		LOG_INF("haptics pcm_route selector=0x%02x channels=%s samples=%u", data[0],
		        haptics_pcm_route_name(data[0]), len - 1);
	}
	if(len == 1 || channels == 0)
	{
		return 0;
	}
	return haptics_submit_pcm(channels, &data[1], len - 1);
}

static int handle_haptic_pcm_mono_with_length(const uint8_t *data, size_t len, bool log)
{
	uint8_t payload_len;

	if(len == 0)
	{
		return -EINVAL;
	}

	payload_len = data[0];
	if(payload_len + 2U > len)
	{
		return -EINVAL;
	}
	return handle_haptic_pcm_route(&data[1], payload_len + 1U, log);
}

static int handle_haptic_pcm_stereo(const uint8_t *data, size_t len, bool log)
{
	size_t expected_len;
	uint8_t sample_count;
	int first_err = 0;
	int err;

	if(len == 0)
	{
		return -EMSGSIZE;
	}
	sample_count = data[0];
	if(log)
	{
		LOG_INF("haptics pcm_stereo samples=%u", sample_count);
	}
	if(sample_count > VALVE_HAPTIC_PCM_STEREO_SAMPLES)
	{
		return -EINVAL;
	}

	expected_len = 1U + (size_t)data[0] * 2U;
	if(len < expected_len)
	{
		return -EMSGSIZE;
	}
	if((size_t)sample_count + 32U > len)
	{
		return -EMSGSIZE;
	}

	err = haptics_submit_pcm(HAPTICS_CHANNEL_RIGHT_1, &data[32], sample_count);
	if(err)
	{
		first_err = err;
	}
	err = haptics_submit_pcm(HAPTICS_CHANNEL_LEFT_1, &data[1], sample_count);
	if(first_err == 0 && err)
	{
		first_err = err;
	}
	return first_err;
}

int haptics_handle_output_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	bool log;

	if(data == NULL)
	{
		return -EINVAL;
	}

	if(report_id == 0 && len > 0)
	{
		report_id = *data++;
		len--;
	}
	atomic_inc(&haptics_reports_seen);
	atomic_set(&haptics_last_report_id, report_id);
	log = haptics_take_report_log_slot();
	if(log)
	{
		haptics_log_report(report_id, data, len);
	}

	switch(report_id)
	{
		case ID_OUT_REPORT_HAPTIC_RUMBLE:
			return handle_haptic_rumble(data, len, log);
		case ID_OUT_REPORT_HAPTIC_PULSE:
			return handle_haptic_pulse(data, len, log);
		case ID_OUT_REPORT_HAPTIC_COMMAND:
			return handle_haptic_command(data, len, log);
		case ID_OUT_REPORT_HAPTIC_LFO_TONE:
			return handle_haptic_lfo_tone(data, len, log);
		case ID_OUT_REPORT_HAPTIC_LOG_SWEEP:
			return handle_haptic_log_sweep(data, len, log);
		case ID_OUT_REPORT_HAPTIC_SCRIPT:
			return handle_haptic_script(data, len, log);
		case VALVE_HAPTIC_REPORT_CLASS:
			return handle_haptic_class(data, len, log);
		case VALVE_HAPTIC_REPORT_PCM_MONO:
			return handle_haptic_pcm_route(data, len, log);
		case VALVE_HAPTIC_REPORT_PCM_STEREO:
			return handle_haptic_pcm_stereo(data, len, log);
		case VALVE_HAPTIC_REPORT_PCM_MONO_WITH_LENGTH:
			return handle_haptic_pcm_mono_with_length(data, len, log);
		case VALVE_HAPTIC_REPORT_GYRO_BIAS:
			return 0;
		default:
			return -ENOTSUP;
	}
}

void hardware_haptic_pulse(void)
{
	if(IS_ENABLED(CONFIG_IBEX_HAPTICS_PWM))
	{
		return;
	}
	(void)haptics_submit_pulse();
}

int hardware_haptic_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
	return haptics_submit_tone(frequency_hz, duration_ms);
}

void haptics_get_debug(struct haptics_debug *debug)
{
	if(debug == NULL)
	{
		return;
	}

	*debug = (struct haptics_debug){
		.ready = atomic_get(&haptics_ready) != 0,
		.enabled = haptics_local_enabled(),
		.last_report_id = (uint8_t)atomic_get(&haptics_last_report_id),
		.last_effect_type = (uint8_t)atomic_get(&haptics_last_effect_type),
		.last_effect_channels = (uint8_t)atomic_get(&haptics_last_effect_channels),
		.reports_seen = (uint32_t)atomic_get(&haptics_reports_seen),
		.effects_submitted = (uint32_t)atomic_get(&haptics_effects_submitted),
		.effects_disabled = (uint32_t)atomic_get(&haptics_effects_disabled),
		.effects_no_channel = (uint32_t)atomic_get(&haptics_effects_no_channel),
		.effects_queue_busy = (uint32_t)atomic_get(&haptics_effects_queue_busy),
		.backend_calls = (uint32_t)atomic_get(&haptics_backend_calls),
		.backend_errors = (uint32_t)atomic_get(&haptics_backend_errors),
		.last_submit_err = (int)atomic_get(&haptics_last_submit_err),
		.last_backend_err = (int)atomic_get(&haptics_last_backend_err),
	};
}

int haptics_debug_play_tick(uint8_t channels)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_TICK,
		.channels = channels,
		.frequency_hz = HAPTICS_DEFAULT_PULSE_FREQUENCY_HZ,
		.duration_ms = HAPTICS_TICK_DURATION_MS,
	};

	return haptics_submit_effect(&command);
}

int haptics_debug_play_click(uint8_t channels)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_CLICK,
		.channels = channels,
		.frequency_hz = HAPTICS_CLICK_FREQUENCY_HZ,
		.duration_ms = HAPTICS_CLICK_DURATION_MS,
	};

	return haptics_submit_effect(&command);
}

int haptics_debug_play_pulse(uint8_t channels)
{
	struct haptics_effect command = {
		.type = HAPTICS_EFFECT_BUTTON_PULSE,
		.channels = channels,
	};

	return haptics_submit_effect(&command);
}
