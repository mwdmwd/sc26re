/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HAPTICS_PCM_MAX_SAMPLES 62
#define HAPTICS_PCM_STEREO_MAX_SAMPLES (HAPTICS_PCM_MAX_SAMPLES / 2)

#define HAPTICS_CHANNEL_LEFT_0 (1U << 0)
#define HAPTICS_CHANNEL_RIGHT_0 (1U << 1)
#define HAPTICS_CHANNEL_LEFT_1 (1U << 2)
#define HAPTICS_CHANNEL_RIGHT_1 (1U << 3)
#define HAPTICS_CHANNELS_PRIMARY (HAPTICS_CHANNEL_LEFT_0 | HAPTICS_CHANNEL_RIGHT_0)
#define HAPTICS_CHANNELS_ALL \
	(HAPTICS_CHANNEL_LEFT_0 | HAPTICS_CHANNEL_RIGHT_0 | HAPTICS_CHANNEL_LEFT_1 | HAPTICS_CHANNEL_RIGHT_1)

enum haptics_effect_type
{
	HAPTICS_EFFECT_STOP,
	HAPTICS_EFFECT_STOP_PCM,
	HAPTICS_EFFECT_BUTTON_PULSE,
	HAPTICS_EFFECT_TICK,
	HAPTICS_EFFECT_CLICK,
	HAPTICS_EFFECT_TONE,
	HAPTICS_EFFECT_PULSE_ONE_SHOT,
	HAPTICS_EFFECT_PULSE_CLICK,
	HAPTICS_EFFECT_PULSE_PERIODIC,
	HAPTICS_EFFECT_LFO_TONE,
	HAPTICS_EFFECT_RANDOM_LFO,
	HAPTICS_EFFECT_LOG_SWEEP,
	HAPTICS_EFFECT_PCM_CONFIG,
	HAPTICS_EFFECT_PCM_S8,
	HAPTICS_EFFECT_PCM_STEREO,
	HAPTICS_EFFECT_SCREAM,
};

struct haptics_effect
{
	enum haptics_effect_type type;
	uint8_t channels;
	int8_t gain_db;
	uint16_t frequency_hz;
	uint16_t end_frequency_hz;
	uint16_t duration_ms;
	uint16_t on_us;
	uint16_t off_us;
	uint16_t repeat_count;
	float lfo_frequency_hz;
	uint8_t lfo_depth;
	uint8_t invert_channels;
	uint8_t pcm_format;
	uint8_t sample_count;
	uint8_t samples[HAPTICS_PCM_MAX_SAMPLES];
};

struct haptics_debug
{
	uint8_t ready;
	uint8_t enabled;
	uint8_t last_report_id;
	uint8_t last_effect_type;
	uint8_t last_effect_channels;
	uint32_t reports_seen;
	uint32_t effects_submitted;
	uint32_t effects_disabled;
	uint32_t effects_no_channel;
	uint32_t effects_queue_busy;
	uint32_t backend_calls;
	uint32_t backend_errors;
	int last_submit_err;
	int last_backend_err;
};

struct haptics_backend_debug
{
	uint32_t play_requests;
	uint32_t play_started;
	uint32_t play_suppressed;
	uint8_t last_requested_channels;
	uint8_t last_physical_channels;
	uint8_t last_play_channels;
	uint32_t last_sample_count;
	int last_play_err;
};

int haptics_init(void);
int haptics_handle_output_report(uint8_t report_id, const uint8_t *data, size_t len);
void haptics_get_debug(struct haptics_debug *debug);
int haptics_debug_play_tick(uint8_t channels);
int haptics_debug_play_click(uint8_t channels);
int haptics_debug_play_pulse(uint8_t channels);
void haptics_touchpad_update(bool right, bool pressure_active, bool touch_active, int16_t x,
                             int16_t y);

int haptics_backend_init(void);
int haptics_backend_effect(const struct haptics_effect *effect);
int haptics_backend_get_debug(struct haptics_backend_debug *debug);
int haptics_backend_set_master_gain_db(int16_t gain_db);
int haptics_backend_set_amplifier_mode(bool forced_on);
int haptics_backend_pulse(void);
int haptics_backend_tone(uint32_t frequency_hz, uint32_t duration_ms);
