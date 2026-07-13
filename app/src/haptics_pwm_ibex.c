/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <hal/nrf_gpio.h>
#include <nrfx_pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "haptics.h"
#include "haptics_waveforms.h"

LOG_MODULE_DECLARE(haptics);

#define IBEX_HAPTICS_NODE DT_NODELABEL(ibex_haptics)
#define IBEX_HAPTICS_I2S_NODE DT_NODELABEL(i2s0)

#define IBEX_HAPTICS_SUPPORTED_CHANNELS HAPTICS_CHANNELS_ALL

/* OFW exposes the touchpad sequencer as 16-bit stereo at 4 kHz backed by PWM0. */
#define IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ 4000U
#define IBEX_HAPTICS_PRIMARY_BLOCK_MS 8U
#define IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES \
	((IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ * IBEX_HAPTICS_PRIMARY_BLOCK_MS) / 1000U)
#define IBEX_HAPTICS_PRIMARY_START_SILENCE_SAMPLES 16U
#define IBEX_HAPTICS_PRIMARY_BUFFER_COUNT 2U
#define IBEX_HAPTICS_PWM_INSTANCE_ID 0
#define IBEX_HAPTICS_PWM_CLOCK_HZ 16000000U
#define IBEX_HAPTICS_TOP_VALUE 256U
#define IBEX_HAPTICS_PWM_SAMPLE_REPEATS 15U

/* OFW haptics-sequencer-gri-v3 uses the real nRF I2S peripheral at 8 kHz. */
#define IBEX_HAPTICS_SECONDARY_SAMPLE_RATE_HZ 8000U
#define IBEX_HAPTICS_SECONDARY_BLOCK_MS IBEX_HAPTICS_PRIMARY_BLOCK_MS
#define IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES \
	((IBEX_HAPTICS_SECONDARY_SAMPLE_RATE_HZ * IBEX_HAPTICS_SECONDARY_BLOCK_MS) / 1000U)
#define IBEX_HAPTICS_SECONDARY_START_SILENCE_FRAMES 32U
#define IBEX_HAPTICS_SECONDARY_FRAME_BYTES (2U * sizeof(int16_t))
#define IBEX_HAPTICS_SECONDARY_BLOCK_BYTES \
	(IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES * IBEX_HAPTICS_SECONDARY_FRAME_BYTES)
#define IBEX_HAPTICS_I2S_BLOCK_COUNT 2U
#define IBEX_HAPTICS_I2S_START_BLOCKS 2U

#define IBEX_HAPTICS_THREAD_STACK_SIZE 2048
#define IBEX_HAPTICS_THREAD_PRIORITY 8
#define IBEX_HAPTICS_DEFAULT_FREQUENCY_HZ 882U
#define IBEX_HAPTICS_GAIN_DB_MIN (-24)
#define IBEX_HAPTICS_GAIN_DB_MAX 24
#define IBEX_HAPTICS_GAIN_Q15_ONE 32768.0f
#define IBEX_HAPTICS_TWO_PI 6.2831853071795864769f
#define IBEX_HAPTICS_NEG_PI_4 (-0.7853981633974483f)
#define IBEX_HAPTICS_LOG_SWEEP_ATTACK_S 0.08f
#define IBEX_HAPTICS_LOG_SWEEP_RELEASE_S 0.005f
#define IBEX_HAPTICS_PCM_RING_BYTES 512U
#define IBEX_HAPTICS_EFFECT_SLOT_COUNT 8U

#define GPIO_ABS_PIN(prop) \
	NRF_GPIO_PIN_MAP(DT_PROP(DT_GPIO_CTLR(IBEX_HAPTICS_NODE, prop), port), \
	                 DT_GPIO_PIN(IBEX_HAPTICS_NODE, prop))

#define IBEX_HAPTICS_LEFT_PIN GPIO_ABS_PIN(left_gpios)
#define IBEX_HAPTICS_RIGHT_PIN GPIO_ABS_PIN(right_gpios)
#define IBEX_HAPTICS_LEFT_CHANNEL DT_PROP(IBEX_HAPTICS_NODE, left_channel)
#define IBEX_HAPTICS_RIGHT_CHANNEL DT_PROP(IBEX_HAPTICS_NODE, right_channel)

BUILD_ASSERT(IBEX_HAPTICS_TOP_VALUE <= 0x7fff);
BUILD_ASSERT(IBEX_HAPTICS_LEFT_CHANNEL < NRF_PWM_CHANNEL_COUNT);
BUILD_ASSERT(IBEX_HAPTICS_RIGHT_CHANNEL < NRF_PWM_CHANNEL_COUNT);
BUILD_ASSERT(IBEX_HAPTICS_LEFT_CHANNEL != IBEX_HAPTICS_RIGHT_CHANNEL);
BUILD_ASSERT(IBEX_HAPTICS_LEFT_CHANNEL / 2U != IBEX_HAPTICS_RIGHT_CHANNEL / 2U);
BUILD_ASSERT(IBEX_HAPTICS_PRIMARY_START_SILENCE_SAMPLES < IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES);
BUILD_ASSERT(IBEX_HAPTICS_SECONDARY_START_SILENCE_FRAMES < IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES);
BUILD_ASSERT(IBEX_HAPTICS_I2S_BLOCK_COUNT == IBEX_HAPTICS_I2S_START_BLOCKS);
BUILD_ASSERT(IBEX_HAPTICS_PCM_RING_BYTES >
             2U * IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES * sizeof(int16_t));
BUILD_ASSERT(HAPTICS_WAVEFORM_TICK_SAMPLE_RATE_HZ == IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ);
BUILD_ASSERT(HAPTICS_WAVEFORM_CLICK_SAMPLE_RATE_HZ == IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ);
BUILD_ASSERT(HAPTICS_WAVEFORM_TICK_SAMPLE_FORMAT == HAPTICS_WAVEFORM_SAMPLE_FORMAT_S8);
BUILD_ASSERT(HAPTICS_WAVEFORM_CLICK_SAMPLE_FORMAT == HAPTICS_WAVEFORM_SAMPLE_FORMAT_S8);
BUILD_ASSERT(HAPTICS_WAVEFORM_SCREAM_SAMPLE_RATE_HZ == IBEX_HAPTICS_SECONDARY_SAMPLE_RATE_HZ);
BUILD_ASSERT(HAPTICS_WAVEFORM_SCREAM_SAMPLE_FORMAT == HAPTICS_WAVEFORM_SAMPLE_FORMAT_S16);
BUILD_ASSERT(HAPTICS_WAVEFORM_TICK_SAMPLES == ARRAY_SIZE(haptics_waveform_tick));
BUILD_ASSERT(HAPTICS_WAVEFORM_CLICK_SAMPLES == ARRAY_SIZE(haptics_waveform_click));
BUILD_ASSERT(HAPTICS_WAVEFORM_SCREAM_SAMPLES == ARRAY_SIZE(haptics_waveform_scream));

enum ibex_haptics_channel_index
{
	IBEX_HAPTICS_INDEX_LEFT_0,
	IBEX_HAPTICS_INDEX_RIGHT_0,
	IBEX_HAPTICS_INDEX_LEFT_1,
	IBEX_HAPTICS_INDEX_RIGHT_1,
	IBEX_HAPTICS_CHANNEL_COUNT,
};

enum ibex_haptics_effect_kind
{
	IBEX_HAPTICS_EFFECT_IDLE,
	IBEX_HAPTICS_EFFECT_WAVEFORM,
	IBEX_HAPTICS_EFFECT_SINE,
	IBEX_HAPTICS_EFFECT_PULSE,
	IBEX_HAPTICS_EFFECT_LOG_SWEEP,
	IBEX_HAPTICS_EFFECT_RANDOM_LFO,
	IBEX_HAPTICS_EFFECT_PCM,
};

enum ibex_haptics_effect_slot
{
	IBEX_HAPTICS_SLOT_TICK,
	IBEX_HAPTICS_SLOT_CLICK,
	IBEX_HAPTICS_SLOT_PULSE,
	IBEX_HAPTICS_SLOT_LFO_TONE,
	IBEX_HAPTICS_SLOT_RANDOM_LFO,
	IBEX_HAPTICS_SLOT_LOG_SWEEP,
	IBEX_HAPTICS_SLOT_PCM,
	IBEX_HAPTICS_SLOT_SCREAM,
};

struct ibex_haptics_channel_state
{
	enum ibex_haptics_effect_kind kind;
	float gain;
	uint32_t sample_index;
	uint32_t total_samples;
	bool invert;
	const void *waveform;
	uint32_t waveform_len;
	uint32_t waveform_rate_hz;
	uint32_t waveform_output_rate_hz;
	uint32_t waveform_source_index;
	uint32_t waveform_phase;
	uint8_t waveform_format;
	float phase;
	float phase_step;
	float lfo_phase;
	float lfo_phase_step;
	float lfo_depth;
	uint32_t period_samples;
	uint32_t high_samples;
	uint32_t repeat_count;
	uint32_t random_period_samples;
	uint32_t random_period_index;
	uint16_t carrier_cycles;
	float random_gain;
	float log_ratio;
	float phase_scale;
	uint32_t attack_samples;
	uint32_t release_start_sample;
	uint8_t pcm[IBEX_HAPTICS_PCM_RING_BYTES];
	uint16_t pcm_head;
	uint16_t pcm_tail;
	uint16_t pcm_count;
	uint8_t pcm_format;
	uint8_t pcm_sample_bytes;
	uint8_t pcm_rate_divisor;
	uint32_t pcm_interp_count;
	uint32_t pcm_interp_index;
	uint32_t pcm_start_threshold_bytes;
	float pcm_current_sample;
	float pcm_next_sample;
	bool pcm_configured;
};

static const struct gpio_dt_spec haptics_enable = GPIO_DT_SPEC_GET(IBEX_HAPTICS_NODE, enable_gpios);
/*
 * There is a separate enable GPIO per haptics sequencer instance. The touchpad/PWM
 * sequencer uses GPIO0.9, while the GRI v3/I2S sequencer uses GPIO1.5.
 */
static const struct gpio_dt_spec haptics_secondary_enable =
    GPIO_DT_SPEC_GET_OR(IBEX_HAPTICS_NODE, secondary_enable_gpios, { 0 });
static const struct device *const haptics_i2s = DEVICE_DT_GET(IBEX_HAPTICS_I2S_NODE);
static const nrfx_pwm_t haptics_pwm = NRFX_PWM_INSTANCE(IBEX_HAPTICS_PWM_INSTANCE_ID);

static K_MUTEX_DEFINE(haptics_state_mutex);
static K_SEM_DEFINE(haptics_primary_sem, 0, 1);
static K_SEM_DEFINE(haptics_secondary_sem, 0, 1);
static struct k_thread haptics_primary_thread;
static struct k_thread haptics_secondary_thread;
static K_THREAD_STACK_DEFINE(haptics_primary_stack, IBEX_HAPTICS_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(haptics_secondary_stack, IBEX_HAPTICS_THREAD_STACK_SIZE);
static K_MEM_SLAB_DEFINE(haptics_i2s_slab, IBEX_HAPTICS_SECONDARY_BLOCK_BYTES,
                         IBEX_HAPTICS_I2S_BLOCK_COUNT, 4);

static nrf_pwm_values_grouped_t haptics_sequences[IBEX_HAPTICS_PRIMARY_BUFFER_COUNT]
                                                 [IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES];
static nrf_pwm_values_grouped_t haptics_primary_neutral;
static struct ibex_haptics_channel_state haptics_channels[IBEX_HAPTICS_CHANNEL_COUNT]
                                                         [IBEX_HAPTICS_EFFECT_SLOT_COUNT];
static bool haptics_output_threads_started;
static bool haptics_primary_running;
static bool haptics_primary_idle;
static bool haptics_primary_buffer_active[IBEX_HAPTICS_PRIMARY_BUFFER_COUNT];
static bool haptics_secondary_running;
static uint32_t haptics_prng_state = 5323U;
static atomic_t haptics_master_gain_db;
static atomic_t haptics_amplifier_mode;
static atomic_t haptics_primary_completed;
static atomic_t haptics_primary_last_completed;

static atomic_t haptics_play_requests;
static atomic_t haptics_play_started;
static atomic_t haptics_play_suppressed;
static atomic_t haptics_last_requested_channels;
static atomic_t haptics_last_physical_channels;
static atomic_t haptics_last_play_channels;
static atomic_t haptics_last_sample_count;
static atomic_t haptics_last_play_err;

/* OFW haptics_lfo_waveform_lookup indexes 10^(dB/20) for -24..+24 dB. */
static const uint32_t haptics_gain_q15[] = {
	2068,   2320,   2603,   2920,   3277,   3677,   4125,   4629,   5193,   5827,
	6538,   7336,   8231,   9235,   10362,  11627,  13045,  14637,  16423,  18427,
	20675,  23198,  26029,  29205,  32768,  36766,  41252,  46286,  51934,  58271,
	65381,  73358,  82309,  92353,  103622, 116265, 130452, 146369, 164229, 184268,
	206752, 231980, 260285, 292045, 327680, 367663, 412525, 462860, 519338,
};

BUILD_ASSERT(ARRAY_SIZE(haptics_gain_q15) ==
             (IBEX_HAPTICS_GAIN_DB_MAX - IBEX_HAPTICS_GAIN_DB_MIN + 1));

/* OFW haptics_sample8_lookup_normalized table at 0x4f9ae. */
static const int16_t haptics_sample8_lookup[] = {
	/* clang-format off */
	-32124, -31100, -30076, -29052, -28028, -27004, -25980, -24956,
	-23932, -22908, -21884, -20860, -19836, -18812, -17788, -16764,
	-15996, -15484, -14972, -14460, -13948, -13436, -12924, -12412,
	-11900, -11388, -10876, -10364,  -9852,  -9340,  -8828,  -8316,
	 -7932,  -7676,  -7420,  -7164,  -6908,  -6652,  -6396,  -6140,
	 -5884,  -5628,  -5372,  -5116,  -4860,  -4604,  -4348,  -4092,
	 -3900,  -3772,  -3644,  -3516,  -3388,  -3260,  -3132,  -3004,
	 -2876,  -2748,  -2620,  -2492,  -2364,  -2236,  -2108,  -1980,
	 -1884,  -1820,  -1756,  -1692,  -1628,  -1564,  -1500,  -1436,
	 -1372,  -1308,  -1244,  -1180,  -1116,  -1052,   -988,   -924,
	  -876,   -844,   -812,   -780,   -748,   -716,   -684,   -652,
	  -620,   -588,   -556,   -524,   -492,   -460,   -428,   -396,
	  -372,   -356,   -340,   -324,   -308,   -292,   -276,   -260,
	  -244,   -228,   -212,   -196,   -180,   -164,   -148,   -132,
	  -120,   -112,   -104,    -96,    -88,    -80,    -72,    -64,
	   -56,    -48,    -40,    -32,    -24,    -16,     -8,      0,
	 32124,  31100,  30076,  29052,  28028,  27004,  25980,  24956,
	 23932,  22908,  21884,  20860,  19836,  18812,  17788,  16764,
	 15996,  15484,  14972,  14460,  13948,  13436,  12924,  12412,
	 11900,  11388,  10876,  10364,   9852,   9340,   8828,   8316,
	  7932,   7676,   7420,   7164,   6908,   6652,   6396,   6140,
	  5884,   5628,   5372,   5116,   4860,   4604,   4348,   4092,
	  3900,   3772,   3644,   3516,   3388,   3260,   3132,   3004,
	  2876,   2748,   2620,   2492,   2364,   2236,   2108,   1980,
	  1884,   1820,   1756,   1692,   1628,   1564,   1500,   1436,
	  1372,   1308,   1244,   1180,   1116,   1052,    988,    924,
	   876,    844,    812,    780,    748,    716,    684,    652,
	   620,    588,    556,    524,    492,    460,    428,    396,
	   372,    356,    340,    324,    308,    292,    276,    260,
	   244,    228,    212,    196,    180,    164,    148,    132,
	   120,    112,    104,     96,     88,     80,     72,     64,
	    56,     48,     40,     32,     24,     16,      8,      0,
	/* clang-format on */
};

BUILD_ASSERT(ARRAY_SIZE(haptics_sample8_lookup) == 256);

static uint8_t channel_mask_from_index(size_t index)
{
	return BIT(index);
}

static bool channel_is_primary(size_t index)
{
	return index <= IBEX_HAPTICS_INDEX_RIGHT_0;
}

static uint32_t channel_sample_rate_hz(size_t index)
{
	return channel_is_primary(index) ? IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ
	                                 : IBEX_HAPTICS_SECONDARY_SAMPLE_RATE_HZ;
}

static uint32_t channel_block_frames(size_t index)
{
	return channel_is_primary(index) ? IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES
	                                 : IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES;
}

static float gain_from_db(int8_t gain_db)
{
	int32_t gain = CLAMP((int32_t)gain_db, IBEX_HAPTICS_GAIN_DB_MIN, IBEX_HAPTICS_GAIN_DB_MAX);

	return (float)haptics_gain_q15[gain - IBEX_HAPTICS_GAIN_DB_MIN] / IBEX_HAPTICS_GAIN_Q15_ONE;
}

static uint32_t samples_from_duration(uint32_t sample_rate_hz, uint16_t duration_ms)
{
	if(duration_ms == 0)
	{
		return 0;
	}
	return ((uint32_t)duration_ms * sample_rate_hz) / 1000U;
}

static uint32_t pcm_source_rate_hz(uint8_t format)
{
	switch(format & 3U)
	{
		case 0:
			return 8000U;
		case 1:
			return 4000U;
		case 2:
			return 2000U;
		default:
			return 1000U;
	}
}

static uint8_t pcm_rate_divisor(uint8_t format)
{
	switch(format & 3U)
	{
		case 0:
			return 1U;
		case 1:
			return 2U;
		case 2:
			return 4U;
		default:
			return 8U;
	}
}

static uint8_t pcm_config_rate_divisor(uint8_t format)
{
	return format > 10U ? 8U : pcm_rate_divisor(format);
}

static uint32_t pcm_config_source_rate_hz(uint8_t format)
{
	return 8000U / pcm_config_rate_divisor(format);
}

static bool pcm_format_supported(uint8_t format)
{
	return format <= 11U;
}

static uint8_t pcm_sample_bytes_for_format(uint8_t format)
{
	return format <= 3U ? 2U : 1U;
}

static void pcm_apply_decode_format(struct ibex_haptics_channel_state *state,
                                    uint32_t sample_rate_hz, uint8_t format)
{
	uint32_t source_rate_hz = pcm_source_rate_hz(format);

	state->pcm_format = format;
	state->pcm_sample_bytes = pcm_sample_bytes_for_format(format);
	state->pcm_rate_divisor = pcm_rate_divisor(format);
	state->pcm_interp_count = sample_rate_hz / source_rate_hz;
}

static void pcm_init_decode_default(struct ibex_haptics_channel_state *state,
                                    uint32_t sample_rate_hz)
{
	if(state->pcm_sample_bytes == 0U)
	{
		pcm_apply_decode_format(state, sample_rate_hz, 0);
	}
}

static float phase_step_from_hz(uint32_t sample_rate_hz, float frequency_hz)
{
	return (frequency_hz * IBEX_HAPTICS_TWO_PI) / (float)sample_rate_hz;
}

static uint32_t haptics_prng_next15(void)
{
	haptics_prng_state = 8253729U * haptics_prng_state + 2396403U;
	return haptics_prng_state % 0x7FFFU;
}

static void channel_stop(struct ibex_haptics_channel_state *state)
{
	uint8_t pcm_format = state->pcm_format;
	uint8_t pcm_sample_bytes = state->pcm_sample_bytes;
	uint8_t pcm_rate_divisor = state->pcm_rate_divisor;
	uint32_t pcm_interp_count = state->pcm_interp_count;
	uint32_t pcm_start_threshold_bytes = state->pcm_start_threshold_bytes;
	bool pcm_configured = state->pcm_configured;

	memset(state, 0, sizeof(*state));
	state->kind = IBEX_HAPTICS_EFFECT_IDLE;
	state->pcm_format = pcm_format;
	state->pcm_sample_bytes = pcm_sample_bytes;
	state->pcm_rate_divisor = pcm_rate_divisor;
	state->pcm_interp_count = pcm_interp_count;
	state->pcm_start_threshold_bytes = pcm_start_threshold_bytes;
	state->pcm_configured = pcm_configured;
}

static void channel_deactivate_pcm_preserve_buffer(struct ibex_haptics_channel_state *state)
{
	state->kind = IBEX_HAPTICS_EFFECT_IDLE;
	state->pcm_interp_index = 0;
	state->pcm_current_sample = 0.0f;
	state->pcm_next_sample = 0.0f;
}

static bool channel_active(const struct ibex_haptics_channel_state *state)
{
	return state->kind != IBEX_HAPTICS_EFFECT_IDLE;
}

static bool output_channel_active_locked(size_t channel)
{
	for(size_t slot = 0; slot < IBEX_HAPTICS_EFFECT_SLOT_COUNT; ++slot)
	{
		if(channel_active(&haptics_channels[channel][slot]))
		{
			return true;
		}
	}
	return false;
}

static bool any_primary_active_locked(void)
{
	return output_channel_active_locked(IBEX_HAPTICS_INDEX_LEFT_0) ||
	       output_channel_active_locked(IBEX_HAPTICS_INDEX_RIGHT_0);
}

static bool any_secondary_active_locked(void)
{
	return output_channel_active_locked(IBEX_HAPTICS_INDEX_LEFT_1) ||
	       output_channel_active_locked(IBEX_HAPTICS_INDEX_RIGHT_1);
}

static void pcm_ring_clear(struct ibex_haptics_channel_state *state)
{
	state->pcm_head = 0;
	state->pcm_tail = 0;
	state->pcm_count = 0;
	state->pcm_interp_index = 0;
	state->pcm_current_sample = 0.0f;
	state->pcm_next_sample = 0.0f;
}

static void pcm_ring_push(struct ibex_haptics_channel_state *state, uint8_t sample)
{
	if(state->pcm_count == IBEX_HAPTICS_PCM_RING_BYTES)
	{
		return;
	}
	state->pcm[state->pcm_head] = sample;
	state->pcm_head = (state->pcm_head + 1U) % IBEX_HAPTICS_PCM_RING_BYTES;
	state->pcm_count++;
}

static bool pcm_ring_read_source_sample(struct ibex_haptics_channel_state *state, bool consume,
                                        float *sample)
{
	uint8_t bytes[2] = { 0, 0 };

	if(state->pcm_sample_bytes == 0U || state->pcm_count < state->pcm_sample_bytes)
	{
		return false;
	}
	for(uint8_t i = 0; i < state->pcm_sample_bytes; ++i)
	{
		uint16_t index = (state->pcm_tail + i) % IBEX_HAPTICS_PCM_RING_BYTES;

		bytes[i] = state->pcm[index];
	}
	if(consume)
	{
		state->pcm_tail = (state->pcm_tail + state->pcm_sample_bytes) % IBEX_HAPTICS_PCM_RING_BYTES;
		state->pcm_count -= state->pcm_sample_bytes;
	}
	if(state->pcm_sample_bytes == 2U)
	{
		int16_t value = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));

		*sample = (float)value / 32767.0f;
	}
	else if(state->pcm_format >= 8U && state->pcm_format <= 11U)
	{
		*sample = (float)haptics_sample8_lookup[bytes[0]] / 32767.0f;
	}
	else
	{
		*sample = (float)(int8_t)bytes[0] / 127.0f;
	}
	return true;
}

static void channel_start_waveform(struct ibex_haptics_channel_state *state, const void *waveform,
                                   uint32_t waveform_len, uint8_t waveform_format,
                                   uint32_t waveform_rate_hz, int8_t gain_db,
                                   uint32_t sample_rate_hz)
{
	uint64_t total_samples;

	channel_stop(state);
	state->kind = IBEX_HAPTICS_EFFECT_WAVEFORM;
	/* OFW stores this lookup result but its fixed/table waveform step never uses it. */
	ARG_UNUSED(gain_db);
	state->gain = 1.0f;
	state->waveform = waveform;
	state->waveform_len = waveform_len;
	state->waveform_format = waveform_format;
	state->waveform_rate_hz = waveform_rate_hz == 0U ? sample_rate_hz : waveform_rate_hz;
	state->waveform_output_rate_hz = sample_rate_hz;
	if(waveform_len > 0U)
	{
		total_samples = ((uint64_t)waveform_len * sample_rate_hz + state->waveform_rate_hz - 1U) /
		                state->waveform_rate_hz;
		state->total_samples = total_samples > UINT32_MAX ? UINT32_MAX : (uint32_t)total_samples;
	}
}

static void channel_start_sine(struct ibex_haptics_channel_state *state, uint32_t sample_rate_hz,
                               uint16_t frequency_hz, uint16_t duration_ms, int8_t gain_db,
                               float lfo_frequency_hz, uint8_t lfo_depth)
{
	bool was_sine = state->kind == IBEX_HAPTICS_EFFECT_SINE;
	float phase = was_sine ? state->phase : 0.0f;
	float lfo_phase = was_sine ? state->lfo_phase : IBEX_HAPTICS_NEG_PI_4;

	channel_stop(state);
	state->total_samples = samples_from_duration(sample_rate_hz, duration_ms);
	state->kind = IBEX_HAPTICS_EFFECT_SINE;
	state->gain = gain_from_db(gain_db);
	state->phase = phase;
	state->phase_step = phase_step_from_hz(sample_rate_hz, frequency_hz);
	state->lfo_phase = lfo_phase;
	state->lfo_phase_step = phase_step_from_hz(sample_rate_hz, lfo_frequency_hz);
	state->lfo_depth = ((float)MIN(lfo_depth, (uint8_t)100) / 100.0f) * 0.5f;
}

static void channel_start_pulse(struct ibex_haptics_channel_state *state, uint32_t sample_rate_hz,
                                uint16_t on_us, uint16_t off_us, uint16_t repeat_count)
{
	uint32_t sample_period_us = 1000000U / sample_rate_hz;

	channel_stop(state);
	if(on_us == 0 || repeat_count == 0)
	{
		return;
	}
	state->period_samples = (on_us + (uint32_t)off_us) / sample_period_us;
	state->high_samples = on_us / sample_period_us;
	if(state->period_samples == 0 || state->high_samples == 0)
	{
		return;
	}
	state->high_samples = MIN(state->high_samples, state->period_samples);
	state->repeat_count = repeat_count;
	state->total_samples = state->period_samples * repeat_count;
	state->gain = 1.0f;
	state->kind = IBEX_HAPTICS_EFFECT_PULSE;
}

static void channel_start_log_sweep(struct ibex_haptics_channel_state *state,
                                    uint32_t sample_rate_hz, uint16_t start_frequency_hz,
                                    uint16_t end_frequency_hz, uint16_t duration_ms, int8_t gain_db,
                                    bool invert)
{
	channel_stop(state);
	state->total_samples = samples_from_duration(sample_rate_hz, duration_ms);
	if(start_frequency_hz == 0 || end_frequency_hz == 0 || state->total_samples == 0)
	{
		return;
	}
	state->kind = IBEX_HAPTICS_EFFECT_LOG_SWEEP;
	state->gain = gain_from_db(gain_db);
	state->invert = invert;
	if(start_frequency_hz == end_frequency_hz)
	{
		state->phase_step = phase_step_from_hz(sample_rate_hz, start_frequency_hz);
	}
	else
	{
		state->log_ratio = log1pf(((float)end_frequency_hz - (float)start_frequency_hz) /
		                          (float)start_frequency_hz);
		state->phase_scale = ((float)duration_ms / 1000.0f) *
		                     ((float)start_frequency_hz * IBEX_HAPTICS_TWO_PI) /
		                     state->log_ratio;
	}
	state->attack_samples = (uint32_t)((float)sample_rate_hz * IBEX_HAPTICS_LOG_SWEEP_ATTACK_S);
	state->release_start_sample =
	    state->total_samples -
	    MIN(state->total_samples,
	        (uint32_t)((float)sample_rate_hz * IBEX_HAPTICS_LOG_SWEEP_RELEASE_S));
	state->attack_samples = MIN(state->attack_samples, state->total_samples / 2U);
}

static void channel_start_random_lfo(struct ibex_haptics_channel_state *state,
                                     uint32_t sample_rate_hz, uint16_t frequency_hz,
                                     uint16_t duration_ms, int8_t gain_db, float random_rate_hz)
{
	bool was_random = state->kind == IBEX_HAPTICS_EFFECT_RANDOM_LFO;
	float phase = was_random ? state->phase : 0.0f;
	uint32_t random_period_index = was_random ? state->random_period_index : 0U;
	uint16_t carrier_cycles = was_random ? state->carrier_cycles : 0U;
	float random_gain = was_random ? state->random_gain : 1.0f;

	channel_stop(state);
	state->total_samples = samples_from_duration(sample_rate_hz, duration_ms);
	if(frequency_hz == 0 || random_rate_hz == 0.0f || state->total_samples == 0)
	{
		return;
	}
	state->kind = IBEX_HAPTICS_EFFECT_RANDOM_LFO;
	state->gain = gain_from_db(gain_db);
	state->phase = phase;
	state->phase_step = phase_step_from_hz(sample_rate_hz, frequency_hz);
	state->random_period_samples = MAX(1U, (uint32_t)((float)sample_rate_hz / random_rate_hz));
	state->random_period_index = random_period_index;
	state->carrier_cycles = carrier_cycles;
	state->random_gain = random_gain;
}

static void channel_configure_pcm(struct ibex_haptics_channel_state *state, size_t index,
                                  uint8_t format)
{
	uint32_t sample_rate_hz = channel_sample_rate_hz(index);
	uint8_t threshold_sample_bytes = pcm_sample_bytes_for_format(format);
	uint8_t threshold_rate_divisor = pcm_config_rate_divisor(format);
	uint32_t start_threshold;

	if(pcm_config_source_rate_hz(format) > sample_rate_hz)
	{
		return;
	}

	channel_stop(state);
	pcm_init_decode_default(state, sample_rate_hz);
	if(pcm_format_supported(format))
	{
		pcm_apply_decode_format(state, sample_rate_hz, format);
	}
	else
	{
		state->pcm_format = format;
	}

	start_threshold =
	    (2U * channel_block_frames(index) * threshold_sample_bytes) / threshold_rate_divisor;
	state->pcm_start_threshold_bytes = start_threshold;
	state->pcm_configured = true;
}

static void channel_start_pcm(struct ibex_haptics_channel_state *state, size_t index,
                              const uint8_t *samples, uint8_t sample_count)
{
	bool start;

	if(state->kind != IBEX_HAPTICS_EFFECT_PCM && state->kind != IBEX_HAPTICS_EFFECT_IDLE)
	{
		channel_stop(state);
	}

	pcm_init_decode_default(state, channel_sample_rate_hz(index));
	for(uint8_t i = 0; i < sample_count; ++i)
	{
		pcm_ring_push(state, samples[i]);
	}
	start = state->pcm_configured &&
	        state->kind != IBEX_HAPTICS_EFFECT_PCM &&
	        state->pcm_count >= state->pcm_start_threshold_bytes;
	if(start)
	{
		state->kind = IBEX_HAPTICS_EFFECT_PCM;
		state->pcm_interp_index = 0;
	}
}

static float waveform_raw_sample(const struct ibex_haptics_channel_state *state, uint32_t index)
{
	if(index >= state->waveform_len)
	{
		return 0.0f;
	}
	switch(state->waveform_format)
	{
		case HAPTICS_WAVEFORM_SAMPLE_FORMAT_S16:
			return (float)((const int16_t *)state->waveform)[index] / 32767.0f;
		case HAPTICS_WAVEFORM_SAMPLE_FORMAT_S8:
		default:
			return (float)((const int8_t *)state->waveform)[index] / 127.0f;
	}
}

static float render_waveform_sample(struct ibex_haptics_channel_state *state)
{
	uint32_t index;
	uint32_t phase_remainder;
	uint32_t next_phase;
	float sample;

	if(state->sample_index >= state->total_samples)
	{
		channel_stop(state);
		return 0.0f;
	}

	index = state->waveform_source_index;
	phase_remainder = state->waveform_phase;
	if(index >= state->waveform_len)
	{
		channel_stop(state);
		return 0.0f;
	}

	sample = waveform_raw_sample(state, index);
	if(phase_remainder != 0U && index + 1U < state->waveform_len)
	{
		float next = waveform_raw_sample(state, index + 1U);
		float frac = (float)phase_remainder / (float)state->waveform_output_rate_hz;

		sample += (next - sample) * frac;
	}

	next_phase = state->waveform_phase + state->waveform_rate_hz;
	state->waveform_source_index += next_phase / state->waveform_output_rate_hz;
	state->waveform_phase = next_phase % state->waveform_output_rate_hz;
	state->sample_index++;
	return sample * state->gain;
}

static float render_sine_sample(struct ibex_haptics_channel_state *state)
{
	float sample;

	if(state->total_samples != 0 && state->sample_index >= state->total_samples)
	{
		channel_stop(state);
		return 0.0f;
	}

	sample = sinf(state->phase) * state->gain;
	state->phase += state->phase_step;

	if(state->lfo_depth != 0.0f && state->lfo_phase_step != 0.0f)
	{
		float lfo_phase = state->lfo_phase;
		float next_lfo_phase = state->lfo_phase_step + lfo_phase;
		float modulation = (1.0f - state->lfo_depth) + (state->lfo_depth * sinf(lfo_phase));

		sample *= modulation;
		if(next_lfo_phase > IBEX_HAPTICS_TWO_PI)
		{
			next_lfo_phase -= IBEX_HAPTICS_TWO_PI;
		}
		state->lfo_phase = next_lfo_phase;
	}

	state->sample_index++;
	return sample;
}

static float render_pulse_sample(struct ibex_haptics_channel_state *state)
{
	uint32_t period_index;
	float sample;

	if(state->sample_index >= state->total_samples)
	{
		channel_stop(state);
		return 0.0f;
	}

	period_index = state->sample_index % state->period_samples;
	sample = period_index < state->high_samples ? state->gain : -state->gain;
	state->sample_index++;
	return sample;
}

static float render_log_sweep_sample(struct ibex_haptics_channel_state *state)
{
	float phase;
	float envelope = 1.0f;
	float sample;

	if(state->sample_index >= state->total_samples)
	{
		channel_stop(state);
		return 0.0f;
	}

	if(state->log_ratio == 0.0f)
	{
		phase = state->phase_step * (float)state->sample_index;
	}
	else
	{
		phase =
		    state->phase_scale *
		    expm1f(((float)state->sample_index / (float)state->total_samples) * state->log_ratio);
	}
	if(state->sample_index < state->attack_samples && state->attack_samples != 0)
	{
		envelope = (float)state->sample_index / (float)state->attack_samples;
	}
	if(state->sample_index >= state->release_start_sample &&
	   state->release_start_sample + 1U < state->total_samples)
	{
		uint32_t release_index = state->sample_index - state->release_start_sample;
		uint32_t release_samples = state->total_samples - state->release_start_sample;

		envelope = MIN(envelope, 1.0f - ((float)release_index / (float)(release_samples - 1U)));
	}

	sample = sinf(phase) * envelope * state->gain;
	if(state->invert)
	{
		sample = -sample;
	}
	state->sample_index++;
	return sample;
}

static float render_random_lfo_sample(struct ibex_haptics_channel_state *state)
{
	float sample;
	uint32_t period_index;
	uint32_t sample_index;

	if(state->sample_index > state->total_samples)
	{
		channel_stop(state);
		return 0.0f;
	}
	period_index = state->random_period_index;
	state->random_period_index = period_index + 1U;
	if(period_index > state->random_period_samples)
	{
		state->random_period_index = 0;
		state->carrier_cycles = 0;
		state->phase = 0.0f;
		state->random_gain = gain_from_db(3 - (int8_t)(haptics_prng_next15() & 7U));
	}

	state->phase += state->phase_step;
	if(state->phase > IBEX_HAPTICS_TWO_PI)
	{
		state->phase -= IBEX_HAPTICS_TWO_PI;
		state->carrier_cycles++;
	}
	if(state->carrier_cycles > 2U)
	{
		sample = 0.0f;
	}
	else
	{
		sample = sinf(state->phase) * state->gain * state->random_gain;
	}

	sample_index = state->sample_index;
	state->sample_index = sample_index + 1U;
	if(sample_index == state->total_samples)
	{
		channel_stop(state);
	}
	return sample;
}

static float render_pcm_sample(struct ibex_haptics_channel_state *state)
{
	float sample;

	if(state->pcm_interp_count <= 1U)
	{
		if(!pcm_ring_read_source_sample(state, true, &sample))
		{
			pcm_ring_clear(state);
			channel_stop(state);
			return 0.0f;
		}
		return sample;
	}

	if(state->pcm_interp_index == 0U)
	{
		if(!pcm_ring_read_source_sample(state, true, &state->pcm_current_sample))
		{
			pcm_ring_clear(state);
			channel_stop(state);
			return 0.0f;
		}
		if(!pcm_ring_read_source_sample(state, false, &state->pcm_next_sample))
		{
			sample = state->pcm_current_sample;
			channel_deactivate_pcm_preserve_buffer(state);
			return sample;
		}
	}

	sample = state->pcm_current_sample +
	         ((state->pcm_next_sample - state->pcm_current_sample) *
	          ((float)state->pcm_interp_index / (float)state->pcm_interp_count));
	state->pcm_interp_index++;
	if(state->pcm_interp_index >= state->pcm_interp_count)
	{
		state->pcm_interp_index = 0;
	}
	return sample;
}

static float render_effect_sample(struct ibex_haptics_channel_state *state)
{
	switch(state->kind)
	{
		case IBEX_HAPTICS_EFFECT_WAVEFORM:
			return render_waveform_sample(state);
		case IBEX_HAPTICS_EFFECT_SINE:
			return render_sine_sample(state);
		case IBEX_HAPTICS_EFFECT_PULSE:
			return render_pulse_sample(state);
		case IBEX_HAPTICS_EFFECT_LOG_SWEEP:
			return render_log_sweep_sample(state);
		case IBEX_HAPTICS_EFFECT_RANDOM_LFO:
			return render_random_lfo_sample(state);
		case IBEX_HAPTICS_EFFECT_PCM:
			return render_pcm_sample(state);
		case IBEX_HAPTICS_EFFECT_IDLE:
		default:
			return 0.0f;
	}
}

static float render_output_channel_sample(size_t channel)
{
	float sample = 0.0f;
	float master_gain = gain_from_db((int8_t)atomic_get(&haptics_master_gain_db));

	for(size_t slot = 0; slot < IBEX_HAPTICS_EFFECT_SLOT_COUNT; ++slot)
	{
		sample += render_effect_sample(&haptics_channels[channel][slot]);
	}
	/* OFW channel wiring inverts left0 and right1 after applying the master gain. */
	if(channel == IBEX_HAPTICS_INDEX_LEFT_0 || channel == IBEX_HAPTICS_INDEX_RIGHT_1)
	{
		master_gain = -master_gain;
	}
	return sample * master_gain;
}

static uint16_t pwm_compare_from_sample(float sample)
{
	float compare;

	sample = CLAMP(sample, -1.0f, 1.0f);
	/* OFW rounds round(127.5 + sample * 127.5), so neutral is compare 128. */
	compare = (sample * 127.5f) + 127.5f;
	return (uint16_t)CLAMP((int32_t)(compare + 0.5f), 0, 255);
}

static int16_t i2s_sample_from_float(float sample)
{
	int32_t value;

	sample = CLAMP(sample, -1.0f, 1.0f);
	value = (int32_t)(sample * 32767.0f);
	return (int16_t)CLAMP(value, (int32_t)INT16_MIN, (int32_t)INT16_MAX);
}

static void sequence_set_channel(nrf_pwm_values_grouped_t *sample, uint8_t channel, uint16_t value)
{
	if(channel < 2U)
	{
		sample->group_0 = value;
	}
	else
	{
		sample->group_1 = value;
	}
}

static void sequence_clear_sample(nrf_pwm_values_grouped_t *sample)
{
	uint16_t neutral = pwm_compare_from_sample(0.0f);

	sample->group_0 = neutral;
	sample->group_1 = neutral;
}

static bool render_primary_block(nrf_pwm_values_grouped_t *sequence, uint32_t silence_samples)
{
	bool active;

	silence_samples = MIN(silence_samples, IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES);
	k_mutex_lock(&haptics_state_mutex, K_FOREVER);
	active = any_primary_active_locked();
	for(uint32_t i = 0; i < IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES; ++i)
	{
		float left = 0.0f;
		float right = 0.0f;

		if(i >= silence_samples)
		{
			left = render_output_channel_sample(IBEX_HAPTICS_INDEX_LEFT_0);
			right = render_output_channel_sample(IBEX_HAPTICS_INDEX_RIGHT_0);
		}
		sequence_clear_sample(&sequence[i]);
		sequence_set_channel(&sequence[i], IBEX_HAPTICS_LEFT_CHANNEL,
		                     pwm_compare_from_sample(left));
		sequence_set_channel(&sequence[i], IBEX_HAPTICS_RIGHT_CHANNEL,
		                     pwm_compare_from_sample(right));
	}
	k_mutex_unlock(&haptics_state_mutex);

	return active;
}

static bool render_secondary_block(int16_t *samples, uint32_t silence_frames)
{
	bool active;

	silence_frames = MIN(silence_frames, IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES);
	k_mutex_lock(&haptics_state_mutex, K_FOREVER);
	active = any_secondary_active_locked();
	for(uint32_t i = 0; i < IBEX_HAPTICS_SECONDARY_BLOCK_FRAMES; ++i)
	{
		float left = 0.0f;
		float right = 0.0f;

		if(i >= silence_frames)
		{
			left = render_output_channel_sample(IBEX_HAPTICS_INDEX_LEFT_1);
			right = render_output_channel_sample(IBEX_HAPTICS_INDEX_RIGHT_1);
		}
		samples[2U * i] = i2s_sample_from_float(left);
		samples[2U * i + 1U] = i2s_sample_from_float(right);
	}
	k_mutex_unlock(&haptics_state_mutex);

	return active;
}

static bool amplifier_forced_on(void)
{
	return atomic_get(&haptics_amplifier_mode) != 0;
}

static void haptics_pwm_event_handler(nrfx_pwm_evt_type_t event_type, void *context)
{
	ARG_UNUSED(context);

	if(event_type == NRFX_PWM_EVT_END_SEQ0)
	{
		atomic_or(&haptics_primary_completed, BIT(0));
		atomic_set(&haptics_primary_last_completed, 0);
		k_sem_give(&haptics_primary_sem);
	}
	else if(event_type == NRFX_PWM_EVT_END_SEQ1)
	{
		atomic_or(&haptics_primary_completed, BIT(1));
		atomic_set(&haptics_primary_last_completed, 1);
		k_sem_give(&haptics_primary_sem);
	}
}

static bool primary_state_active(void)
{
	bool active;

	k_mutex_lock(&haptics_state_mutex, K_FOREVER);
	active = any_primary_active_locked();
	k_mutex_unlock(&haptics_state_mutex);
	return active;
}

static void primary_start_neutral(void)
{
	nrf_pwm_sequence_t sequence = {
		.values.p_grouped = &haptics_primary_neutral,
		.length = 2U,
		.repeats = 0,
		.end_delay = 0,
	};

	sequence_clear_sample(&haptics_primary_neutral);
	atomic_set(&haptics_primary_completed, 0);
	(void)nrfx_pwm_simple_playback(&haptics_pwm, &sequence, 1,
	                               NRFX_PWM_FLAG_LOOP | NRFX_PWM_FLAG_NO_EVT_FINISHED);
	haptics_primary_idle = true;
}

static int pwm_stop_outputs(void)
{
	int err = gpio_pin_set_dt(&haptics_enable, amplifier_forced_on() ? 1 : 0);

	if(haptics_primary_running && nrfx_pwm_init_check(&haptics_pwm))
	{
		(void)nrfx_pwm_stop(&haptics_pwm, true);
	}
	haptics_primary_running = false;
	haptics_primary_buffer_active[0] = false;
	haptics_primary_buffer_active[1] = false;
	atomic_set(&haptics_primary_completed, 0);
	if(nrfx_pwm_init_check(&haptics_pwm) && !haptics_primary_idle)
	{
		/* OFW leaves the last neutral PWM value driving the pins between effects. */
		primary_start_neutral();
	}
	return err;
}

static int primary_start(void)
{
	nrf_pwm_sequence_t sequences[IBEX_HAPTICS_PRIMARY_BUFFER_COUNT] = {
		{
		    .values.p_grouped = haptics_sequences[0],
		    .length = IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES * 2U,
		    .repeats = IBEX_HAPTICS_PWM_SAMPLE_REPEATS,
		    .end_delay = 0,
		},
		{
		    .values.p_grouped = haptics_sequences[1],
		    .length = IBEX_HAPTICS_PRIMARY_BLOCK_SAMPLES * 2U,
		    .repeats = IBEX_HAPTICS_PWM_SAMPLE_REPEATS,
		    .end_delay = 0,
		},
	};
	int err;

	haptics_primary_buffer_active[0] =
	    render_primary_block(haptics_sequences[0], IBEX_HAPTICS_PRIMARY_START_SILENCE_SAMPLES);
	haptics_primary_buffer_active[1] = render_primary_block(haptics_sequences[1], 0);
	if(!haptics_primary_buffer_active[0] &&
	   !haptics_primary_buffer_active[1] &&
	   !amplifier_forced_on())
	{
		return 0;
	}

	if(haptics_primary_idle)
	{
		(void)nrfx_pwm_stop(&haptics_pwm, true);
		haptics_primary_idle = false;
	}

	atomic_set(&haptics_primary_completed, 0);
	atomic_set(&haptics_primary_last_completed, 0);
	(void)nrfx_pwm_complex_playback(&haptics_pwm, &sequences[0], &sequences[1], 1,
	                                NRFX_PWM_FLAG_LOOP |
	                                    NRFX_PWM_FLAG_SIGNAL_END_SEQ0 |
	                                    NRFX_PWM_FLAG_SIGNAL_END_SEQ1 |
	                                    NRFX_PWM_FLAG_NO_EVT_FINISHED);
	haptics_primary_running = true;
	err = gpio_pin_set_dt(&haptics_enable, 1);
	if(err)
	{
		(void)nrfx_pwm_stop(&haptics_pwm, true);
		haptics_primary_running = false;
		primary_start_neutral();
		return err;
	}
	return 0;
}

static int primary_refill(void)
{
	atomic_val_t completed = atomic_set(&haptics_primary_completed, 0);

	if(!haptics_primary_running || completed == 0)
	{
		return 0;
	}
	if((completed & (BIT(0) | BIT(1))) == (BIT(0) | BIT(1)))
	{
		/* The most recently completed sequence is safe while its peer is playing. */
		completed = BIT(atomic_get(&haptics_primary_last_completed));
	}

	for(size_t i = 0; i < IBEX_HAPTICS_PRIMARY_BUFFER_COUNT; ++i)
	{
		if((completed & BIT(i)) != 0)
		{
			haptics_primary_buffer_active[i] = render_primary_block(haptics_sequences[i], 0);
		}
	}

	if(!haptics_primary_buffer_active[0] &&
	   !haptics_primary_buffer_active[1] &&
	   !primary_state_active() &&
	   !amplifier_forced_on())
	{
		return pwm_stop_outputs();
	}
	return 0;
}

static int secondary_enable_set(bool enabled)
{
	if(haptics_secondary_enable.port == NULL)
	{
		return 0;
	}
	return gpio_pin_set_dt(&haptics_secondary_enable, enabled ? 1 : 0);
}

static void secondary_stop(void)
{
	if(haptics_secondary_running)
	{
		void *returned[IBEX_HAPTICS_I2S_BLOCK_COUNT] = { 0 };

		if(i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN) < 0)
		{
			(void)i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
		}
		for(size_t i = 0; i < ARRAY_SIZE(returned); ++i)
		{
			if(k_mem_slab_alloc(&haptics_i2s_slab, &returned[i], K_FOREVER) != 0)
			{
				break;
			}
		}
		for(size_t i = 0; i < ARRAY_SIZE(returned); ++i)
		{
			if(returned[i] != NULL)
			{
				k_mem_slab_free(&haptics_i2s_slab, returned[i]);
			}
		}
		haptics_secondary_running = false;
	}
	(void)secondary_enable_set(amplifier_forced_on());
}

static void secondary_drop(void)
{
	(void)i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
	haptics_secondary_running = false;
	(void)secondary_enable_set(amplifier_forced_on());
}

static int secondary_recover_block(void *block)
{
	int err = i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_PREPARE);

	if(err)
	{
		secondary_drop();
		k_mem_slab_free(&haptics_i2s_slab, block);
		return err;
	}

	err = i2s_write(haptics_i2s, block, IBEX_HAPTICS_SECONDARY_BLOCK_BYTES);
	if(err)
	{
		secondary_drop();
		k_mem_slab_free(&haptics_i2s_slab, block);
		return err;
	}

	err = secondary_enable_set(true);
	if(!err)
	{
		err = i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	}
	if(err)
	{
		/* The successful retry transferred ownership of block to the I2S driver. */
		secondary_drop();
		return err;
	}

	haptics_secondary_running = true;
	return 1;
}

static bool secondary_queue_full(int err)
{
	return err == -ENOMEM || err == -EAGAIN || err == -ENOMSG;
}

static int secondary_queue_block(uint32_t silence_frames)
{
	void *block;
	int err;

	err = k_mem_slab_alloc(&haptics_i2s_slab, &block, K_FOREVER);
	if(err)
	{
		return err;
	}

	if(!render_secondary_block((int16_t *)block, silence_frames) && !amplifier_forced_on())
	{
		k_mem_slab_free(&haptics_i2s_slab, block);
		return 0;
	}

	err = i2s_write(haptics_i2s, block, IBEX_HAPTICS_SECONDARY_BLOCK_BYTES);
	if(err == -EIO)
	{
		return secondary_recover_block(block);
	}
	if(err)
	{
		k_mem_slab_free(&haptics_i2s_slab, block);
		return err;
	}

	return 1;
}

static int secondary_write_block(void)
{
	uint32_t target_blocks = haptics_secondary_running ? 1U : IBEX_HAPTICS_I2S_START_BLOCKS;
	uint32_t queued_blocks = 0;
	int err = 0;

	for(uint32_t i = 0; i < target_blocks; ++i)
	{
		uint32_t silence_frames =
		    !haptics_secondary_running && i == 0 ? IBEX_HAPTICS_SECONDARY_START_SILENCE_FRAMES : 0;

		err = secondary_queue_block(silence_frames);
		if(err < 0)
		{
			return secondary_queue_full(err) ? 0 : err;
		}
		if(err == 0)
		{
			break;
		}
		queued_blocks++;
	}

	if(queued_blocks == 0)
	{
		return 0;
	}

	if(!haptics_secondary_running)
	{
		err = secondary_enable_set(true);
		if(err)
		{
			(void)i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_DROP);
			return err;
		}
		err = i2s_trigger(haptics_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
		if(err)
		{
			/* i2s_write() owns the block; DROP purges queued TX buffers in i2s_nrfx. */
			secondary_drop();
			return err;
		}
		haptics_secondary_running = true;
	}

	return 0;
}

static void haptics_primary_thread_main(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for(;;)
	{
		bool primary_active;
		int err;

		k_sem_take(&haptics_primary_sem, K_FOREVER);

		if(haptics_primary_running)
		{
			err = primary_refill();
		}
		else
		{
			err = 0;
		}

		k_mutex_lock(&haptics_state_mutex, K_FOREVER);
		primary_active = any_primary_active_locked();
		k_mutex_unlock(&haptics_state_mutex);
		if((primary_active || amplifier_forced_on()) && !haptics_primary_running)
		{
			int pwm_err = primary_start();

			if(err == 0)
			{
				err = pwm_err;
			}
		}

		if(err)
		{
			atomic_inc(&haptics_play_suppressed);
			atomic_set(&haptics_last_play_err, err);
		}
	}
}

static void haptics_secondary_thread_main(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	for(;;)
	{
		bool secondary_active;
		int err;

		k_mutex_lock(&haptics_state_mutex, K_FOREVER);
		secondary_active = any_secondary_active_locked();
		k_mutex_unlock(&haptics_state_mutex);
		if(!secondary_active && !amplifier_forced_on())
		{
			secondary_stop();
			k_sem_take(&haptics_secondary_sem, K_FOREVER);
			continue;
		}

		err = secondary_write_block();
		if(err && !secondary_queue_full(err))
		{
			atomic_inc(&haptics_play_suppressed);
			atomic_set(&haptics_last_play_err, err);
		}
	}
}

static uint32_t effect_frequency_hz(const struct haptics_effect *effect)
{
	return effect->frequency_hz == 0 ? IBEX_HAPTICS_DEFAULT_FREQUENCY_HZ : effect->frequency_hz;
}

static uint8_t prepare_channels(uint8_t channels)
{
	uint8_t physical_channels = channels & IBEX_HAPTICS_SUPPORTED_CHANNELS;

	atomic_inc(&haptics_play_requests);
	atomic_set(&haptics_last_requested_channels, channels);
	atomic_set(&haptics_last_physical_channels, physical_channels);
	if(physical_channels == 0)
	{
		atomic_inc(&haptics_play_suppressed);
		atomic_set(&haptics_last_play_channels, 0);
		atomic_set(&haptics_last_sample_count, 0);
		atomic_set(&haptics_last_play_err, 0);
	}
	return physical_channels;
}

static void apply_to_channels(uint8_t channels, enum ibex_haptics_effect_slot slot,
                              void (*apply)(struct ibex_haptics_channel_state *state, size_t index,
                                            void *context),
                              void *context)
{
	for(size_t i = 0; i < IBEX_HAPTICS_CHANNEL_COUNT; ++i)
	{
		if((channels & channel_mask_from_index(i)) != 0)
		{
			apply(&haptics_channels[i][slot], i, context);
		}
	}
}

static uint8_t inactive_slot_channels_locked(uint8_t channels, enum ibex_haptics_effect_slot slot)
{
	uint8_t inactive_channels = 0;

	for(size_t i = 0; i < IBEX_HAPTICS_CHANNEL_COUNT; ++i)
	{
		uint8_t channel = channel_mask_from_index(i);

		if((channels & channel) != 0 && !channel_active(&haptics_channels[i][slot]))
		{
			inactive_channels |= channel;
		}
	}
	return inactive_channels;
}

struct waveform_context
{
	const void *waveform;
	uint32_t waveform_len;
	uint8_t waveform_format;
	uint32_t waveform_rate_hz;
	int8_t gain_db;
};

static void apply_waveform(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct waveform_context *waveform = context;

	channel_start_waveform(state, waveform->waveform, waveform->waveform_len,
	                       waveform->waveform_format, waveform->waveform_rate_hz, waveform->gain_db,
	                       channel_sample_rate_hz(index));
}

struct sine_context
{
	uint16_t frequency_hz;
	uint16_t duration_ms;
	int8_t gain_db;
	float lfo_frequency_hz;
	uint8_t lfo_depth;
};

static void apply_sine(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct sine_context *sine = context;

	channel_start_sine(state, channel_sample_rate_hz(index), sine->frequency_hz, sine->duration_ms,
	                   sine->gain_db, sine->lfo_frequency_hz, sine->lfo_depth);
}

struct pulse_context
{
	uint16_t on_us;
	uint16_t off_us;
	uint16_t repeat_count;
};

static void apply_pulse(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct pulse_context *pulse = context;

	channel_start_pulse(state, channel_sample_rate_hz(index), pulse->on_us, pulse->off_us,
	                    pulse->repeat_count);
}

struct log_sweep_context
{
	uint8_t invert_channels;
	uint16_t start_frequency_hz;
	uint16_t end_frequency_hz;
	uint16_t duration_ms;
	int8_t gain_db;
};

static void apply_log_sweep(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct log_sweep_context *sweep = context;
	uint8_t channel = channel_mask_from_index(index);

	channel_start_log_sweep(state, channel_sample_rate_hz(index), sweep->start_frequency_hz,
	                        sweep->end_frequency_hz, sweep->duration_ms, sweep->gain_db,
	                        (sweep->invert_channels & channel) != 0);
}

struct random_lfo_context
{
	uint16_t frequency_hz;
	uint16_t duration_ms;
	int8_t gain_db;
	float random_rate_hz;
};

static void apply_random_lfo(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct random_lfo_context *random = context;

	channel_start_random_lfo(state, channel_sample_rate_hz(index), random->frequency_hz,
	                         random->duration_ms, random->gain_db, random->random_rate_hz);
}

struct pcm_context
{
	const uint8_t *samples;
	uint8_t sample_count;
};

static void apply_pcm(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct pcm_context *pcm = context;

	channel_start_pcm(state, index, pcm->samples, pcm->sample_count);
}

struct pcm_config_context
{
	uint8_t format;
};

static void apply_pcm_config(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	const struct pcm_config_context *config = context;

	channel_configure_pcm(state, index, config->format);
}

static void apply_stop(struct ibex_haptics_channel_state *state, size_t index, void *context)
{
	ARG_UNUSED(index);
	ARG_UNUSED(context);
	channel_stop(state);
}

static void wake_output_channels(uint8_t channels)
{
	if((channels & HAPTICS_CHANNELS_PRIMARY) != 0)
	{
		k_sem_give(&haptics_primary_sem);
	}
	if((channels & (HAPTICS_CHANNEL_LEFT_1 | HAPTICS_CHANNEL_RIGHT_1)) != 0)
	{
		k_sem_give(&haptics_secondary_sem);
	}
}

static int submit_channels(uint8_t channels, enum ibex_haptics_effect_slot slot,
                           void (*apply)(struct ibex_haptics_channel_state *state, size_t index,
                                         void *context),
                           void *context, uint32_t sample_count)
{
	channels = prepare_channels(channels);
	if(channels == 0)
	{
		return 0;
	}

	k_mutex_lock(&haptics_state_mutex, K_FOREVER);
	apply_to_channels(channels, slot, apply, context);
	k_mutex_unlock(&haptics_state_mutex);

	atomic_inc(&haptics_play_started);
	atomic_set(&haptics_last_play_channels, channels);
	atomic_set(&haptics_last_sample_count, sample_count);
	atomic_set(&haptics_last_play_err, 0);
	wake_output_channels(channels);
	return 0;
}

static int submit_channels_start(uint8_t channels, enum ibex_haptics_effect_slot slot,
                                 void (*apply)(struct ibex_haptics_channel_state *state,
                                               size_t index, void *context),
                                 void *context, uint32_t sample_count)
{
	uint8_t play_channels;

	channels = prepare_channels(channels);
	if(channels == 0)
	{
		return 0;
	}

	k_mutex_lock(&haptics_state_mutex, K_FOREVER);
	/*
	 * Keep repeated short fixed-waveform feedback from restarting its attack while it is
	 * already sounding. Synthesized effects use submit_channels() because OFW replaces
	 * their per-output slot when a new start command is dequeued.
	 */
	play_channels = inactive_slot_channels_locked(channels, slot);
	if(play_channels != 0)
	{
		apply_to_channels(play_channels, slot, apply, context);
	}
	k_mutex_unlock(&haptics_state_mutex);

	if(play_channels != channels)
	{
		atomic_inc(&haptics_play_suppressed);
	}
	if(play_channels == 0)
	{
		atomic_set(&haptics_last_play_channels, 0);
		atomic_set(&haptics_last_sample_count, sample_count);
		atomic_set(&haptics_last_play_err, -EALREADY);
		return 0;
	}

	atomic_inc(&haptics_play_started);
	atomic_set(&haptics_last_play_channels, play_channels);
	atomic_set(&haptics_last_sample_count, sample_count);
	atomic_set(&haptics_last_play_err, 0);
	wake_output_channels(play_channels);
	return 0;
}

int haptics_backend_init(void)
{
	nrfx_pwm_config_t config =
	    NRFX_PWM_DEFAULT_CONFIG(NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED,
	                            NRF_PWM_PIN_NOT_CONNECTED, NRF_PWM_PIN_NOT_CONNECTED);
	struct i2s_config i2s_config = {
		.word_size = 16,
		.channels = 2,
		.format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_DATA_ORDER_MSB | I2S_FMT_CLK_NF_NB,
		.options = I2S_OPT_BIT_CLK_CONT | I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = IBEX_HAPTICS_SECONDARY_SAMPLE_RATE_HZ,
		.mem_slab = &haptics_i2s_slab,
		.block_size = IBEX_HAPTICS_SECONDARY_BLOCK_BYTES,
		.timeout = SYS_FOREVER_MS,
	};
	nrfx_err_t err;
	int ret;

	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(IBEX_HAPTICS_PWM_INSTANCE_ID)),
	            IRQ_PRIO_LOWEST, NRFX_PWM_INST_HANDLER_GET(IBEX_HAPTICS_PWM_INSTANCE_ID), 0, 0);

	if(!device_is_ready(haptics_enable.port))
	{
		return -ENODEV;
	}
	if(haptics_secondary_enable.port != NULL && !device_is_ready(haptics_secondary_enable.port))
	{
		return -ENODEV;
	}
	if(!device_is_ready(haptics_i2s))
	{
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&haptics_enable, GPIO_OUTPUT_INACTIVE);
	if(ret)
	{
		return ret;
	}
	if(haptics_secondary_enable.port != NULL)
	{
		ret = gpio_pin_configure_dt(&haptics_secondary_enable, GPIO_OUTPUT_INACTIVE);
		if(ret)
		{
			return ret;
		}
	}

	config.output_pins[IBEX_HAPTICS_LEFT_CHANNEL] = IBEX_HAPTICS_LEFT_PIN;
	config.output_pins[IBEX_HAPTICS_RIGHT_CHANNEL] = IBEX_HAPTICS_RIGHT_PIN;
	config.base_clock = NRF_PWM_CLK_16MHz;
	config.top_value = IBEX_HAPTICS_TOP_VALUE;
	config.load_mode = NRF_PWM_LOAD_GROUPED;
	config.step_mode = NRF_PWM_STEP_AUTO;
	config.irq_priority = IRQ_PRIO_LOWEST;

	err = nrfx_pwm_init(&haptics_pwm, &config, haptics_pwm_event_handler, NULL);
	if(err == NRFX_ERROR_ALREADY)
	{
		return -EBUSY;
	}
	if(err != NRFX_SUCCESS)
	{
		return -EIO;
	}

	ret = i2s_configure(haptics_i2s, I2S_DIR_TX, &i2s_config);
	if(ret)
	{
		return ret;
	}

	ret = pwm_stop_outputs();
	if(ret)
	{
		return ret;
	}
	secondary_stop();

	if(!haptics_output_threads_started)
	{
		haptics_output_threads_started = true;
		k_thread_create(&haptics_primary_thread, haptics_primary_stack,
		                K_THREAD_STACK_SIZEOF(haptics_primary_stack), haptics_primary_thread_main,
		                NULL, NULL, NULL, K_PRIO_PREEMPT(IBEX_HAPTICS_THREAD_PRIORITY), K_FP_REGS,
		                K_NO_WAIT);
		k_thread_create(&haptics_secondary_thread, haptics_secondary_stack,
		                K_THREAD_STACK_SIZEOF(haptics_secondary_stack),
		                haptics_secondary_thread_main, NULL, NULL, NULL,
		                K_PRIO_PREEMPT(IBEX_HAPTICS_THREAD_PRIORITY), K_FP_REGS, K_NO_WAIT);
	}

	return 0;
}

int haptics_backend_effect(const struct haptics_effect *effect)
{
	switch(effect->type)
	{
		case HAPTICS_EFFECT_STOP:
			/* OFW routes generic OFF through cleanup slot 9 and does not interrupt active slots. */
			return 0;
		case HAPTICS_EFFECT_STOP_PCM:
			return submit_channels(effect->channels, IBEX_HAPTICS_SLOT_PCM, apply_stop, NULL, 0);
		case HAPTICS_EFFECT_BUTTON_PULSE:
		case HAPTICS_EFFECT_TICK:
		case HAPTICS_EFFECT_PULSE_ONE_SHOT:
		{
			struct waveform_context context = {
				.waveform = haptics_waveform_tick,
				.waveform_len = ARRAY_SIZE(haptics_waveform_tick),
				.waveform_format = HAPTICS_WAVEFORM_TICK_SAMPLE_FORMAT,
				.waveform_rate_hz = HAPTICS_WAVEFORM_TICK_SAMPLE_RATE_HZ,
				.gain_db = effect->gain_db,
			};

			return submit_channels_start(effect->channels, IBEX_HAPTICS_SLOT_TICK, apply_waveform,
			                             &context, ARRAY_SIZE(haptics_waveform_tick));
		}
		case HAPTICS_EFFECT_CLICK:
		case HAPTICS_EFFECT_PULSE_CLICK:
		{
			struct waveform_context context = {
				.waveform = haptics_waveform_click,
				.waveform_len = ARRAY_SIZE(haptics_waveform_click),
				.waveform_format = HAPTICS_WAVEFORM_CLICK_SAMPLE_FORMAT,
				.waveform_rate_hz = HAPTICS_WAVEFORM_CLICK_SAMPLE_RATE_HZ,
				.gain_db = effect->gain_db,
			};

			return submit_channels_start(effect->channels, IBEX_HAPTICS_SLOT_CLICK, apply_waveform,
			                             &context, ARRAY_SIZE(haptics_waveform_click));
		}
		case HAPTICS_EFFECT_TONE:
		{
			uint32_t frequency_hz = effect_frequency_hz(effect);
			struct sine_context context = {
				.frequency_hz = frequency_hz,
				.duration_ms = effect->duration_ms,
				.gain_db = effect->gain_db,
				.lfo_frequency_hz = 0.0f,
				.lfo_depth = 0,
			};

			return submit_channels(
			    effect->channels, IBEX_HAPTICS_SLOT_LFO_TONE, apply_sine, &context,
			    samples_from_duration(IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ, effect->duration_ms));
		}
		case HAPTICS_EFFECT_LFO_TONE:
		{
			struct sine_context context = {
				.frequency_hz = effect->frequency_hz,
				.duration_ms = effect->duration_ms,
				.gain_db = effect->gain_db,
				.lfo_frequency_hz = effect->lfo_frequency_hz,
				.lfo_depth = effect->lfo_depth,
			};

			return submit_channels(
			    effect->channels, IBEX_HAPTICS_SLOT_LFO_TONE, apply_sine, &context,
			    samples_from_duration(IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ, effect->duration_ms));
		}
		case HAPTICS_EFFECT_PULSE_PERIODIC:
		{
			struct pulse_context context = {
				.on_us = effect->on_us,
				.off_us = effect->off_us,
				.repeat_count = effect->repeat_count,
			};

			return submit_channels(
			    effect->channels, IBEX_HAPTICS_SLOT_PULSE, apply_pulse, &context,
			    samples_from_duration(IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ, effect->duration_ms));
		}
		case HAPTICS_EFFECT_LOG_SWEEP:
		{
			struct log_sweep_context context = {
				.invert_channels = effect->invert_channels,
				.start_frequency_hz = effect->frequency_hz,
				.end_frequency_hz = effect->end_frequency_hz,
				.duration_ms = effect->duration_ms,
				.gain_db = effect->gain_db,
			};

			return submit_channels(
			    effect->channels, IBEX_HAPTICS_SLOT_LOG_SWEEP, apply_log_sweep, &context,
			    samples_from_duration(IBEX_HAPTICS_PRIMARY_SAMPLE_RATE_HZ, effect->duration_ms));
		}
		case HAPTICS_EFFECT_RANDOM_LFO:
		{
			struct random_lfo_context context = {
				.frequency_hz = effect->frequency_hz,
				.duration_ms = effect->duration_ms,
				.gain_db = effect->gain_db,
				.random_rate_hz = effect->lfo_frequency_hz,
			};

			return submit_channels(
			    effect->channels, IBEX_HAPTICS_SLOT_RANDOM_LFO, apply_random_lfo, &context,
			    samples_from_duration(IBEX_HAPTICS_SECONDARY_SAMPLE_RATE_HZ, effect->duration_ms));
		}
		case HAPTICS_EFFECT_PCM_CONFIG:
		{
			struct pcm_config_context context = {
				.format = effect->pcm_format,
			};

			return submit_channels(effect->channels, IBEX_HAPTICS_SLOT_PCM, apply_pcm_config,
			                       &context, 0);
		}
		case HAPTICS_EFFECT_PCM_S8:
		{
			struct pcm_context context = {
				.samples = effect->samples,
				.sample_count = effect->sample_count,
			};

			return submit_channels(effect->channels, IBEX_HAPTICS_SLOT_PCM, apply_pcm, &context,
			                       effect->sample_count);
		}
		case HAPTICS_EFFECT_SCREAM:
		{
			struct waveform_context context = {
				.waveform = haptics_waveform_scream,
				.waveform_len = ARRAY_SIZE(haptics_waveform_scream),
				.waveform_format = HAPTICS_WAVEFORM_SCREAM_SAMPLE_FORMAT,
				.waveform_rate_hz = HAPTICS_WAVEFORM_SCREAM_SAMPLE_RATE_HZ,
				.gain_db = effect->gain_db,
			};

			return submit_channels(effect->channels, IBEX_HAPTICS_SLOT_SCREAM, apply_waveform,
			                       &context, ARRAY_SIZE(haptics_waveform_scream));
		}
		default:
			return -EINVAL;
	}
}

int haptics_backend_pulse(void)
{
	struct haptics_effect effect = {
		.type = HAPTICS_EFFECT_BUTTON_PULSE,
		.channels = HAPTICS_CHANNELS_PRIMARY,
	};

	return haptics_backend_effect(&effect);
}

int haptics_backend_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
	struct haptics_effect effect = {
		.type = HAPTICS_EFFECT_TONE,
		.channels = HAPTICS_CHANNELS_PRIMARY,
		.frequency_hz = MIN(frequency_hz, (uint32_t)UINT16_MAX),
		.duration_ms = MIN(duration_ms, (uint32_t)UINT16_MAX),
	};

	return haptics_backend_effect(&effect);
}

int haptics_backend_get_debug(struct haptics_backend_debug *debug)
{
	if(debug == NULL)
	{
		return -EINVAL;
	}

	*debug = (struct haptics_backend_debug){
		.play_requests = (uint32_t)atomic_get(&haptics_play_requests),
		.play_started = (uint32_t)atomic_get(&haptics_play_started),
		.play_suppressed = (uint32_t)atomic_get(&haptics_play_suppressed),
		.last_requested_channels = (uint8_t)atomic_get(&haptics_last_requested_channels),
		.last_physical_channels = (uint8_t)atomic_get(&haptics_last_physical_channels),
		.last_play_channels = (uint8_t)atomic_get(&haptics_last_play_channels),
		.last_sample_count = (uint32_t)atomic_get(&haptics_last_sample_count),
		.last_play_err = (int)atomic_get(&haptics_last_play_err),
	};
	return 0;
}

int haptics_backend_set_master_gain_db(int16_t gain_db)
{
	atomic_set(&haptics_master_gain_db,
	           CLAMP((int32_t)gain_db, IBEX_HAPTICS_GAIN_DB_MIN, IBEX_HAPTICS_GAIN_DB_MAX));
	return 0;
}

int haptics_backend_set_amplifier_mode(bool forced_on)
{
	atomic_set(&haptics_amplifier_mode, forced_on ? 1 : 0);
	wake_output_channels(HAPTICS_CHANNELS_ALL);
	return 0;
}
