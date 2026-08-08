/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#include "haptics.h"
#include "ibex_settings_registry.h"

#define STEAM_HAPTICS_PLAYER_PCM_STEREO_REPORT_ID 0x88U
#define STEAM_HAPTICS_PLAYER_REPORT_BYTES 64U
#define STEAM_HAPTICS_PLAYER_PCM_CHANNEL_BYTES 31U
#define BACKEND_EFFECT_TIMEOUT K_MSEC(100)

K_MSGQ_DEFINE(observed_backend_effects, sizeof(struct haptics_effect), 4, 4);

bool ibex_setting_get(uint8_t id, int16_t *value)
{
	ARG_UNUSED(id);

	if(value == NULL)
	{
		return false;
	}
	*value = 1;
	return true;
}

int ibex_settings_register_callback(ibex_setting_changed_cb_t callback)
{
	ARG_UNUSED(callback);
	return 0;
}

int haptics_backend_effect(const struct haptics_effect *effect)
{
	return k_msgq_put(&observed_backend_effects, effect, K_NO_WAIT);
}

static void drain_observed_effects(void)
{
	struct haptics_effect effect;

	while(k_msgq_get(&observed_backend_effects, &effect, K_NO_WAIT) == 0)
	{
	}
}

static void *haptics_stereo_setup(void)
{
	zassert_ok(haptics_init());
	return NULL;
}

static void haptics_stereo_before(void *fixture)
{
	ARG_UNUSED(fixture);
	drain_observed_effects();
}

static struct haptics_effect receive_only_backend_effect(void)
{
	struct haptics_effect effect;
	struct haptics_effect extra_effect;

	zassert_ok(k_msgq_get(&observed_backend_effects, &effect, BACKEND_EFFECT_TIMEOUT),
	           "haptics backend did not receive the stereo effect");
	zassert_not_equal(k_msgq_get(&observed_backend_effects, &extra_effect, K_MSEC(10)), 0,
	                  "one stereo report produced multiple backend effects");
	return effect;
}

static void assert_stereo_effect(const struct haptics_effect *effect, const uint8_t *left,
                                 const uint8_t *right, uint8_t sample_count)
{
	zassert_equal(effect->type, HAPTICS_EFFECT_PCM_STEREO);
	zassert_equal(effect->channels, HAPTICS_CHANNEL_LEFT_1 | HAPTICS_CHANNEL_RIGHT_1);
	zassert_equal(effect->sample_count, sample_count);
	zassert_mem_equal(effect->samples, left, sample_count);
	zassert_mem_equal(&effect->samples[HAPTICS_PCM_STEREO_MAX_SAMPLES], right, sample_count);
}

ZTEST(haptics_stereo, test_wired_player_report_is_one_atomic_stereo_effect)
{
	uint8_t report[STEAM_HAPTICS_PLAYER_REPORT_BYTES];
	struct haptics_effect effect;

	memset(report, 0xa5, sizeof(report));
	report[0] = STEAM_HAPTICS_PLAYER_PCM_STEREO_REPORT_ID;
	report[1] = 30U;
	for(uint8_t i = 0; i < 30U; ++i)
	{
		report[2U + i] = (uint8_t)(0x10U + i);
		report[33U + i] = (uint8_t)(0x80U + i);
	}

	zassert_ok(haptics_handle_output_report(0, report, sizeof(report)));
	effect = receive_only_backend_effect();
	assert_stereo_effect(&effect, &report[2], &report[33], 30U);
	zassert_equal(effect.samples[30], 0U, "unused left slot copied packet padding");
	zassert_equal(effect.samples[61], 0U, "unused right slot copied packet padding");
}

ZTEST(haptics_stereo, test_puck_player_report_preserves_full_ulaw_channels)
{
	uint8_t report[STEAM_HAPTICS_PLAYER_REPORT_BYTES] = {
		STEAM_HAPTICS_PLAYER_PCM_STEREO_REPORT_ID,
		STEAM_HAPTICS_PLAYER_PCM_CHANNEL_BYTES,
	};
	struct haptics_effect effect;

	for(uint8_t i = 0; i < STEAM_HAPTICS_PLAYER_PCM_CHANNEL_BYTES; ++i)
	{
		report[2U + i] = (uint8_t)(3U * i + 1U);
		report[33U + i] = (uint8_t)(0xffU - 5U * i);
	}

	zassert_ok(haptics_handle_output_report(0, report, sizeof(report)));
	effect = receive_only_backend_effect();
	assert_stereo_effect(&effect, &report[2], &report[33], STEAM_HAPTICS_PLAYER_PCM_CHANNEL_BYTES);
}

ZTEST_SUITE(haptics_stereo, NULL, haptics_stereo_setup, haptics_stereo_before, NULL, NULL);
