/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdl/controller_constants.h"

#define IBEX_SETTING_SDL_COUNT SETTING_COUNT
#define IBEX_SETTING_COUNT (IBEX_SETTING_SDL_COUNT + 1)

enum ibex_setting_id
{
	IBEX_SETTING_OLYMPUS_CLICK_PRESS = SETTING_TRACKPAD_OUTER_SPIN_VEL,
	IBEX_SETTING_OLYMPUS_CLICK_RELEASE = SETTING_TRACKPAD_OUTER_SPIN_RADIUS,
	IBEX_SETTING_IMU_MODE = SETTING_IMU_MODE,
	IBEX_SETTING_TRACKPAD_NOISE_THRESHOLD = SETTING_TRACKPAD_NOISE_THRESHOLD,
	IBEX_SETTING_LEFT_TRACKPAD_CLICK_PRESSURE = SETTING_LEFT_TRACKPAD_CLICK_PRESSURE,
	IBEX_SETTING_RIGHT_TRACKPAD_CLICK_PRESSURE = SETTING_RIGHT_TRACKPAD_CLICK_PRESSURE,
	IBEX_SETTING_TIMP_TOUCH_THRESHOLD_ON = SETTING_TIMP_TOUCH_THRESHOLD_ON,
	IBEX_SETTING_TIMP_TOUCH_THRESHOLD_OFF = SETTING_TIMP_TOUCH_THRESHOLD_OFF,
	IBEX_SETTING_OLYMPUS_CLICK_SUPPRESS_MASK = IBEX_SETTING_SDL_COUNT,
};

enum ibex_setting_kind
{
	IBEX_SETTING_CURRENT,
	IBEX_SETTING_DEFAULT,
	IBEX_SETTING_MIN,
	IBEX_SETTING_MAX,
};

typedef void (*ibex_setting_changed_cb_t)(uint8_t id, int16_t value);

void ibex_settings_registry_init(void);
void ibex_settings_registry_reset_defaults(void);

bool ibex_setting_get(uint8_t id, int16_t *value);
bool ibex_setting_get_meta(uint8_t id, enum ibex_setting_kind kind, int16_t *value);
const char *ibex_setting_name(uint8_t id);
const char *ibex_setting_name_source(uint8_t id);
const char *ibex_setting_persist_path(uint8_t id);
int ibex_setting_set(uint8_t id, int16_t value);
int ibex_settings_register_callback(ibex_setting_changed_cb_t callback);

bool ibex_settings_feature_response(const uint8_t *request, size_t request_len, uint8_t *response,
                                    size_t response_capacity, size_t *response_len);
bool ibex_settings_feature_write(const uint8_t *request, size_t request_len);
