/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ibex_settings_registry.h"

LOG_MODULE_REGISTER(ibex_settings);

#define IBEX_SETTINGS_MAX_CALLBACKS 8

struct ibex_setting_entry
{
	int16_t default_value;
	int16_t min_value;
	int16_t max_value;
	const char *name;
	const char *name_source;
	const char *persist_path;
};

/* Registry entries exposed through the Valve HID feature protocol. */
#define SETTING_ENTRY_WITH_SOURCE(default_, min_, max_, name_, source_, path_) \
	{ \
		.default_value = default_, \
		.min_value = min_, \
		.max_value = max_, \
		.name = name_, \
		.name_source = source_, \
		.persist_path = path_ \
	}
#define SETTING_ENTRY(default_, min_, max_, name_, path_) \
	SETTING_ENTRY_WITH_SOURCE(default_, min_, max_, name_, "sdl", path_)
#define OFW_SETTING_ENTRY(default_, min_, max_, name_, path_) \
	SETTING_ENTRY_WITH_SOURCE(default_, min_, max_, name_, "ofw", path_)
#define UNKNOWN_SETTING_ENTRY(default_, min_, max_, name_, path_) \
	SETTING_ENTRY_WITH_SOURCE(default_, min_, max_, name_, "unknown", path_)

static const struct ibex_setting_entry setting_entries[IBEX_SETTING_COUNT] = {
	[0] = SETTING_ENTRY(0, 0, 10, "mouse_sensitivity", NULL),
	[1] = SETTING_ENTRY(2, 0, 10, "mouse_acceleration", NULL),
	[2] = SETTING_ENTRY(0, 0, 360, "trackball_rotation_angle", NULL),
	[3] = SETTING_ENTRY(1200, 0, 25000, "haptic_intensity_unused", NULL),
	[4] = SETTING_ENTRY(0, 0, 1, "left_gamepad_stick_enabled", NULL),
	[5] = SETTING_ENTRY(0, 0, 1, "right_gamepad_stick_enabled", NULL),
	[6] = SETTING_ENTRY(0, 0, 255, "usb_debug_mode", NULL),
	[7] = SETTING_ENTRY(0, 0, 255, "left_trackpad_mode", NULL),
	[8] = SETTING_ENTRY(0, 0, 1, "right_trackpad_mode", NULL),
	[9] = OFW_SETTING_ENTRY(1, 0, 1, "lizard_mode", NULL),
	[10] = SETTING_ENTRY(7000, 0, 16384, "dpad_deadzone", NULL),
	[11] = SETTING_ENTRY(200, 0, 1000, "minimum_momentum_vel", NULL),
	[12] = SETTING_ENTRY(100, 1, 1000, "momentum_decay_amount", NULL),
	[13] = SETTING_ENTRY(50, 1, 500, "trackpad_relative_mode_ticks_per_pixel", NULL),
	[14] = SETTING_ENTRY(5500, 0, 32767, "haptic_increment", NULL),
	[15] = SETTING_ENTRY(923, 0, 2000, "dpad_angle_sin", NULL),
	[16] = SETTING_ENTRY(382, 0, 2000, "dpad_angle_cos", NULL),
	[17] = SETTING_ENTRY(2, 1, 10, "momentum_vertical_divisor", NULL),
	[18] = SETTING_ENTRY(8000, 0, 20000, "momentum_maximum_velocity", NULL),
	[19] = SETTING_ENTRY(1770, 0, 4096, "trackpad_z_on", NULL),
	[20] = SETTING_ENTRY(1630, 0, 4096, "trackpad_z_off", NULL),
	[21] = SETTING_ENTRY(5, 0, 500, "sensitivity_scale_amount", NULL),
	[22] = SETTING_ENTRY(2, 0, 8, "left_trackpad_secondary_mode", NULL),
	[23] = SETTING_ENTRY(2, 0, 8, "right_trackpad_secondary_mode", NULL),
	[24] = SETTING_ENTRY(20, 0, 30, "smooth_absolute_mouse", NULL),
	[25] = SETTING_ENTRY(40, 3, 99, "steambutton_poweroff_time", NULL),
	[26] = SETTING_ENTRY(0, 0, 1, "unused_1", NULL),
	[27] = OFW_SETTING_ENTRY(-10, -24, 12, "haptic_rumble_gain_db_offset", NULL),
	[28] = SETTING_ENTRY(16500, 0, 4096, "trackpad_z_on_left", NULL),
	[29] = SETTING_ENTRY(15000, 0, 4096, "trackpad_z_off_left", NULL),
	[30] = OFW_SETTING_ENTRY(500, 100, 1000, "olympus_click_press", NULL),
	[31] = OFW_SETTING_ENTRY(400, 0, 800, "olympus_click_release", NULL),
	[32] = SETTING_ENTRY(800, 100, 1200, "trackpad_outer_spin_horizontal_only", NULL),
	[33] = SETTING_ENTRY(0, 0, 1, "trackpad_relative_mode_deadzone", NULL),
	[34] = SETTING_ENTRY(100, 25, 400, "trackpad_relative_mode_max_vel", NULL),
	[35] = SETTING_ENTRY(80, 25, 100, "trackpad_relative_mode_invert_y", NULL),
	[36] = UNKNOWN_SETTING_ENTRY(0, 0, 100, "unknown_36", NULL),
	[37] = OFW_SETTING_ENTRY(0, 0, 1, "rgbw_led_override_enabled", NULL),
	[38] = UNKNOWN_SETTING_ENTRY(50, 1, 1000, "unknown_38", NULL),
	[39] = SETTING_ENTRY(0, 0, 1, "trackpad_outer_radius_release_on_transition", NULL),
	[40] = SETTING_ENTRY(20, 1, 180, "radial_mode_angle", NULL),
	[41] = SETTING_ENTRY(3900, 0, 25000, "haptic_intensity_mouse_mode", NULL),
	[42] = SETTING_ENTRY(1, 0, 1, "left_dpad_requires_click", NULL),
	[43] = SETTING_ENTRY(0, 0, 1, "right_dpad_requires_click", NULL),
	[44] = SETTING_ENTRY(50, 0, 100, "led_baseline_brightness", NULL),
	[45] = OFW_SETTING_ENTRY(50, 0, 100, "led_user_brightness", NULL),
	[46] = OFW_SETTING_ENTRY(0, 0, 2, "enable_raw_joystick", NULL),
	[47] = SETTING_ENTRY(0, 0, 16, "enable_fast_scan", NULL),
	[48] = OFW_SETTING_ENTRY(0, 0, 32767, "imu_mode", "settings/sensors/imu/mode"),
	[49] = SETTING_ENTRY(2, 1, 2, "wireless_packet_version", NULL),
	[50] = OFW_SETTING_ENTRY(900, 0, 32767, "sleep_inactivity_timeout", NULL),
	[51] = SETTING_ENTRY(250, 0, 32767, "trackpad_noise_threshold", NULL),
	[52] = SETTING_ENTRY(40, -1, 100, "left_trackpad_click_pressure", NULL),
	[53] = SETTING_ENTRY(40, -1, 100, "right_trackpad_click_pressure", NULL),
	[54] = SETTING_ENTRY(100, 0, 100, "left_bumper_click_pressure", NULL),
	[55] = SETTING_ENTRY(100, 0, 100, "right_bumper_click_pressure", NULL),
	[56] = SETTING_ENTRY(10, 0, 100, "left_grip_click_pressure", NULL),
	[57] = SETTING_ENTRY(10, 0, 100, "right_grip_click_pressure", NULL),
	[58] = SETTING_ENTRY(0, 0, 100, "left_grip2_click_pressure", NULL),
	[59] = SETTING_ENTRY(0, 0, 100, "right_grip2_click_pressure", NULL),
	[60] = SETTING_ENTRY(0, 0, 1, "pressure_mode", NULL),
	[61] = SETTING_ENTRY(0, 0, 15, "controller_test_mode", NULL),
	[62] = OFW_SETTING_ENTRY(0, 0, 2, "trigger_mode", NULL),
	[63] = SETTING_ENTRY(150, 0, 300, "trackpad_z_threshold", NULL),
	[64] = SETTING_ENTRY(4, 1, 16, "frame_rate", NULL),
	[65] = SETTING_ENTRY(1, 0, 1, "trackpad_filt_ctrl", NULL),
	[66] = SETTING_ENTRY(1, 0, 1, "trackpad_clip", NULL),
	[67] = SETTING_ENTRY(0, 0, 7, "debug_output_select", NULL),
	[68] = SETTING_ENTRY(90, 40, 99, "trigger_threshold_percent", NULL),
	[69] = SETTING_ENTRY(1, 0, 1, "trackpad_frequency_hopping", NULL),
	[70] = OFW_SETTING_ENTRY(1, 0, 2, "haptics_enabled", "settings/haptics/enabled"),
	[71] = OFW_SETTING_ENTRY(1, 0, 1, "steam_watchdog_enable", NULL),
	[72] = SETTING_ENTRY(1200, 0, 16000, "timp_touch_threshold_on", NULL),
	[73] = SETTING_ENTRY(1000, 0, 16000, "timp_touch_threshold_off", NULL),
	[74] = SETTING_ENTRY(3, 0, 3, "freq_hopping", NULL),
	[75] = OFW_SETTING_ENTRY(0, 0, 1, "haptic_amplifier_mode", "settings/haptics/amplifier_mode"),
	[76] = OFW_SETTING_ENTRY(-3, -24, 6, "haptic_master_gain_db",
	                         "settings/haptics/haptic_master_gain_db"),
	[77] = SETTING_ENTRY(0, 0, 1, "thumb_touch_thresh", NULL),
	[78] = SETTING_ENTRY(1, 0, 1, "device_power_status", NULL),
	[79] = SETTING_ENTRY(2, 1, 4, "haptic_intensity", NULL),
	[80] = SETTING_ENTRY(1, 0, 2, "stabilizer_enabled", NULL),
	[81] = SETTING_ENTRY(0, 0, 1, "timp_mode_mte", NULL),
	[82] = OFW_SETTING_ENTRY(3, 0, 3, "olympus_click_suppress_mask", NULL),
};

static int16_t setting_values[IBEX_SETTING_COUNT];
static ibex_setting_changed_cb_t callbacks[IBEX_SETTINGS_MAX_CALLBACKS];
static uint8_t callback_count;

static bool validate_id(uint8_t id)
{
	return id < IBEX_SETTING_COUNT;
}

static int16_t setting_meta_value(uint8_t id, enum ibex_setting_kind kind)
{
	switch(kind)
	{
		case IBEX_SETTING_DEFAULT:
			return setting_entries[id].default_value;
		case IBEX_SETTING_MIN:
			return setting_entries[id].min_value;
		case IBEX_SETTING_MAX:
			return setting_entries[id].max_value;
		case IBEX_SETTING_CURRENT:
		default:
			return setting_values[id];
	}
}

static void notify_changed(uint8_t id, int16_t value)
{
	for(uint8_t i = 0; i < callback_count; ++i)
	{
		callbacks[i](id, value);
	}
}

void ibex_settings_registry_init(void)
{
	ibex_settings_registry_reset_defaults();
}

void ibex_settings_registry_reset_defaults(void)
{
	for(uint8_t i = 0; i < IBEX_SETTING_COUNT; ++i)
	{
		if(setting_values[i] != setting_entries[i].default_value)
		{
			setting_values[i] = setting_entries[i].default_value;
			notify_changed(i, setting_values[i]);
		}
		else
		{
			setting_values[i] = setting_entries[i].default_value;
		}
	}
}

bool ibex_setting_get(uint8_t id, int16_t *value)
{
	if(!validate_id(id))
	{
		return false;
	}
	*value = setting_values[id];
	return true;
}

bool ibex_setting_get_meta(uint8_t id, enum ibex_setting_kind kind, int16_t *value)
{
	if(!validate_id(id))
	{
		return false;
	}
	*value = setting_meta_value(id, kind);
	return true;
}

const char *ibex_setting_name(uint8_t id)
{
	if(!validate_id(id))
	{
		return NULL;
	}
	return setting_entries[id].name;
}

const char *ibex_setting_name_source(uint8_t id)
{
	if(!validate_id(id))
	{
		return NULL;
	}
	return setting_entries[id].name_source;
}

const char *ibex_setting_persist_path(uint8_t id)
{
	if(!validate_id(id))
	{
		return NULL;
	}
	return setting_entries[id].persist_path;
}

int ibex_setting_set(uint8_t id, int16_t value)
{
	int16_t clamped;

	if(!validate_id(id))
	{
		return -EINVAL;
	}

	clamped = CLAMP(value, setting_entries[id].min_value, setting_entries[id].max_value);
	if(setting_values[id] != clamped)
	{
		setting_values[id] = clamped;
		notify_changed(id, clamped);
	}
	return 0;
}

int ibex_settings_register_callback(ibex_setting_changed_cb_t callback)
{
	if(callback_count >= ARRAY_SIZE(callbacks))
	{
		return -ENOMEM;
	}
	callbacks[callback_count++] = callback;
	for(uint8_t i = 0; i < IBEX_SETTING_COUNT; ++i)
	{
		callback(i, setting_values[i]);
	}
	return 0;
}

static bool request_body(const uint8_t *request, size_t request_len, const uint8_t **body,
                         size_t *body_len)
{
	if(request_len < 2 || request[1] + 2U > request_len)
	{
		return false;
	}
	*body = &request[2];
	*body_len = request[1];
	return true;
}

static bool append_setting_triplet(uint8_t id, enum ibex_setting_kind kind, uint8_t **cursor,
                                   uint8_t *end)
{
	int16_t value;

	if(*cursor + 3 > end || !ibex_setting_get_meta(id, kind, &value))
	{
		return false;
	}

	*(*cursor)++ = id;
	sys_put_le16((uint16_t)value, *cursor);
	*cursor += sizeof(uint16_t);
	return true;
}

static bool append_requested_values(const uint8_t *body, size_t body_len,
                                    enum ibex_setting_kind kind, uint8_t *response,
                                    size_t response_capacity, size_t *response_len)
{
	uint8_t *cursor = response;
	uint8_t *end = response + response_capacity;

	if(body_len == 0)
	{
		for(uint8_t id = 0; id < IBEX_SETTING_COUNT; ++id)
		{
			if(!append_setting_triplet(id, kind, &cursor, end))
			{
				break;
			}
		}
	}
	else
	{
		for(size_t i = 0; i < body_len; ++i)
		{
			(void)append_setting_triplet(body[i], kind, &cursor, end);
		}
	}

	*response_len = cursor - response;
	return true;
}

static bool append_requested_kind(const uint8_t *request, size_t request_len,
                                  enum ibex_setting_kind kind, uint8_t *response,
                                  size_t response_capacity, size_t *response_len)
{
	const uint8_t *body;
	size_t body_len;

	if(!request_body(request, request_len, &body, &body_len))
	{
		return false;
	}

	return append_requested_values(body, body_len, kind, response, response_capacity, response_len);
}

bool ibex_settings_feature_response(const uint8_t *request, size_t request_len, uint8_t *response,
                                    size_t response_capacity, size_t *response_len)
{
	*response_len = 0;
	if(request_len == 0)
	{
		return false;
	}

	switch(request[0])
	{
		case ID_GET_SETTINGS_VALUES:
			return append_requested_kind(request, request_len, IBEX_SETTING_CURRENT, response,
			                             response_capacity, response_len);
		case ID_GET_SETTINGS_DEFAULTS:
			return append_requested_kind(request, request_len, IBEX_SETTING_DEFAULT, response,
			                             response_capacity, response_len);
		case ID_GET_SETTINGS_MAXS:
			return append_requested_kind(request, request_len, IBEX_SETTING_MAX, response,
			                             response_capacity, response_len);
		case ID_LOAD_DEFAULT_SETTINGS:
		case ID_CLEAR_SETTINGS_VALUES:
			ibex_settings_registry_reset_defaults();
			return true;
		default:
			return false;
	}
}

bool ibex_settings_feature_write(const uint8_t *request, size_t request_len)
{
	const uint8_t *body;
	size_t body_len;

	if(request_len == 0)
	{
		return false;
	}

	switch(request[0])
	{
		case ID_SET_SETTINGS_VALUES:
			if(!request_body(request, request_len, &body, &body_len))
			{
				return false;
			}
			for(size_t offset = 0; offset + 3 <= body_len; offset += 3)
			{
				uint8_t id = body[offset];
				int16_t value = (int16_t)sys_get_le16(&body[offset + 1]);

				(void)ibex_setting_set(id, value);
			}
			return true;
		case ID_LOAD_DEFAULT_SETTINGS:
		case ID_CLEAR_SETTINGS_VALUES:
			ibex_settings_registry_reset_defaults();
			return true;
		default:
			return false;
	}
}
