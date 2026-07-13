/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "controller.h"
#include "ibex_settings_registry.h"
#include "lizard.h"

LOG_MODULE_REGISTER(lizard);

#define LIZARD_MOUSE_REPORT_ID 0x40
#define LIZARD_KEYBOARD_REPORT_ID 0x41
#define LIZARD_MOUSE_REPORT_SIZE 5
#define LIZARD_KEYBOARD_REPORT_SIZE 8
#define LIZARD_KEY_SLOTS 6
#define LIZARD_TRACKPAD_HISTORY 20
#define LIZARD_TRACKPAD_RECENT 8
#define LIZARD_TRACKPAD_RELEASE_SKIP 5
#define LIZARD_TRACKPAD_MIN_TRAVEL 700.0f
#define LIZARD_RIGHT_PAD_DIVISOR 128
#define LIZARD_LEFT_PAD_DIVISOR 1024
#define LIZARD_STEAM_WATCHDOG_MS 10000

struct lizard_ring
{
	int32_t samples[LIZARD_TRACKPAD_HISTORY];
	uint8_t count;
	uint8_t write_index;
};

struct lizard_trackpad_state
{
	int32_t velocity_x;
	int32_t velocity_y;
	float momentum;
	float travel;
	struct lizard_ring x_history;
	struct lizard_ring y_history;
	int32_t remainder_x;
	int32_t remainder_y;
	int32_t previous_x;
	int32_t previous_y;
	bool previous_active;
};

static const uint8_t default_key_mappings[30] = {
	[CONTROLLER_BUTTON_A] = 0x28,          /* Enter */
	[CONTROLLER_BUTTON_B] = 0x29,          /* Escape */
	[CONTROLLER_BUTTON_VIEW] = 0x29,       /* Escape */
	[CONTROLLER_BUTTON_DPAD_DOWN] = 0x51,  /* Down */
	[CONTROLLER_BUTTON_DPAD_RIGHT] = 0x4f, /* Right */
	[CONTROLLER_BUTTON_DPAD_LEFT] = 0x50,  /* Left */
	[CONTROLLER_BUTTON_DPAD_UP] = 0x52,    /* Up */
	[CONTROLLER_BUTTON_MENU] = 0x2b,       /* Tab */
};

static K_MUTEX_DEFINE(lizard_mutex);
static bool lizard_initialized;
static bool lizard_mode = true;
static bool mappings_loaded = true;
static bool steam_watchdog_enabled = true;
static bool mouse_idle = true;
static bool keyboard_report_valid;
static bool mouse_release_pending;
static bool keyboard_release_pending;
static int16_t minimum_momentum_velocity = 200;
static int16_t momentum_decay_amount = 100;
static int16_t click_suppress_mask = 3;
static uint8_t last_keyboard_report[LIZARD_KEYBOARD_REPORT_SIZE];
static struct lizard_trackpad_state trackpads[2];
static atomic_t transport_reset_pending;

static void steam_watchdog_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(steam_watchdog_work, steam_watchdog_work_handler);

static void reset_trackpad(struct lizard_trackpad_state *state)
{
	memset(state, 0, sizeof(*state));
}

static void ring_reset(struct lizard_ring *ring)
{
	ring->count = 0;
	ring->write_index = 0;
}

static void ring_push(struct lizard_ring *ring, int32_t sample)
{
	if(ring->count < ARRAY_SIZE(ring->samples))
	{
		ring->samples[ring->count++] = sample;
		return;
	}

	ring->samples[ring->write_index++] = sample;
	if(ring->write_index == ARRAY_SIZE(ring->samples))
	{
		ring->write_index = 0;
	}
}

static int32_t ring_mean_recent(const struct lizard_ring *ring, uint8_t requested)
{
	uint8_t count = MIN(requested, ring->count);
	uint8_t index;
	int32_t sum = 0;

	if(count == 0)
	{
		return 0;
	}

	index = ring->write_index + ring->count - count;
	if(index >= ARRAY_SIZE(ring->samples))
	{
		index -= ARRAY_SIZE(ring->samples);
	}
	for(uint8_t i = 0; i < count; ++i)
	{
		sum += ring->samples[index++];
		if(index == ARRAY_SIZE(ring->samples))
		{
			index = 0;
		}
	}
	return sum / count;
}

static int32_t ring_mean_excluding_recent(const struct lizard_ring *ring, uint8_t skip_recent)
{
	const uint8_t count = ARRAY_SIZE(ring->samples) - skip_recent;
	uint8_t index = ring->write_index;
	int32_t sum = 0;

	if(ring->count < ARRAY_SIZE(ring->samples))
	{
		return 0;
	}
	for(uint8_t i = 0; i < count; ++i)
	{
		sum += ring->samples[index++];
		if(index == ARRAY_SIZE(ring->samples))
		{
			index = 0;
		}
	}
	return sum / count;
}

static int32_t trackpad_emit_delta(int32_t movement, int32_t divisor, int32_t *remainder)
{
	int64_t total = (int64_t)movement + *remainder;
	int32_t output = CLAMP(total / divisor, -127, 127);
	int64_t excess = total - (int64_t)output * divisor;

	*remainder = CLAMP(excess, INT32_MIN, INT32_MAX);
	return output;
}

static void trackpad_update(struct lizard_trackpad_state *state, bool active, int32_t x, int32_t y,
                            int32_t divisor, int32_t *out_x, int32_t *out_y)
{
	int32_t movement_x = 0;
	int32_t movement_y = 0;

	if(active)
	{
		if(state->previous_active)
		{
			int32_t delta_x = x - state->previous_x;
			int32_t delta_y = y - state->previous_y;

			state->travel +=
			    sqrtf((float)delta_x * (float)delta_x + (float)delta_y * (float)delta_y);
			ring_push(&state->x_history, delta_x);
			ring_push(&state->y_history, delta_y);
			movement_x = ring_mean_recent(&state->x_history, LIZARD_TRACKPAD_RECENT);
			movement_y = ring_mean_recent(&state->y_history, LIZARD_TRACKPAD_RECENT);
		}
		else
		{
			ring_reset(&state->x_history);
			ring_reset(&state->y_history);
			state->travel = 0.0f;
		}
	}
	else
	{
		if(state->previous_active)
		{
			float speed;

			/* OFW estimates release velocity from the oldest 15 of exactly 20 deltas. */
			state->velocity_x =
			    ring_mean_excluding_recent(&state->x_history, LIZARD_TRACKPAD_RELEASE_SKIP);
			state->velocity_y =
			    ring_mean_excluding_recent(&state->y_history, LIZARD_TRACKPAD_RELEASE_SKIP);
			speed = sqrtf((float)state->velocity_x * (float)state->velocity_x +
			              (float)state->velocity_y * (float)state->velocity_y);
			if(state->travel > LIZARD_TRACKPAD_MIN_TRAVEL &&
			   (int32_t)speed > minimum_momentum_velocity)
			{
				state->momentum = 1.0f;
			}
			else
			{
				state->momentum = 0.0f;
			}
		}

		if(state->momentum > 0.0f)
		{
			movement_x = (int32_t)((float)state->velocity_x * state->momentum);
			movement_y = (int32_t)((float)state->velocity_y * state->momentum);
			state->momentum -= 1.0f / (float)MAX(momentum_decay_amount, 1);
		}
	}

	*out_x = trackpad_emit_delta(movement_x, divisor, &state->remainder_x);
	*out_y = trackpad_emit_delta(movement_y, divisor, &state->remainder_y);
	state->previous_x = x;
	state->previous_y = y;
	state->previous_active = active;
}

static void build_keyboard_report(uint32_t buttons, uint8_t report[LIZARD_KEYBOARD_REPORT_SIZE])
{
	uint8_t key_count = 0;

	memset(report, 0, LIZARD_KEYBOARD_REPORT_SIZE);
	for(uint8_t button = 0; button < ARRAY_SIZE(default_key_mappings); ++button)
	{
		if(default_key_mappings[button] == 0 || (buttons & BIT(button)) == 0)
		{
			continue;
		}
		if(key_count == LIZARD_KEY_SLOTS)
		{
			memset(&report[2], 1, LIZARD_KEY_SLOTS);
			return;
		}
		report[2 + key_count++] = default_key_mappings[button];
	}
}

static bool report_has_data(const uint8_t *report, size_t len)
{
	for(size_t i = 0; i < len; ++i)
	{
		if(report[i] != 0)
		{
			return true;
		}
	}
	return false;
}

static bool send_pending_release_reports_locked(void)
{
	static const uint8_t mouse_release[LIZARD_MOUSE_REPORT_SIZE];
	static const uint8_t keyboard_release[LIZARD_KEYBOARD_REPORT_SIZE];

	if(mouse_release_pending && transport_send_input_report(LIZARD_MOUSE_REPORT_ID, mouse_release,
	                                                        sizeof(mouse_release)) == 0)
	{
		mouse_release_pending = false;
		mouse_idle = true;
	}
	if(keyboard_release_pending &&
	   transport_send_input_report(LIZARD_KEYBOARD_REPORT_ID, keyboard_release,
	                               sizeof(keyboard_release)) == 0)
	{
		keyboard_release_pending = false;
		keyboard_report_valid = true;
		memset(last_keyboard_report, 0, sizeof(last_keyboard_report));
	}

	return mouse_release_pending || keyboard_release_pending;
}

static void send_release_reports(void)
{
	bool pending;

	k_mutex_lock(&lizard_mutex, K_FOREVER);
	mouse_release_pending = true;
	keyboard_release_pending = true;
	mouse_idle = false;
	keyboard_report_valid = false;
	pending = send_pending_release_reports_locked();
	k_mutex_unlock(&lizard_mutex);

	/* Wake one immediate retry without turning persistent disconnects into a busy loop. */
	if(pending)
	{
		hardware_signal_input_changed();
	}
}

static void schedule_steam_watchdog(void)
{
	if(steam_watchdog_enabled)
	{
		(void)k_work_reschedule(&steam_watchdog_work, K_MSEC(LIZARD_STEAM_WATCHDOG_MS));
	}
}

static void lizard_setting_written(uint8_t id, int16_t value)
{
	if(id == SETTING_LIZARD_MODE && value == 0)
	{
		schedule_steam_watchdog();
	}
}

static void lizard_setting_changed(uint8_t id, int16_t value)
{
	bool release = false;
	bool schedule = false;
	bool cancel = false;

	k_mutex_lock(&lizard_mutex, K_FOREVER);
	switch(id)
	{
		case SETTING_LIZARD_MODE:
			release = lizard_initialized && lizard_mode && mappings_loaded;
			lizard_mode = value != 0;
			mappings_loaded = lizard_mode;
			reset_trackpad(&trackpads[0]);
			reset_trackpad(&trackpads[1]);
			if(lizard_mode)
			{
				cancel = true;
			}
			break;
		case SETTING_MINIMUM_MOMENTUM_VEL:
			minimum_momentum_velocity = value;
			break;
		case SETTING_MOMENTUM_DECAY_AMOUNT:
			momentum_decay_amount = MAX(value, 1);
			break;
		case SETTING_STEAM_WATCHDOG_ENABLE:
			steam_watchdog_enabled = value != 0;
			if(!steam_watchdog_enabled)
			{
				cancel = true;
			}
			else if(!lizard_mode)
			{
				schedule = true;
			}
			break;
		case IBEX_SETTING_OLYMPUS_CLICK_SUPPRESS_MASK:
			click_suppress_mask = value;
			break;
		default:
			break;
	}
	k_mutex_unlock(&lizard_mutex);

	if(release)
	{
		send_release_reports();
	}
	if(cancel)
	{
		(void)k_work_cancel_delayable(&steam_watchdog_work);
	}
	if(schedule)
	{
		schedule_steam_watchdog();
	}
}

static void steam_watchdog_work_handler(struct k_work *work)
{
	int16_t mode;

	ARG_UNUSED(work);
	if(!ibex_setting_get(SETTING_LIZARD_MODE, &mode) || mode != 0)
	{
		return;
	}

	LOG_INF("Steam watchdog restoring default digital mappings");
	lizard_set_default_digital_mappings();
	ibex_settings_registry_reset_defaults();
}

int lizard_init(void)
{
	int err;

	err = ibex_settings_register_callback(lizard_setting_changed);
	if(err)
	{
		return err;
	}
	err = ibex_settings_register_set_callback(lizard_setting_written);
	if(err)
	{
		return err;
	}

	k_mutex_lock(&lizard_mutex, K_FOREVER);
	lizard_initialized = true;
	k_mutex_unlock(&lizard_mutex);
	send_release_reports();
	return 0;
}

void lizard_clear_digital_mappings(void)
{
	bool release;

	k_mutex_lock(&lizard_mutex, K_FOREVER);
	release = lizard_mode && mappings_loaded;
	mappings_loaded = false;
	reset_trackpad(&trackpads[0]);
	reset_trackpad(&trackpads[1]);
	k_mutex_unlock(&lizard_mutex);

	if(release)
	{
		send_release_reports();
	}
	schedule_steam_watchdog();
	(void)ibex_setting_set(SETTING_LIZARD_MODE, 0);
}

void lizard_set_default_digital_mappings(void)
{
	bool active;

	k_mutex_lock(&lizard_mutex, K_FOREVER);
	mappings_loaded = true;
	active = lizard_mode;
	reset_trackpad(&trackpads[0]);
	reset_trackpad(&trackpads[1]);
	k_mutex_unlock(&lizard_mutex);

	if(active)
	{
		send_release_reports();
	}
}

void lizard_update(const struct controller_report *report)
{
	uint8_t keyboard_report[LIZARD_KEYBOARD_REPORT_SIZE];
	uint8_t mouse_report[LIZARD_MOUSE_REPORT_SIZE] = { 0 };
	int32_t right_x, right_y, left_x, left_y;
	bool right_touched;
	bool left_touched;
	bool send_keyboard;
	bool send_mouse;

	k_mutex_lock(&lizard_mutex, K_FOREVER);
	if(atomic_cas(&transport_reset_pending, 1, 0))
	{
		keyboard_report_valid = false;
		mouse_idle = false;
		reset_trackpad(&trackpads[0]);
		reset_trackpad(&trackpads[1]);
	}
	if(send_pending_release_reports_locked())
	{
		k_mutex_unlock(&lizard_mutex);
		return;
	}
	if(!lizard_mode || !mappings_loaded)
	{
		k_mutex_unlock(&lizard_mutex);
		return;
	}

	build_keyboard_report(report->buttons, keyboard_report);
	send_keyboard = !keyboard_report_valid ||
	                memcmp(keyboard_report, last_keyboard_report, sizeof(keyboard_report)) != 0;

	right_touched = (report->buttons & BIT(CONTROLLER_BUTTON_RIGHT_TOUCHPAD_TOUCH)) != 0;
	left_touched = (report->buttons & BIT(CONTROLLER_BUTTON_LEFT_TOUCHPAD_TOUCH)) != 0;
	trackpad_update(&trackpads[1], right_touched, report->touchpad_right_x,
	                report->touchpad_right_y, LIZARD_RIGHT_PAD_DIVISOR, &right_x, &right_y);
	trackpad_update(&trackpads[0], left_touched, report->touchpad_left_x, report->touchpad_left_y,
	                LIZARD_LEFT_PAD_DIVISOR, &left_x, &left_y);

	if((report->buttons & BIT(CONTROLLER_BUTTON_RIGHT_STICK_TOUCH)) != 0 &&
	   (click_suppress_mask & BIT(1)) != 0)
	{
		right_x = 0;
		right_y = 0;
	}
	if((report->buttons & BIT(CONTROLLER_BUTTON_LEFT_STICK_TOUCH)) != 0 &&
	   (click_suppress_mask & BIT(0)) != 0)
	{
		left_x = 0;
		left_y = 0;
	}

	if((report->buttons & (BIT(CONTROLLER_BUTTON_RIGHT_TRIGGER_CLICK) |
	                       BIT(CONTROLLER_BUTTON_RIGHT_TOUCHPAD_CLICK))) != 0)
	{
		mouse_report[0] |= BIT(0);
	}
	if((report->buttons & BIT(CONTROLLER_BUTTON_LEFT_TRIGGER_CLICK)) != 0)
	{
		mouse_report[0] |= BIT(1);
	}
	/* Report 0x40 is buttons, cursor X/Y, vertical wheel, then horizontal pan. */
	mouse_report[1] = (uint8_t)(int8_t)right_x;
	mouse_report[2] = (uint8_t)(int8_t)-right_y;
	mouse_report[3] = (uint8_t)(int8_t)-left_y;
	mouse_report[4] = (uint8_t)(int8_t)left_x;
	send_mouse = !mouse_idle || report_has_data(mouse_report, sizeof(mouse_report));

	if(send_mouse &&
	   transport_send_input_report(LIZARD_MOUSE_REPORT_ID, mouse_report, sizeof(mouse_report)) == 0)
	{
		mouse_idle = !report_has_data(mouse_report, sizeof(mouse_report));
	}
	if(send_keyboard && transport_send_input_report(LIZARD_KEYBOARD_REPORT_ID, keyboard_report,
	                                                sizeof(keyboard_report)) == 0)
	{
		keyboard_report_valid = true;
		memcpy(last_keyboard_report, keyboard_report, sizeof(last_keyboard_report));
	}
	k_mutex_unlock(&lizard_mutex);
}

void lizard_transport_reset(void)
{
	atomic_set(&transport_reset_pending, 1);
	hardware_signal_input_changed();
}
