/* SPDX-License-Identifier: AGPL-3.0-or-later */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "analog.h"
#include "battery.h"
#include "charge_mode.h"
#include "controller.h"
#include "power.h"
#include "puck_interface.h"
#include "rgbw_led.h"
#include "watchdog.h"

LOG_MODULE_REGISTER(charge_mode);

#define CHARGE_MODE_BOOT_WAIT_MS 1500
#define CHARGE_MODE_CHECK_MS 20
#define CHARGE_MODE_LED_STEP_MS 50
#define CHARGE_MODE_CONTACT_LOST_MS 2000
#define CHARGE_MODE_CONTACT_RESAMPLE_MS 500
#define CHARGE_MODE_STEAM_HOLD_MS 250
#define CHARGE_MODE_FULL_LED_MS 10000

static const struct rgbw_color led_off = { 0, 0, 0, 0 };
static const struct rgbw_color led_dim_amber = { 4, 5, 0, 0 };
static const struct rgbw_color led_peak_amber = { 26, 20, 0, 0 };
static bool charge_contact_latched;
static int64_t charge_contact_last_seen_ms;
static int64_t puck_pilot_last_sample_ms;

static bool steam_pressed(void)
{
	return (hardware_read_buttons() & BIT(CONTROLLER_BUTTON_STEAM)) != 0U;
}

static bool steam_power_requested(void)
{
	int64_t start;

	if(!steam_pressed())
	{
		return false;
	}

	start = k_uptime_get();
	while(k_uptime_get() - start < CHARGE_MODE_STEAM_HOLD_MS)
	{
		k_msleep(CHARGE_MODE_CHECK_MS);
		watchdog_feed();
		if(!steam_pressed())
		{
			return false;
		}
	}

	return true;
}

static bool puck_pilot_present(void)
{
	bool present;
	int err;

	if(!IS_ENABLED(CONFIG_IBEX_ANALOG_INPUTS))
	{
		return false;
	}

	err = analog_puck_pilot_present(&present);
	if(err)
	{
		LOG_DBG("puck pilot sample failed: %d", err);
		return false;
	}

	return present;
}

static bool puck_contact_present(void)
{
	return puck_interface_active() || puck_pilot_present();
}

static void charge_mode_note_contact(void)
{
	charge_contact_latched = true;
	charge_contact_last_seen_ms = k_uptime_get();
}

static bool charge_mode_live_contact_present(void)
{
	int64_t now;

	if(transport_usb_attached() || puck_interface_active())
	{
		return true;
	}

	now = k_uptime_get();
	if(now - puck_pilot_last_sample_ms < CHARGE_MODE_CONTACT_RESAMPLE_MS)
	{
		return false;
	}

	puck_pilot_last_sample_ms = now;
	return puck_contact_present();
}

static bool charge_mode_contact_present(void)
{
	if(charge_mode_live_contact_present())
	{
		charge_mode_note_contact();
		return true;
	}

	return charge_contact_latched &&
	       k_uptime_get() - charge_contact_last_seen_ms < CHARGE_MODE_CONTACT_LOST_MS;
}

static bool charge_full(void)
{
	struct controller_battery_report report;

	if(battery_read_fresh_status(&report))
	{
		return false;
	}

	return report.charge_complete;
}

static enum charge_mode_result charge_mode_check_exit(void)
{
	if(!charge_mode_contact_present())
	{
		LOG_INF("charger removed, powering off");
		power_off();
	}
	if(transport_usb_configured())
	{
		LOG_INF("USB configured, leaving charge-only mode");
		return CHARGE_MODE_USB_CONFIGURED;
	}
	if(steam_power_requested())
	{
		LOG_INF("Steam pressed, leaving charge-only mode");
		return CHARGE_MODE_POWER_ON_RADIO;
	}

	return CHARGE_MODE_SKIPPED;
}

static uint8_t lerp8(uint8_t from, uint8_t to, uint32_t num, uint32_t denom)
{
	uint32_t value;

	if(denom == 0U || num >= denom)
	{
		return to;
	}

	value = (uint32_t)from * (denom - num) + (uint32_t)to * num;
	return (uint8_t)(value / denom);
}

static void charge_led_set(const struct rgbw_color color)
{
#if CONFIG_IBEX_RGBW_LED
	rgbw_led_set(color);
#else
	ARG_UNUSED(color);
#endif
}

static enum charge_mode_result charge_mode_delay(uint32_t duration_ms)
{
	for(uint32_t elapsed = 0; elapsed < duration_ms; elapsed += CHARGE_MODE_CHECK_MS)
	{
		enum charge_mode_result result = charge_mode_check_exit();

		if(result != CHARGE_MODE_SKIPPED)
		{
			return result;
		}

		k_msleep(MIN(CHARGE_MODE_CHECK_MS, duration_ms - elapsed));
		watchdog_feed();
	}

	return CHARGE_MODE_SKIPPED;
}

static enum charge_mode_result charge_led_fade(const struct rgbw_color from,
                                               const struct rgbw_color to, uint32_t duration_ms)
{
	for(uint32_t elapsed = 0; elapsed <= duration_ms; elapsed += CHARGE_MODE_LED_STEP_MS)
	{
		struct rgbw_color color = {
			.r = lerp8(from.r, to.r, elapsed, duration_ms),
			.g = lerp8(from.g, to.g, elapsed, duration_ms),
			.b = lerp8(from.b, to.b, elapsed, duration_ms),
			.w = lerp8(from.w, to.w, elapsed, duration_ms),
		};
		enum charge_mode_result result;

		charge_led_set(color);
		result = charge_mode_delay(MIN(CHARGE_MODE_LED_STEP_MS, duration_ms - elapsed));
		if(result != CHARGE_MODE_SKIPPED)
		{
			return result;
		}
	}

	return CHARGE_MODE_SKIPPED;
}

static enum charge_mode_result charge_led_breathe_once(void)
{
#define DO_STEP(x) do { result = x; if(result != CHARGE_MODE_SKIPPED) { return result; } } while(0)
	enum charge_mode_result result;

	DO_STEP(charge_led_fade(led_off, led_dim_amber, 200));
	DO_STEP(charge_led_fade(led_dim_amber, led_peak_amber, 1600));
	DO_STEP(charge_led_fade(led_peak_amber, led_dim_amber, 1600));
	return charge_led_fade(led_dim_amber, led_off, 200);
#undef DO_STEP
}

static enum charge_mode_result wait_for_usb_or_steam(void)
{
	int64_t deadline = k_uptime_get() + CHARGE_MODE_BOOT_WAIT_MS;

	while(k_uptime_get() < deadline)
	{
		if(!transport_usb_attached())
		{
			return CHARGE_MODE_SKIPPED;
		}
		if(transport_usb_configured())
		{
			return CHARGE_MODE_USB_CONFIGURED;
		}
		k_msleep(CHARGE_MODE_CHECK_MS);
		watchdog_feed();
	}

	return steam_power_requested() ? CHARGE_MODE_POWER_ON_RADIO : CHARGE_MODE_SKIPPED;
}

enum charge_mode_result charge_mode_run_if_needed(void)
{
	int64_t charge_complete_since_ms = 0;
	enum charge_mode_result result;
	bool charge_complete = false;
	bool puck_contact;
	bool usb_attached;

	usb_attached = transport_usb_attached();
	puck_contact = puck_contact_present();
	if(!usb_attached && !puck_contact)
	{
		return CHARGE_MODE_SKIPPED;
	}
	charge_mode_note_contact();

	if(usb_attached && !puck_contact)
	{
		result = wait_for_usb_or_steam();
		if(result != CHARGE_MODE_SKIPPED)
		{
			return result;
		}
		if(!transport_usb_attached() || transport_usb_configured())
		{
			return transport_usb_configured() ? CHARGE_MODE_USB_CONFIGURED : CHARGE_MODE_SKIPPED;
		}
	}
	else if(steam_power_requested())
	{
		return CHARGE_MODE_POWER_ON_RADIO;
	}

	LOG_INF("%s; entering charge-only mode",
	        usb_attached ? "USB VBUS did not enumerate" : "puck contact detected");

	for(;;)
	{
		if(!charge_complete && charge_full())
		{
			charge_complete = true;
			charge_complete_since_ms = k_uptime_get();
		}

		if(charge_complete)
		{
			struct rgbw_color color = led_off;

			if(k_uptime_get() - charge_complete_since_ms < CHARGE_MODE_FULL_LED_MS)
			{
				color = led_peak_amber;
			}

			charge_led_set(color);
			result = charge_mode_delay(250);
		}
		else
		{
			result = charge_led_breathe_once();
		}

		if(result != CHARGE_MODE_SKIPPED)
		{
			charge_led_set(led_off);
			return result;
		}
	}
}
