/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "battery.h"
#include "charge_mode.h"
#include "controller.h"
#include "ibex_settings_registry.h"
#include "lizard.h"
#include "power.h"
#include "puck_interface.h"
#include "valve_settings.h"
#include "watchdog.h"

#if CONFIG_IBEX_RGBW_LED
#include "rgbw_led.h"
#endif

#define BOOT_CHORD_SETTLE_MS 150
#define PERSONALITY_PERSIST_DELAY_MS 60000
#define STEAM_POWER_HOLD_MS 5000
#define INPUT_REFRESH_MS 15
#define INACTIVITY_STICK_THRESHOLD 4000
#define INACTIVITY_PHYSICAL_BUTTONS BIT_MASK(20)
#define INACTIVITY_TOUCH_BUTTONS \
	(BIT(CONTROLLER_BUTTON_RIGHT_STICK_TOUCH) | \
	 BIT(CONTROLLER_BUTTON_RIGHT_TOUCHPAD_TOUCH) | \
	 BIT(CONTROLLER_BUTTON_LEFT_STICK_TOUCH) | BIT(CONTROLLER_BUTTON_LEFT_TOUCHPAD_TOUCH))

LOG_MODULE_REGISTER(app);

static bool chord_pressed(uint32_t buttons, enum controller_button first,
                          enum controller_button second)
{
	uint32_t mask = BIT(first) | BIT(second);

	return (buttons & mask) == mask;
}

static bool report_has_activity(const struct controller_report *report)
{
	if((report->buttons & (INACTIVITY_PHYSICAL_BUTTONS | INACTIVITY_TOUCH_BUTTONS)) != 0U ||
	   report->trigger_left != 0 ||
	   report->trigger_right != 0)
	{
		return true;
	}

	return abs(report->stick_left_x) > INACTIVITY_STICK_THRESHOLD ||
	       abs(report->stick_left_y) > INACTIVITY_STICK_THRESHOLD ||
	       abs(report->stick_right_x) > INACTIVITY_STICK_THRESHOLD ||
	       abs(report->stick_right_y) > INACTIVITY_STICK_THRESHOLD;
}

static enum radio_personality boot_chord_personality(void)
{
	uint32_t buttons;

	k_sleep(K_MSEC(BOOT_CHORD_SETTLE_MS));
	buttons = hardware_read_buttons();

	/* BLE wins the three-button chord so A+B also opens its pairing window. */
	if(chord_pressed(buttons, CONTROLLER_BUTTON_RIGHT_SHOULDER, CONTROLLER_BUTTON_B))
	{
		return RADIO_PERSONALITY_BLE;
	}
	if(chord_pressed(buttons, CONTROLLER_BUTTON_RIGHT_SHOULDER, CONTROLLER_BUTTON_A))
	{
		return RADIO_PERSONALITY_ESB;
	}

	return radio_personality_get();
}

static void set_boot_led(bool usb_radio_off)
{
#if CONFIG_IBEX_RGBW_LED
	if(usb_radio_off)
	{
		rgbw_led_set((struct rgbw_color){ 0, 255, 0, 0 });
	}
	else if(radio_personality_get() == RADIO_PERSONALITY_BLE)
	{
		rgbw_led_set((struct rgbw_color){ 0, 0, 255, 0 });
	}
	else
	{
		rgbw_led_set((struct rgbw_color){ 0, 0, 0, 255 });
	}
#endif
}

int main(void)
{
	struct controller_report previous = { 0 };
	int64_t steam_button_since = 0;
	int64_t last_input_sent = 0;
	int64_t last_activity_ms;
	int16_t inactivity_timeout_seconds = 0;
	enum charge_mode_result charge_result = CHARGE_MODE_SKIPPED;
	bool usb_vbus_charge_radio;
	bool first = true;
	int err;

	power_capture_boot_state();

	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		err = settings_subsys_init();
		if(err)
		{
			LOG_ERR("settings initialization failed: %d", err);
			return err;
		}
	}
	radio_personality_init();
	valve_settings_load_feature_state();
	ibex_settings_registry_init();
	err = lizard_init();
	if(err)
	{
		LOG_ERR("lizard mode initialization failed: %d", err);
		return err;
	}

	err = hardware_init();
	if(err)
	{
		LOG_ERR("hardware initialization failed: %d", err);
		return err;
	}
	err = puck_interface_init();
	if(err)
	{
		LOG_WRN("puck interface initialization failed: %d", err);
	}

	if(IS_ENABLED(CONFIG_IBEX_BATTERY))
	{
		err = battery_init();
		if(err)
		{
			LOG_WRN("battery initialization failed: %d", err);
		}
	}

	if(IS_ENABLED(CONFIG_IBEX_USB_HID))
	{
		err = transport_usb_init();
		if(err)
		{
			LOG_ERR("USB initialization failed: %d", err);
			return err;
		}
	}
	charge_result = charge_mode_run_if_needed();
	usb_vbus_charge_radio = charge_result == CHARGE_MODE_POWER_ON_RADIO &&
	                        transport_usb_attached() &&
	                        !transport_usb_configured();

	if(transport_usb_attached() && !usb_vbus_charge_radio)
	{
		LOG_INF("USB attached; using radio-off USB mode without persisting personality");
	}
	else
	{
		enum radio_personality requested_personality = boot_chord_personality();

		if(requested_personality != radio_personality_get())
		{
			LOG_INF("boot chord selected %s personality",
			        requested_personality == RADIO_PERSONALITY_ESB ? "ESB" : "BLE");
			radio_personality_reboot_into(requested_personality);
		}
		radio_personality_persist_after(radio_personality_get(), PERSONALITY_PERSIST_DELAY_MS);
	}
	set_boot_led(transport_usb_attached() && !usb_vbus_charge_radio);

	if(usb_vbus_charge_radio)
	{
		err = transport_set_usb_radio_mode(TRANSPORT_USB_RADIO_VBUS_CHARGE);
	}
	else
	{
		err = transport_init();
	}
	if(err)
	{
		LOG_ERR("transport initialization failed: %d", err);
		return err;
	}

	LOG_INF("SC26re started in %s mode", radio_personality_name());
	(void)ibex_setting_get(SETTING_SLEEP_INACTIVITY_TIMEOUT, &inactivity_timeout_seconds);
	last_activity_ms = k_uptime_get();

	for(;;)
	{
		struct controller_report current;
		int16_t timeout_seconds = inactivity_timeout_seconds;
		int64_t now_ms;

		hardware_read_report(&current);
		now_ms = k_uptime_get();
		if(ibex_setting_get(SETTING_SLEEP_INACTIVITY_TIMEOUT, &timeout_seconds) &&
		   timeout_seconds != inactivity_timeout_seconds)
		{
			inactivity_timeout_seconds = timeout_seconds;
			last_activity_ms = now_ms;
		}
		if(report_has_activity(&current) || transport_usb_attached() || puck_interface_active())
		{
			last_activity_ms = now_ms;
		}
		else if(inactivity_timeout_seconds > 0 &&
		        now_ms - last_activity_ms >= (int64_t)inactivity_timeout_seconds * MSEC_PER_SEC)
		{
			LOG_INF("input inactive for %d seconds: shutting down", inactivity_timeout_seconds);
			last_activity_ms = now_ms;
			power_off();
		}
		if((current.buttons & BIT(CONTROLLER_BUTTON_STEAM)) != 0U)
		{
			if(steam_button_since == 0)
			{
				steam_button_since = k_uptime_get();
			}
			else if(k_uptime_get() - steam_button_since >= STEAM_POWER_HOLD_MS)
			{
				LOG_INF("Steam held: shutting down");
				power_off_after_buttons_released(BIT(CONTROLLER_BUTTON_STEAM));
			}
		}
		else
		{
			steam_button_since = 0;
		}

		bool changed = first || memcmp(&current, &previous, sizeof(current)) != 0;
		bool refresh_due = k_uptime_get() - last_input_sent >= INPUT_REFRESH_MS;
		/* OFW uses each full-state ESB report as the container boundary for lizard
		 * reports produced by the preceding input sample. */
		bool esb_state_boundary = transport_esb_connected();

		if(changed || refresh_due || esb_state_boundary)
		{
			if(changed && (current.buttons & ~previous.buttons) != 0U)
			{
				hardware_haptic_pulse();
			}

			err = transport_send(&current);
			if(err && err != -ENOTCONN)
			{
				LOG_WRN("report send failed: %d", err);
			}
			else
			{
				last_input_sent = k_uptime_get();
			}
			previous = current;
			first = false;
		}
		/* OFW sends the full state before waking lizard input processing. */
		lizard_update(&current);

		hardware_wait_for_change();
		watchdog_feed();
	}
}
