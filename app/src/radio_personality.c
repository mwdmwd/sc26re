/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <hal/nrf_power.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>

#include "controller.h"

#define RADIO_PERSONALITY_GPREGRET_BLE 0xb1
#define RADIO_PERSONALITY_GPREGRET_ESB 0xe5
#define RADIO_PERSONALITY_SETTINGS_KEY "radio/personality"

static enum radio_personality current_personality;
static enum radio_personality pending_personality;
static enum radio_personality pending_persisted_personality;
static uint8_t saved_personality;

static uint8_t radio_personality_to_marker(enum radio_personality personality)
{
	switch(personality)
	{
		case RADIO_PERSONALITY_ESB:
			return RADIO_PERSONALITY_GPREGRET_ESB;
		case RADIO_PERSONALITY_BLE:
			return RADIO_PERSONALITY_GPREGRET_BLE;
	}
	__builtin_unreachable();
}

static enum radio_personality radio_personality_from_marker(uint8_t marker)
{
	if(IS_ENABLED(CONFIG_IBEX_ESB) && marker == RADIO_PERSONALITY_GPREGRET_ESB)
	{
		return RADIO_PERSONALITY_ESB;
	}

	if(IS_ENABLED(CONFIG_IBEX_BLE) && marker == RADIO_PERSONALITY_GPREGRET_BLE)
	{
		return RADIO_PERSONALITY_BLE;
	}

	return IS_ENABLED(CONFIG_IBEX_ESB) ? RADIO_PERSONALITY_ESB : RADIO_PERSONALITY_BLE;
}

static bool radio_personality_marker_is_valid(uint8_t marker)
{
	return (IS_ENABLED(CONFIG_IBEX_ESB) && marker == RADIO_PERSONALITY_GPREGRET_ESB) ||
	       (IS_ENABLED(CONFIG_IBEX_BLE) && marker == RADIO_PERSONALITY_GPREGRET_BLE);
}

static int persisted_personality_set(const char *name, size_t len, settings_read_cb read_cb,
                                     void *cb_arg)
{
	int err;

	if(strcmp(name, "personality") != 0)
	{
		return -ENOENT;
	}

	if(len != sizeof(saved_personality))
	{
		return -EINVAL;
	}

	err = read_cb(cb_arg, &saved_personality, sizeof(saved_personality));
	if(err < 0)
	{
		return err;
	}

	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(radio, "radio", NULL, persisted_personality_set, NULL, NULL);

static void delayed_reboot_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	radio_personality_reboot_into(pending_personality);
}

static void delayed_persist_handler(struct k_work *work)
{
	uint8_t marker;

	ARG_UNUSED(work);

	marker = radio_personality_to_marker(pending_persisted_personality);
	if(saved_personality == marker || !IS_ENABLED(CONFIG_SETTINGS))
	{
		return;
	}

	if(settings_save_one(RADIO_PERSONALITY_SETTINGS_KEY, &marker, sizeof(marker)) != 0)
	{
		return;
	}

	saved_personality = marker;
}

K_WORK_DELAYABLE_DEFINE(delayed_reboot_work, delayed_reboot_handler);
K_WORK_DELAYABLE_DEFINE(delayed_persist_work, delayed_persist_handler);

void radio_personality_init(void)
{
	uint8_t marker = nrf_power_gpregret_get(NRF_POWER, 0);

	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		(void)settings_load_subtree("radio");
	}

	if(radio_personality_marker_is_valid(marker))
	{
		current_personality = radio_personality_from_marker(marker);
		return;
	}

	current_personality = radio_personality_from_marker(saved_personality);
}

enum radio_personality radio_personality_get(void)
{
	return current_personality;
}

const char *radio_personality_name(void)
{
	switch(current_personality)
	{
		case RADIO_PERSONALITY_ESB:
			return "ESB";
		case RADIO_PERSONALITY_BLE:
			return "BLE";
	}
	__builtin_unreachable();
}

void radio_personality_reboot_into(enum radio_personality personality)
{
	k_work_cancel_delayable(&delayed_reboot_work);
	k_work_cancel_delayable(&delayed_persist_work);
	radio_personality_remember(personality);
	sys_reboot(SYS_REBOOT_COLD);
}

void radio_personality_remember(enum radio_personality personality)
{
	uint8_t marker = radio_personality_to_marker(personality);

	nrf_power_gpregret_set(NRF_POWER, 0, marker);
}

void radio_personality_persist_after(enum radio_personality personality, uint32_t delay_ms)
{
	pending_persisted_personality = personality;

	if(delay_ms == 0)
	{
		delayed_persist_handler(NULL);
		return;
	}

	k_work_reschedule(&delayed_persist_work, K_MSEC(delay_ms));
}

void radio_personality_cancel_pending_persist(void)
{
	k_work_cancel_delayable(&delayed_persist_work);
}

void radio_personality_reboot_into_after(enum radio_personality personality, uint32_t delay_ms)
{
	if(delay_ms == 0)
	{
		radio_personality_reboot_into(personality);
		return;
	}

	pending_personality = personality;
	k_work_reschedule(&delayed_reboot_work, K_MSEC(delay_ms));
}
