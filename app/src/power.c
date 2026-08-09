/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#if CONFIG_POWEROFF
#include <zephyr/sys/poweroff.h>
#endif
#include <zephyr/sys/reboot.h>

#include "analog.h"
#include "battery.h"
#include "controller.h"
#include "power.h"
#include "puck_interface.h"
#include "rgbw_led.h"
#include "watchdog.h"

#define VALVE_ISP_MAGIC_BASE 0x2001fff0u
#define POWER_OFF_RELEASE_POLL_MS 20
#define POWER_OFF_RELEASE_DEBOUNCE_MS 100
#define POWER_OFF_EXTERNAL_POLL_MS 100
#define POWER_OFF_THREAD_STACK_SIZE 1024
#define POWER_OFF_THREAD_PRIORITY K_PRIO_COOP(9)
#define NRF_GPIO_PORT_COUNT 2
#define NRF_GPIO_PINS_PER_PORT 32
#define STEAM_BUTTON_NODE DT_ALIAS(button_steam)
#define STEAM_BUTTON_MASK BIT(CONTROLLER_BUTTON_STEAM)

#if DT_NODE_HAS_STATUS(STEAM_BUTTON_NODE, okay)
#define STEAM_WAKE_PIN \
	NRF_GPIO_PIN_MAP(DT_PROP(DT_GPIO_CTLR(STEAM_BUTTON_NODE, gpios), port), \
			 DT_GPIO_PIN(STEAM_BUTTON_NODE, gpios))
#if (DT_GPIO_FLAGS(STEAM_BUTTON_NODE, gpios) & GPIO_ACTIVE_LOW)
#define STEAM_WAKE_SENSE NRF_GPIO_PIN_SENSE_LOW
#else
#define STEAM_WAKE_SENSE NRF_GPIO_PIN_SENSE_HIGH
#endif
#endif

LOG_MODULE_REGISTER(power);

static uint32_t boot_reset_reason;
static uint32_t boot_gpio_latches[2];
static bool boot_state_captured;
static atomic_t power_off_claimed;
static uint32_t power_off_release_mask;
K_SEM_DEFINE(power_off_sem, 0, 1);

static void power_arm_systemoff_wake(void)
{
	for(uint32_t port = 0; port < NRF_GPIO_PORT_COUNT; ++port)
	{
		for(uint32_t pin = 0; pin < NRF_GPIO_PINS_PER_PORT; ++pin)
		{
			uint32_t nrf_pin = NRF_GPIO_PIN_MAP(port, pin);

			if(nrf_gpio_pin_present_check(nrf_pin))
			{
				nrf_gpio_cfg_sense_set(nrf_pin, NRF_GPIO_PIN_NOSENSE);
			}
		}
	}

#if DT_NODE_HAS_STATUS(STEAM_BUTTON_NODE, okay)
	nrf_gpio_cfg_sense_set(STEAM_WAKE_PIN, STEAM_WAKE_SENSE);
	LOG_INF("armed Steam button as sole SYSTEMOFF GPIO wake source");
#else
	LOG_WRN("no Steam button alias; SYSTEMOFF has no GPIO wake source");
#endif
}

static void power_prepare_shutdown(void)
{
#if CONFIG_IBEX_RGBW_LED
	rgbw_led_prepare_poweroff();
#endif

	if(IS_ENABLED(CONFIG_IBEX_BLE))
	{
		transport_ble_deactivate();
	}
	if(IS_ENABLED(CONFIG_IBEX_ESB))
	{
		transport_esb_deactivate();
	}
}

static void power_enter_system_off(void)
{
#if CONFIG_POWEROFF
	int err;
	bool waiting_for_external_power = false;

	power_arm_systemoff_wake();
	if(IS_ENABLED(CONFIG_IBEX_BATTERY))
	{
		for(;;)
		{
			bool external_source;
			bool puck_pilot_present = false;

			/* The original release check is stale after waiting on a charger. */
			if(waiting_for_external_power && (hardware_read_buttons() & STEAM_BUTTON_MASK) != 0U)
			{
				LOG_INF("Steam pressed while externally powered: restarting");
				sys_reboot(SYS_REBOOT_COLD);
			}

			external_source = transport_usb_attached() || puck_interface_active();
			if(!external_source)
			{
				err = analog_puck_pilot_present(&puck_pilot_present);
				if(err)
				{
					LOG_WRN("puck pilot read failed: %d; skipping MP2733 shipping mode", err);
					err = 0;
					break;
				}
				external_source = puck_pilot_present;
			}
			err = external_source ? -EAGAIN : battery_prepare_poweroff();
			if(err != -EAGAIN)
			{
				break;
			}
			if(!waiting_for_external_power)
			{
				LOG_INF("external power present; waiting to enter MP2733 shipping mode");
				waiting_for_external_power = true;
			}
			k_msleep(POWER_OFF_EXTERNAL_POLL_MS);
			watchdog_feed();
		}
		if(err < 0)
		{
			LOG_WRN("MP2733 shipping mode failed: %d; falling back to MCU SYSTEMOFF", err);
		}
	}
	sys_poweroff();
#else
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

void power_capture_boot_state(void)
{
	if(boot_state_captured)
	{
		return;
	}

#if NRF_POWER_HAS_RESETREAS
	boot_reset_reason = nrf_power_resetreas_get(NRF_POWER);
	nrf_power_resetreas_clear(NRF_POWER, boot_reset_reason);
#endif

#if defined(NRF_GPIO_LATCH_PRESENT)
	nrf_gpio_latches_read_and_clear(0, ARRAY_SIZE(boot_gpio_latches), boot_gpio_latches);
#endif

	boot_state_captured = true;
	LOG_INF("boot reset reason=0x%08x gpio_latch=p0:0x%08x p1:0x%08x", boot_reset_reason,
	        boot_gpio_latches[0], boot_gpio_latches[1]);
}

uint32_t power_boot_reset_reason(void)
{
	return boot_reset_reason;
}

void power_boot_gpio_latches(uint32_t latches[2])
{
	latches[0] = boot_gpio_latches[0];
	latches[1] = boot_gpio_latches[1];
}

int power_reboot_normal(void)
{
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

void power_off(void)
{
	power_off_after_buttons_released(STEAM_BUTTON_MASK);
}

static void power_off_thread_entry(void *unused1, void *unused2, void *unused3)
{
	uint32_t release_mask;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	k_sem_take(&power_off_sem, K_FOREVER);
	release_mask = power_off_release_mask;

	power_prepare_shutdown();

	if(release_mask != 0U)
	{
		LOG_INF("waiting for power-off buttons to release: 0x%08x", release_mask);
		while((hardware_read_buttons() & release_mask) != 0U)
		{
			hardware_wait_for_change();
			k_msleep(POWER_OFF_RELEASE_POLL_MS);
			watchdog_feed();
		}
		k_msleep(POWER_OFF_RELEASE_DEBOUNCE_MS);
	}

	power_enter_system_off();
}

K_THREAD_DEFINE(power_off_thread, POWER_OFF_THREAD_STACK_SIZE, power_off_thread_entry, NULL, NULL,
                NULL, POWER_OFF_THREAD_PRIORITY, 0, 0);

void power_off_after_buttons_released(uint32_t release_mask)
{
	if(!atomic_cas(&power_off_claimed, 0, 1))
	{
		LOG_DBG("power-off already in progress");
		return;
	}

	power_off_release_mask = release_mask;
	k_sem_give(&power_off_sem);
}

int power_reboot_to_valve_isp(void)
{
	volatile uint32_t *const magic = (volatile uint32_t *)VALVE_ISP_MAGIC_BASE;

	/*
	 * OFW: hid_power_write_isp_magic() writes these magic words and then
	 * the state machine performs a normal system reboot.
	 */
	magic[0] = 0xcd595b80u;
	magic[1] = 0x93541da0u;
	magic[2] = 0x0078143cu;
	magic[3] = 0x00000000u;

	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}
