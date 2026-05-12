/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <stdbool.h>
#include <stdint.h>

#include <hal/nrf_gpio.h>
#include <hal/nrf_power.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#if CONFIG_POWEROFF
#include <zephyr/sys/poweroff.h>
#endif
#include <zephyr/sys/reboot.h>

#include "controller.h"
#include "power.h"
#include "rgbw_led.h"

#define VALVE_ISP_MAGIC_BASE 0x2001fff0u
#define POWER_OFF_RELEASE_POLL_MS 20
#define POWER_OFF_RELEASE_DEBOUNCE_MS 100
#define NRF_GPIO_PORT_COUNT 2
#define NRF_GPIO_PINS_PER_PORT 32
#define STEAM_BUTTON_NODE DT_ALIAS(button_steam)

#if DT_NODE_HAS_STATUS_OKAY(STEAM_BUTTON_NODE)
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

#if DT_NODE_HAS_STATUS_OKAY(STEAM_BUTTON_NODE)
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
	power_prepare_shutdown();
#if CONFIG_POWEROFF
	power_arm_systemoff_wake();
	sys_poweroff();
#else
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

void power_off_after_buttons_released(uint32_t release_mask)
{
	power_prepare_shutdown();

	if(release_mask != 0U)
	{
		LOG_INF("waiting for power-off buttons to release: 0x%08x", release_mask);
		while((hardware_read_buttons() & release_mask) != 0U)
		{
			hardware_wait_for_change();
			k_msleep(POWER_OFF_RELEASE_POLL_MS);
		}
		k_msleep(POWER_OFF_RELEASE_DEBOUNCE_MS);
	}

#if CONFIG_POWEROFF
	power_arm_systemoff_wake();
	sys_poweroff();
#else
	sys_reboot(SYS_REBOOT_COLD);
#endif
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
