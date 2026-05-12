/*
 * Runtime compatibility with Valve's resident Ibex bootloader.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <hal/nrf_wdt.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(valve_bootloader);

static void inherited_watchdog_feed(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	nrf_wdt_reload_request_set(NRF_WDT0, NRF_WDT_RR0);
}

K_TIMER_DEFINE(inherited_watchdog_timer, inherited_watchdog_feed, NULL);

static int valve_bootloader_init(void)
{
	if(!nrf_wdt_started_check(NRF_WDT0) ||
	   !nrf_wdt_reload_request_enable_check(NRF_WDT0, NRF_WDT_RR0))
	{
		return 0;
	}

	/*
	 * Valve's bootloader starts WDT0 with RR0 before jumping to the app.
	 * nRF52 cannot stop or reconfigure a running watchdog, so inherit and
	 * service it for the lifetime of the custom firmware.
	 */
	nrf_wdt_reload_request_set(NRF_WDT0, NRF_WDT_RR0);
	k_timer_start(&inherited_watchdog_timer, K_SECONDS(1), K_SECONDS(1));
	LOG_INF("feeding watchdog inherited from Valve bootloader");
	return 0;
}

SYS_INIT(valve_bootloader_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
