/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#if CONFIG_BOARD_STEAM_CONTROLLER_IBEX
#include <hal/nrf_wdt.h>

/* Reload is required every 5 seconds */
static inline void watchdog_feed(void)
{
	nrf_wdt_reload_request_set(NRF_WDT0, NRF_WDT_RR0);
}
#else
static inline void watchdog_feed(void)
{
}
#endif
