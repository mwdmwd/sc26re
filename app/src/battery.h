/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>

#include "controller.h"

int battery_init(void);
int battery_prepare_poweroff(void);
int battery_get_status(struct controller_battery_report *report);
const char *battery_charge_state_name(uint8_t charge_state);
const char *battery_fuel_gauge_state_name(uint8_t state);
