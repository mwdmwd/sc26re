/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include "controller.h"

int battery_init(void);
int battery_get_status(struct controller_battery_report *report);
const char *battery_charge_state_name(uint8_t charge_state);
