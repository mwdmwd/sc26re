/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "controller.h"

int analog_init(void);
int analog_puck_pilot_present(bool *present);
int analog_battery_voltage_samples_mv(uint16_t *samples, size_t sample_count);
int analog_read_report(struct controller_report *report);
