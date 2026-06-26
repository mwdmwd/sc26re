/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>

#include "controller.h"

int analog_init(void);
int analog_puck_pilot_present(bool *present);
int analog_read_report(struct controller_report *report);
