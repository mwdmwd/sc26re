/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include "controller.h"

int analog_init(void);
int analog_read_report(struct controller_report *report);
