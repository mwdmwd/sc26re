/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdint.h>

int haptics_init(void);

int haptics_backend_init(void);
int haptics_backend_pulse(void);
int haptics_backend_tone(uint32_t frequency_hz, uint32_t duration_ms);
