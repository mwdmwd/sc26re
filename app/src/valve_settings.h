/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void valve_settings_load_feature_state(void);
bool valve_settings_read(const char *path, uint8_t *buf, size_t capacity, size_t *len);
int valve_settings_stage(const char *path, const uint8_t *value, size_t len);
int valve_settings_commit(const char *path);
