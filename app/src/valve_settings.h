/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VALVE_ESB_BOND_SIZE 24U
#define VALVE_ESB_BOND_SLOT_COUNT 2U

typedef void (*valve_settings_esb_bond_changed_cb_t)(void);

void valve_settings_load_feature_state(void);
bool valve_settings_read(const char *path, uint8_t *buf, size_t capacity, size_t *len);
int valve_settings_stage(const char *path, const uint8_t *value, size_t len);
int valve_settings_commit(const char *path);
bool valve_settings_copy_active_esb_bond(uint8_t *bond, size_t capacity, uint8_t *slot);
int valve_settings_register_esb_bond_changed_callback(
    valve_settings_esb_bond_changed_cb_t callback);
int valve_settings_save_esb_bond(uint8_t slot, const uint8_t *bond, size_t len, bool select);
