/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum calibration_side
{
	CALIBRATION_LEFT,
	CALIBRATION_RIGHT,
};

struct trigger_calibration
{
	uint16_t type;
	uint16_t pressed;
	uint16_t idle;
	uint8_t inverted;
} __packed;

struct stick_calibration
{
	uint16_t type;
	uint16_t x_min;
	uint16_t x_max;
	uint16_t x_center_min;
	uint16_t x_center_max;
	uint16_t y_min;
	uint16_t y_max;
	uint16_t y_center_min;
	uint16_t y_center_max;
} __packed;

struct pressure_calibration
{
	uint16_t mode;
	int16_t min;
	int16_t max;
	uint16_t pressure_scale;
} __packed;

bool calibration_trigger_valid(const struct trigger_calibration *cal);
bool calibration_stick_valid(const struct stick_calibration *cal);
bool calibration_pressure_valid(const struct pressure_calibration *cal);
bool calibration_trigger_loaded(enum calibration_side side);
bool calibration_stick_loaded(enum calibration_side side);
const struct trigger_calibration *calibration_trigger(enum calibration_side side);
const struct stick_calibration *calibration_stick(enum calibration_side side);
bool calibration_pressure_loaded(enum calibration_side side);
const struct pressure_calibration *calibration_pressure(enum calibration_side side);
bool calibration_battery_voltage_offset_loaded(void);
int16_t calibration_battery_voltage_offset_mv(void);
int calibration_load_settings(void);
int calibration_import_imu_from_valve_storage(void);
int calibration_import_valve_storage(void);
bool calibration_read_trigger(enum calibration_side side, uint8_t *buf, size_t capacity,
                              size_t *len);
int calibration_stage_trigger(enum calibration_side side, const uint8_t *value, size_t len);
int calibration_commit_trigger(enum calibration_side side);
bool calibration_read_pressure(enum calibration_side side, uint8_t *buf, size_t capacity,
                               size_t *len);
int calibration_stage_pressure(enum calibration_side side, const uint8_t *value, size_t len);
int calibration_commit_pressure(enum calibration_side side);
