/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include "controller.h"

enum triton_state_report_body_offset
{
	TRITON_STATE_SEQUENCE = 0,
	TRITON_STATE_BUTTONS = 1,
	TRITON_STATE_TRIGGER_LEFT = 5,
	TRITON_STATE_TRIGGER_RIGHT = 7,
	TRITON_STATE_STICK_LEFT_X = 9,
	TRITON_STATE_STICK_LEFT_Y = 11,
	TRITON_STATE_STICK_RIGHT_X = 13,
	TRITON_STATE_STICK_RIGHT_Y = 15,
	TRITON_STATE_TOUCHPAD_LEFT_X = 17,
	TRITON_STATE_TOUCHPAD_LEFT_Y = 19,
	TRITON_STATE_TOUCHPAD_LEFT_PRESSURE = 21,
	TRITON_STATE_TOUCHPAD_RIGHT_X = 23,
	TRITON_STATE_TOUCHPAD_RIGHT_Y = 25,
	TRITON_STATE_TOUCHPAD_RIGHT_PRESSURE = 27,
	TRITON_STATE_IMU_TIMESTAMP = 29,
	TRITON_STATE_ACCEL_X = 33,
	TRITON_STATE_ACCEL_Y = 35,
	TRITON_STATE_ACCEL_Z = 37,
	TRITON_STATE_GYRO_X = 39,
	TRITON_STATE_GYRO_Y = 41,
	TRITON_STATE_GYRO_Z = 43,
};

static inline uint32_t triton_state_report_timestamp_us(void)
{
	return k_uptime_get_32() * USEC_PER_MSEC;
}

static inline void triton_state_report_pack_body(uint8_t *body, size_t body_size, uint8_t sequence,
                                                 const struct controller_report *report,
                                                 uint32_t imu_timestamp_us)
{
	memset(body, 0, body_size);
	body[TRITON_STATE_SEQUENCE] = sequence;
	sys_put_le32(report->buttons, &body[TRITON_STATE_BUTTONS]);
	sys_put_le16(report->trigger_left, &body[TRITON_STATE_TRIGGER_LEFT]);
	sys_put_le16(report->trigger_right, &body[TRITON_STATE_TRIGGER_RIGHT]);
	sys_put_le16(report->stick_left_x, &body[TRITON_STATE_STICK_LEFT_X]);
	sys_put_le16(report->stick_left_y, &body[TRITON_STATE_STICK_LEFT_Y]);
	sys_put_le16(report->stick_right_x, &body[TRITON_STATE_STICK_RIGHT_X]);
	sys_put_le16(report->stick_right_y, &body[TRITON_STATE_STICK_RIGHT_Y]);
	sys_put_le16(report->touchpad_left_x, &body[TRITON_STATE_TOUCHPAD_LEFT_X]);
	sys_put_le16(report->touchpad_left_y, &body[TRITON_STATE_TOUCHPAD_LEFT_Y]);
	sys_put_le16(report->touchpad_left_pressure, &body[TRITON_STATE_TOUCHPAD_LEFT_PRESSURE]);
	sys_put_le16(report->touchpad_right_x, &body[TRITON_STATE_TOUCHPAD_RIGHT_X]);
	sys_put_le16(report->touchpad_right_y, &body[TRITON_STATE_TOUCHPAD_RIGHT_Y]);
	sys_put_le16(report->touchpad_right_pressure, &body[TRITON_STATE_TOUCHPAD_RIGHT_PRESSURE]);
	sys_put_le32(imu_timestamp_us, &body[TRITON_STATE_IMU_TIMESTAMP]);
	sys_put_le16(report->accel_x, &body[TRITON_STATE_ACCEL_X]);
	sys_put_le16(report->accel_y, &body[TRITON_STATE_ACCEL_Y]);
	sys_put_le16(report->accel_z, &body[TRITON_STATE_ACCEL_Z]);
}
