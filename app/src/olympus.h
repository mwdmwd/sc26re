/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include "controller.h"

struct olympus_pad_debug
{
	int16_t x;
	int16_t y;
	uint16_t pressure;
	int16_t raw_pressure;
	int16_t raw_click;
	int32_t peak_x;
	int32_t peak_y;
	int32_t peak;
	int32_t noise_threshold;
	int32_t touch_threshold;
	int32_t release_threshold;
	int32_t pad_click_threshold;
	bool stick_touched;
	bool touched;
	bool clicked;
};

struct olympus_debug_snapshot
{
	uint32_t frame_count;
	struct olympus_pad_debug left;
	struct olympus_pad_debug right;
};

int olympus_init(void);
void olympus_read_report(struct controller_report *report);
void olympus_get_debug_snapshot(struct olympus_debug_snapshot *snapshot);
