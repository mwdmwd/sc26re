/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

/* Far above a normal stationary gyro offset, but below plausible motion. */
#define IMU_GYRO_BIAS_MAX_RAD_S 1.0f

static inline bool imu_gyro_bias_valid(const float bias[3])
{
	for(size_t axis = 0; axis < 3; ++axis)
	{
		if(!isfinite(bias[axis]) ||
		   bias[axis] < -IMU_GYRO_BIAS_MAX_RAD_S ||
		   bias[axis] > IMU_GYRO_BIAS_MAX_RAD_S)
		{
			return false;
		}
	}

	return true;
}
