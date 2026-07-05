/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/lsm6dsv16x.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "controller.h"
#include "ibex_settings_registry.h"
#include "imu.h"
#include "sdl/controller_constants.h"

LOG_MODULE_REGISTER(imu);

#if CONFIG_IBEX_ACCELEROMETER && DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(accel0))
#define IMU_HAS_DEVICE 1
static const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(accel0));
#else
#define IMU_HAS_DEVICE 0
#endif

#define IMU_ACCEL_UG_PER_REPORT_UNIT 61
#define IMU_GYRO_REPORT_SCALE_DIVISOR 6103
#define IMU_DEGREES_E5_PER_RADIAN 5729578.0f
#define IMU_QUATERNION_REPORT_SCALE INT16_MAX
#define IMU_SFLP_BIAS_UPDATE_INTERVAL_US USEC_PER_SEC

#define IMU_MODE_PATH "settings/sensors/imu/mode"
#define IMU_MOUNTING_MATRIX_PATH "settings/sensors/imu/mounting_matrix"
#define IMU_GYRO_DZ_THRESHOLD_PATH "settings/sensors/imu/gyro_dz_threshold"
#define IMU_GYRO_BIAS_PATH "cal/sensors/gyroscope/bias"

static int8_t mounting_matrix[9] = {
	/* clang-format off */
	0, 1, 0,
	-1, 0, 0,
	0, 0, 1,
	/* clang-format on */
};
static int8_t staged_mounting_matrix[9];
static bool mounting_matrix_dirty;

static uint16_t imu_mode;
static uint16_t staged_imu_mode;
static bool imu_mode_dirty;
static bool imu_enabled;

static int32_t gyro_dz_threshold;
static int32_t staged_gyro_dz_threshold;
static bool gyro_dz_threshold_dirty;

static float gyro_bias[3];
static float staged_gyro_bias[3];
static float gyro_bias_candidate[3];
static bool gyro_bias_dirty;

static int64_t last_sflp_bias_update_us;
static bool registry_callback_replay;
static struct controller_report cached_imu_report;
static struct k_spinlock cached_imu_report_lock;
static bool imu_motion_trigger_enabled;
static bool imu_trigger_ready;

K_MUTEX_DEFINE(imu_io_lock);

#if IMU_HAS_DEVICE
static const struct sensor_trigger imu_fifo_trigger = {
	.type = SENSOR_TRIG_FIFO_WATERMARK,
	.chan = (enum sensor_channel)LSM6DSV16X_SENSOR_CHAN_FIFO,
};

static void imu_fifo_trigger_handler(const struct device *dev,
                                     const struct sensor_trigger *trigger);
#endif

struct imu_settings_load_target
{
	void *value;
	size_t value_size;
	bool loaded;
};

static float sensor_value_to_float_s(const struct sensor_value *value)
{
	return sensor_value_to_float(value);
}

static int16_t clamp_i16_int(int32_t value)
{
	return (int16_t)CLAMP(value, INT16_MIN, INT16_MAX);
}

static int16_t accel_axis_to_report(const struct sensor_value *value)
{
	int64_t micro_ms2 = sensor_value_to_micro(value);
	int64_t micro_g = micro_ms2 * 1000000 / SENSOR_G;

	return clamp_i16_int((int32_t)(micro_g / IMU_ACCEL_UG_PER_REPORT_UNIT));
}

static int16_t gyro_axis_to_report(float rad_s)
{
	int32_t degrees_e5 = (int32_t)(rad_s * IMU_DEGREES_E5_PER_RADIAN);

	return clamp_i16_int(degrees_e5 / IMU_GYRO_REPORT_SCALE_DIVISOR);
}

static int16_t quaternion_component_to_report(const struct sensor_value *value)
{
	float component = sensor_value_to_float_s(value);

	return clamp_i16_int((int32_t)(component * IMU_QUATERNION_REPORT_SCALE));
}

static void apply_orientation_i16(int16_t xyz[3])
{
	int16_t in[3] = { xyz[0], xyz[1], xyz[2] };

	for(size_t row = 0; row < 3; ++row)
	{
		int32_t value = 0;

		for(size_t col = 0; col < 3; ++col)
		{
			value += mounting_matrix[row * 3 + col] * in[col];
		}
		xyz[row] = clamp_i16_int(value);
	}
}

static void fill_identity_quaternion_report(struct controller_report *report)
{
	report->gyro_quat_w = INT16_MAX;
	report->gyro_quat_x = 0;
	report->gyro_quat_y = 0;
	report->gyro_quat_z = 0;
}

static void fill_quaternion_report(struct controller_report *report,
                                   const struct sensor_value value[4])
{
	int16_t xyz[3] = {
		quaternion_component_to_report(&value[0]),
		quaternion_component_to_report(&value[1]),
		quaternion_component_to_report(&value[2]),
	};

	apply_orientation_i16(xyz);
	report->gyro_quat_w = quaternion_component_to_report(&value[3]);
	report->gyro_quat_x = xyz[0];
	report->gyro_quat_y = xyz[1];
	report->gyro_quat_z = xyz[2];
}

static void copy_imu_report_fields(struct controller_report *dst,
                                   const struct controller_report *src)
{
	dst->accel_x = src->accel_x;
	dst->accel_y = src->accel_y;
	dst->accel_z = src->accel_z;
	dst->gyro_x = src->gyro_x;
	dst->gyro_y = src->gyro_y;
	dst->gyro_z = src->gyro_z;
	dst->gyro_quat_w = src->gyro_quat_w;
	dst->gyro_quat_x = src->gyro_quat_x;
	dst->gyro_quat_y = src->gyro_quat_y;
	dst->gyro_quat_z = src->gyro_quat_z;
	dst->imu_timestamp_us = src->imu_timestamp_us;
}

static void store_cached_imu_report(const struct controller_report *report)
{
	k_spinlock_key_t key = k_spin_lock(&cached_imu_report_lock);

	copy_imu_report_fields(&cached_imu_report, report);
	k_spin_unlock(&cached_imu_report_lock, key);
}

static void load_cached_imu_report(struct controller_report *report)
{
	k_spinlock_key_t key = k_spin_lock(&cached_imu_report_lock);

	copy_imu_report_fields(report, &cached_imu_report);
	k_spin_unlock(&cached_imu_report_lock, key);
}

static void update_bias_candidate_from_sflp(const struct sensor_value value[3])
{
	int64_t now_us = k_ticks_to_us_floor64(k_uptime_ticks());

	if(last_sflp_bias_update_us != 0 &&
	   now_us - last_sflp_bias_update_us < IMU_SFLP_BIAS_UPDATE_INTERVAL_US)
	{
		return;
	}

	for(size_t i = 0; i < 3; ++i)
	{
		gyro_bias_candidate[i] = sensor_value_to_float_s(&value[i]);
	}
	last_sflp_bias_update_us = now_us;
}

static int program_gyro_bias(void)
{
#if IMU_HAS_DEVICE
	struct sensor_value bias[3];
	int err;

	if(!device_is_ready(imu_dev))
	{
		return 0;
	}

	k_mutex_lock(&imu_io_lock, K_FOREVER);
	for(size_t i = 0; i < 3; ++i)
	{
		(void)sensor_value_from_float(&bias[i], gyro_bias[i]);
	}
	err = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OFFSET, bias);
	k_mutex_unlock(&imu_io_lock);

	return err;
#else
	return 0;
#endif
}

static int set_motion_trigger_enabled(bool enabled)
{
#if IMU_HAS_DEVICE
	int err;

	if(!imu_trigger_ready || !device_is_ready(imu_dev))
	{
		return 0;
	}
	if(imu_motion_trigger_enabled == enabled)
	{
		return 0;
	}

	k_mutex_lock(&imu_io_lock, K_FOREVER);
	err = sensor_trigger_set(imu_dev, &imu_fifo_trigger, enabled ? imu_fifo_trigger_handler : NULL);
	k_mutex_unlock(&imu_io_lock);
	if(err)
	{
		LOG_WRN("failed to %s IMU FIFO interrupt: %d", enabled ? "enable" : "disable", err);
		return err;
	}

	imu_motion_trigger_enabled = enabled;
	return 0;
#else
	ARG_UNUSED(enabled);
	return 0;
#endif
}

void imu_reset(void)
{
	last_sflp_bias_update_us = 0;
	(void)program_gyro_bias();
}

static void apply_mode(uint16_t mode)
{
	bool was_enabled = imu_enabled;
	bool enabled = mode != SETTING_GYRO_MODE_OFF;
	uint16_t old_mode = imu_mode;

	imu_mode = mode;
	imu_enabled = enabled;
	if(enabled && (!was_enabled || old_mode != mode))
	{
		imu_reset();
	}
	if(old_mode != mode || was_enabled != enabled)
	{
		(void)set_motion_trigger_enabled(enabled);
	}
}

static void setting_changed(uint8_t id, int16_t value)
{
	if(id != IBEX_SETTING_IMU_MODE || registry_callback_replay)
	{
		return;
	}

	staged_imu_mode = (uint16_t)value;
	imu_mode_dirty = true;
	apply_mode((uint16_t)value);
}

static int load_exact_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
                         void *param)
{
	struct imu_settings_load_target *target = param;
	ssize_t read_len;

	if(key != NULL && key[0] != '\0')
	{
		return 0;
	}
	if(len != target->value_size)
	{
		return -EINVAL;
	}

	read_len = read_cb(cb_arg, target->value, target->value_size);
	if(read_len < 0)
	{
		return read_len;
	}
	if((size_t)read_len != target->value_size)
	{
		return -EINVAL;
	}

	target->loaded = true;
	return 1;
}

static bool load_setting_exact(const char *path, void *value, size_t value_size)
{
	struct imu_settings_load_target target = {
		.value = value,
		.value_size = value_size,
	};

	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		return false;
	}

	(void)settings_load_subtree_direct(path, load_exact_cb, &target);
	return target.loaded;
}

static void load_persisted_settings(void)
{
	uint16_t loaded_mode;
	int32_t loaded_threshold;
	float loaded_bias[3];
	int8_t loaded_matrix[9];

	if(load_setting_exact(IMU_MOUNTING_MATRIX_PATH, loaded_matrix, sizeof(loaded_matrix)))
	{
		memcpy(mounting_matrix, loaded_matrix, sizeof(mounting_matrix));
	}
	if(load_setting_exact(IMU_GYRO_DZ_THRESHOLD_PATH, &loaded_threshold, sizeof(loaded_threshold)))
	{
		gyro_dz_threshold = loaded_threshold;
	}
	if(load_setting_exact(IMU_GYRO_BIAS_PATH, loaded_bias, sizeof(loaded_bias)))
	{
		memcpy(gyro_bias, loaded_bias, sizeof(gyro_bias));
		memcpy(gyro_bias_candidate, loaded_bias, sizeof(gyro_bias_candidate));
	}
	else
	{
		memcpy(gyro_bias_candidate, gyro_bias, sizeof(gyro_bias_candidate));
	}
	if(load_setting_exact(IMU_MODE_PATH, &loaded_mode, sizeof(loaded_mode)))
	{
		(void)ibex_setting_set(IBEX_SETTING_IMU_MODE, (int16_t)loaded_mode);
		apply_mode(loaded_mode);
	}

	memcpy(staged_mounting_matrix, mounting_matrix, sizeof(staged_mounting_matrix));
	staged_gyro_dz_threshold = gyro_dz_threshold;
	memcpy(staged_gyro_bias, gyro_bias, sizeof(staged_gyro_bias));
	staged_imu_mode = imu_mode;
}

int imu_init(void)
{
	int err;

	fill_identity_quaternion_report(&cached_imu_report);

	load_persisted_settings();

	registry_callback_replay = true;
	err = ibex_settings_register_callback(setting_changed);
	registry_callback_replay = false;
	if(err)
	{
		LOG_WRN("failed to register IMU setting callback: %d", err);
	}

#if IMU_HAS_DEVICE
	if(!device_is_ready(imu_dev))
	{
		LOG_WRN("IMU is not ready (initialized=%u init_res=%u), reports will contain zeroes",
		        imu_dev->state->initialized, imu_dev->state->init_res);
	}
	else
	{
		err = program_gyro_bias();
		if(err)
		{
			LOG_WRN("failed to program gyro bias: %d", err);
		}

		imu_trigger_ready = true;
		(void)set_motion_trigger_enabled(imu_enabled);
	}
#else
	LOG_WRN("no accel0 device-tree alias, reports will contain zeroes");
#endif

	return 0;
}

bool imu_ready(void)
{
#if IMU_HAS_DEVICE
	return device_is_ready(imu_dev);
#else
	return false;
#endif
}

static int build_imu_report_from_sensor(struct controller_report *report)
{
#if IMU_HAS_DEVICE
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value sflp_bias[3];
	struct sensor_value sflp_quat[4];
	struct sensor_value timestamp;
	float raw_gyro_rad_s[3];
	float bias_rad_s[3];
	float corrected_gyro_rad_s[3];
	int16_t oriented[3];
	int err;

	if(!imu_enabled || !device_is_ready(imu_dev))
	{
		return 0;
	}

	err = sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if(err)
	{
		return err;
	}
	err = sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if(err)
	{
		return err;
	}

	err = sensor_channel_get(imu_dev, (enum sensor_channel)LSM6DSV16X_SENSOR_CHAN_SFLP_GYRO_BIAS,
	                         sflp_bias);
	if(err == 0)
	{
		update_bias_candidate_from_sflp(sflp_bias);
		for(size_t i = 0; i < 3; ++i)
		{
			bias_rad_s[i] = sensor_value_to_float_s(&sflp_bias[i]);
		}
	}
	else
	{
		memcpy(bias_rad_s, gyro_bias, sizeof(bias_rad_s));
	}

	for(size_t i = 0; i < 3; ++i)
	{
		raw_gyro_rad_s[i] = sensor_value_to_float_s(&gyro[i]);
		corrected_gyro_rad_s[i] = raw_gyro_rad_s[i] - bias_rad_s[i];
	}

	oriented[0] = accel_axis_to_report(&accel[0]);
	oriented[1] = accel_axis_to_report(&accel[1]);
	oriented[2] = accel_axis_to_report(&accel[2]);
	apply_orientation_i16(oriented);
	report->accel_x = oriented[0];
	report->accel_y = oriented[1];
	report->accel_z = oriented[2];

	oriented[0] = gyro_axis_to_report(corrected_gyro_rad_s[0]);
	oriented[1] = gyro_axis_to_report(corrected_gyro_rad_s[1]);
	oriented[2] = gyro_axis_to_report(corrected_gyro_rad_s[2]);
	apply_orientation_i16(oriented);
	for(size_t i = 0; i < 3; ++i)
	{
		if(-gyro_dz_threshold < oriented[i] && gyro_dz_threshold > oriented[i])
		{
			oriented[i] = 0;
		}
	}
	report->gyro_x = oriented[0];
	report->gyro_y = oriented[1];
	report->gyro_z = oriented[2];

	err = sensor_channel_get(
	    imu_dev, (enum sensor_channel)LSM6DSV16X_SENSOR_CHAN_SFLP_GAME_ROTATION_VECTOR, sflp_quat);
	if(err == 0)
	{
		fill_quaternion_report(report, sflp_quat);
	}
	else
	{
		fill_identity_quaternion_report(report);
	}

	err = sensor_channel_get(imu_dev, (enum sensor_channel)LSM6DSV16X_SENSOR_CHAN_TIMESTAMP,
	                         &timestamp);
	if(err == 0)
	{
		report->imu_timestamp_us = (uint32_t)timestamp.val1;
	}

	return 0;
#else
	ARG_UNUSED(report);
	return 0;
#endif
}

#if IMU_HAS_DEVICE
static void imu_fifo_trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
	struct controller_report report = { 0 };
	int err;

	ARG_UNUSED(trigger);

	if(!imu_enabled)
	{
		return;
	}
	k_mutex_lock(&imu_io_lock, K_FOREVER);
	err = sensor_sample_fetch_chan(dev, (enum sensor_channel)LSM6DSV16X_SENSOR_CHAN_FIFO);
	if(err == 0 && imu_enabled)
	{
		err = build_imu_report_from_sensor(&report);
	}
	k_mutex_unlock(&imu_io_lock);

	if(err)
	{
		LOG_DBG("failed to update IMU report: %d", err);
		return;
	}
	if(!imu_enabled)
	{
		return;
	}

	store_cached_imu_report(&report);
}
#endif

int imu_read_report(struct controller_report *report)
{
	if(!imu_enabled)
	{
		return 0;
	}

	load_cached_imu_report(report);
	return 0;
}

int imu_calibrate_gyro(void)
{
	int err;

	k_mutex_lock(&imu_io_lock, K_FOREVER);
	memcpy(gyro_bias, gyro_bias_candidate, sizeof(gyro_bias));
	memcpy(staged_gyro_bias, gyro_bias, sizeof(staged_gyro_bias));
	gyro_bias_dirty = false;
	last_sflp_bias_update_us = 0;
	k_mutex_unlock(&imu_io_lock);

	err = program_gyro_bias();
	if(err)
	{
		LOG_WRN("failed to program gyro bias: %d", err);
		return err;
	}

	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		return 0;
	}

	err = settings_save_one(IMU_GYRO_BIAS_PATH, gyro_bias, sizeof(gyro_bias));
	if(err)
	{
		LOG_WRN("failed to save gyro bias: %d", err);
		return err;
	}
	LOG_INF("saved gyro biases");
	return 0;
}

bool imu_settings_read(const char *path, uint8_t *buf, size_t capacity, size_t *len)
{
	if(strcmp(path, IMU_MODE_PATH) == 0)
	{
		if(capacity < sizeof(uint16_t))
		{
			return false;
		}
		sys_put_le16(imu_mode, buf);
		*len = sizeof(uint16_t);
		return true;
	}
	if(strcmp(path, IMU_MOUNTING_MATRIX_PATH) == 0)
	{
		if(capacity < sizeof(mounting_matrix))
		{
			return false;
		}
		memcpy(buf, mounting_matrix, sizeof(mounting_matrix));
		*len = sizeof(mounting_matrix);
		return true;
	}
	if(strcmp(path, IMU_GYRO_DZ_THRESHOLD_PATH) == 0)
	{
		if(capacity < sizeof(int32_t))
		{
			return false;
		}
		sys_put_le32((uint32_t)gyro_dz_threshold, buf);
		*len = sizeof(int32_t);
		return true;
	}
	if(strcmp(path, IMU_GYRO_BIAS_PATH) == 0)
	{
		if(capacity < sizeof(gyro_bias))
		{
			return false;
		}
		memcpy(buf, gyro_bias, sizeof(gyro_bias));
		*len = sizeof(gyro_bias);
		return true;
	}

	return false;
}

int imu_settings_stage(const char *path, const uint8_t *value, size_t len)
{
	int err;

	if(strcmp(path, IMU_MODE_PATH) == 0)
	{
		if(len != sizeof(uint16_t))
		{
			return -EINVAL;
		}
		staged_imu_mode = sys_get_le16(value);
		imu_mode_dirty = true;
		(void)ibex_setting_set(IBEX_SETTING_IMU_MODE, (int16_t)staged_imu_mode);
		apply_mode(staged_imu_mode);
		return 0;
	}
	if(strcmp(path, IMU_MOUNTING_MATRIX_PATH) == 0)
	{
		if(len != sizeof(mounting_matrix))
		{
			return -EINVAL;
		}
		memcpy(staged_mounting_matrix, value, sizeof(staged_mounting_matrix));
		memcpy(mounting_matrix, staged_mounting_matrix, sizeof(mounting_matrix));
		mounting_matrix_dirty = true;
		return 0;
	}
	if(strcmp(path, IMU_GYRO_DZ_THRESHOLD_PATH) == 0)
	{
		if(len != sizeof(int32_t))
		{
			return -EINVAL;
		}
		staged_gyro_dz_threshold = (int32_t)sys_get_le32(value);
		gyro_dz_threshold = staged_gyro_dz_threshold;
		gyro_dz_threshold_dirty = true;
		return 0;
	}
	if(strcmp(path, IMU_GYRO_BIAS_PATH) == 0)
	{
		if(len != sizeof(gyro_bias))
		{
			return -EINVAL;
		}
		k_mutex_lock(&imu_io_lock, K_FOREVER);
		memcpy(staged_gyro_bias, value, sizeof(staged_gyro_bias));
		memcpy(gyro_bias, staged_gyro_bias, sizeof(gyro_bias));
		memcpy(gyro_bias_candidate, gyro_bias, sizeof(gyro_bias_candidate));
		gyro_bias_dirty = true;
		last_sflp_bias_update_us = 0;
		k_mutex_unlock(&imu_io_lock);
		err = program_gyro_bias();
		if(err)
		{
			return err;
		}
		return 0;
	}

	return -ENOENT;
}

static int commit_setting_if_dirty(const char *path, const void *value, size_t len, bool *dirty)
{
	int err;

	if(!*dirty)
	{
		return 0;
	}
	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		*dirty = false;
		return 0;
	}

	err = settings_save_one(path, value, len);
	if(err)
	{
		return err;
	}

	*dirty = false;
	return 0;
}

int imu_settings_commit(const char *path)
{
	if(strcmp(path, IMU_MODE_PATH) == 0)
	{
		return commit_setting_if_dirty(path, &staged_imu_mode, sizeof(staged_imu_mode),
		                               &imu_mode_dirty);
	}
	if(strcmp(path, IMU_MOUNTING_MATRIX_PATH) == 0)
	{
		return commit_setting_if_dirty(path, staged_mounting_matrix, sizeof(staged_mounting_matrix),
		                               &mounting_matrix_dirty);
	}
	if(strcmp(path, IMU_GYRO_DZ_THRESHOLD_PATH) == 0)
	{
		return commit_setting_if_dirty(path, &staged_gyro_dz_threshold,
		                               sizeof(staged_gyro_dz_threshold), &gyro_dz_threshold_dirty);
	}
	if(strcmp(path, IMU_GYRO_BIAS_PATH) == 0)
	{
		return commit_setting_if_dirty(path, staged_gyro_bias, sizeof(staged_gyro_bias),
		                               &gyro_bias_dirty);
	}

	return -ENOENT;
}
