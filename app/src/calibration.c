/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "calibration.h"
#include "imu_bias.h"
#include "valve_nvs.h"

LOG_MODULE_REGISTER(calibration);

#define IMU_GYRO_BIAS_PATH "cal/sensors/gyroscope/bias"
#define BATTERY_VOLTAGE_OFFSET_LIMIT_MV 500

struct side_calibration
{
	struct trigger_calibration trigger;
	struct stick_calibration stick;
	struct pressure_calibration pressure;
	bool trigger_loaded;
	bool stick_loaded;
	bool pressure_loaded;
	bool trigger_dirty;
	bool pressure_dirty;
};

static struct side_calibration sides[2] = {
	[CALIBRATION_LEFT] = {
		.trigger = { .type = 129 },
		.stick = { .type = 129 },
	},
	[CALIBRATION_RIGHT] = {
		.trigger = { .type = 129 },
		.stick = { .type = 129 },
	},
};
static int16_t battery_voltage_offset_mv;
static bool battery_voltage_offset_loaded;

K_MUTEX_DEFINE(calibration_mutex);

bool calibration_trigger_valid(const struct trigger_calibration *cal)
{
	return abs((int)cal->idle - (int)cal->pressed) >= 100;
}

bool calibration_stick_valid(const struct stick_calibration *cal)
{
	return cal->x_min < cal->x_center_min &&
	       cal->x_center_min <= cal->x_center_max &&
	       cal->x_center_max < cal->x_max &&
	       cal->y_min < cal->y_center_min &&
	       cal->y_center_min <= cal->y_center_max &&
	       cal->y_center_max < cal->y_max;
}

bool calibration_pressure_valid(const struct pressure_calibration *cal)
{
	return (cal->mode == 1U || cal->mode == 128U) &&
	       cal->max > cal->min &&
	       cal->pressure_scale != 0U;
}

bool calibration_trigger_loaded(enum calibration_side side)
{
	bool loaded;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	loaded = sides[side].trigger_loaded;
	k_mutex_unlock(&calibration_mutex);
	return loaded;
}

bool calibration_stick_loaded(enum calibration_side side)
{
	bool loaded;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	loaded = sides[side].stick_loaded;
	k_mutex_unlock(&calibration_mutex);
	return loaded;
}

bool calibration_pressure_loaded(enum calibration_side side)
{
	bool loaded;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	loaded = sides[side].pressure_loaded;
	k_mutex_unlock(&calibration_mutex);
	return loaded;
}

void calibration_analog_snapshot(struct analog_calibration_snapshot *snapshot)
{
	k_mutex_lock(&calibration_mutex, K_FOREVER);
	snapshot->trigger_left = sides[CALIBRATION_LEFT].trigger;
	snapshot->trigger_right = sides[CALIBRATION_RIGHT].trigger;
	snapshot->stick_left = sides[CALIBRATION_LEFT].stick;
	snapshot->stick_right = sides[CALIBRATION_RIGHT].stick;
	k_mutex_unlock(&calibration_mutex);
}

bool calibration_pressure_snapshot(enum calibration_side side,
                                   struct pressure_calibration *snapshot)
{
	bool loaded;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	*snapshot = sides[side].pressure;
	loaded = sides[side].pressure_loaded;
	k_mutex_unlock(&calibration_mutex);
	return loaded;
}

bool calibration_battery_voltage_offset_loaded(void)
{
	bool loaded;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	loaded = battery_voltage_offset_loaded;
	k_mutex_unlock(&calibration_mutex);
	return loaded;
}

int16_t calibration_battery_voltage_offset_mv(void)
{
	int16_t offset_mv;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	offset_mv = battery_voltage_offset_mv;
	k_mutex_unlock(&calibration_mutex);
	return offset_mv;
}

static bool battery_voltage_offset_valid(int16_t offset_mv)
{
	/* Corruption guard only, factory trim should be much smaller. */
	return offset_mv >= -BATTERY_VOLTAGE_OFFSET_LIMIT_MV &&
	       offset_mv <= BATTERY_VOLTAGE_OFFSET_LIMIT_MV;
}

static int load_battery_voltage_offset(settings_read_cb read_cb, void *cb_arg)
{
	int16_t loaded;
	ssize_t read_len = read_cb(cb_arg, &loaded, sizeof(loaded));

	if(read_len < 0)
	{
		return read_len;
	}
	if(read_len != sizeof(loaded))
	{
		return -EINVAL;
	}
	if(!battery_voltage_offset_valid(loaded))
	{
		LOG_WRN("Ignoring implausible battery-meter voltage offset: %d mV", loaded);
		return 0;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	battery_voltage_offset_mv = loaded;
	battery_voltage_offset_loaded = true;
	k_mutex_unlock(&calibration_mutex);
	LOG_INF("Loaded battery-meter voltage offset: %d mV", loaded);
	return 0;
}

static int load_trigger_setting(enum calibration_side side, settings_read_cb read_cb, void *cb_arg)
{
	struct trigger_calibration loaded;
	ssize_t read_len = read_cb(cb_arg, &loaded, sizeof(loaded));

	if(read_len < 0)
	{
		return read_len;
	}
	if(read_len != sizeof(loaded))
	{
		return -EINVAL;
	}
	if(!calibration_trigger_valid(&loaded))
	{
		LOG_WRN("Ignoring invalid %s trigger calibration: pressed=%u, idle=%u",
		        side == CALIBRATION_RIGHT ? "right" : "left", loaded.pressed, loaded.idle);
		return 0;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	sides[side].trigger = loaded;
	sides[side].trigger_loaded = true;
	k_mutex_unlock(&calibration_mutex);
	LOG_INF("Loaded %s trigger calibration: pressed=%u, idle=%u, inverted=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", loaded.pressed, loaded.idle,
	        loaded.inverted);
	return 0;
}

static int load_stick_setting(enum calibration_side side, settings_read_cb read_cb, void *cb_arg)
{
	struct stick_calibration loaded;
	ssize_t read_len = read_cb(cb_arg, &loaded, sizeof(loaded));

	if(read_len < 0)
	{
		return read_len;
	}
	if(read_len != sizeof(loaded))
	{
		return -EINVAL;
	}
	if(!calibration_stick_valid(&loaded))
	{
		LOG_WRN("Ignoring invalid %s stick calibration",
		        side == CALIBRATION_RIGHT ? "right" : "left");
		return 0;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	sides[side].stick = loaded;
	sides[side].stick_loaded = true;
	k_mutex_unlock(&calibration_mutex);
	LOG_INF("Loaded %s stick calibration: x_min=%u, x_center=%u..%u, x_max=%u, y_min=%u, "
	        "y_center=%u..%u, y_max=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", loaded.x_min, loaded.x_center_min,
	        loaded.x_center_max, loaded.x_max, loaded.y_min, loaded.y_center_min,
	        loaded.y_center_max, loaded.y_max);
	return 0;
}

static int load_pressure_setting(enum calibration_side side, settings_read_cb read_cb, void *cb_arg)
{
	struct pressure_calibration loaded;
	ssize_t read_len = read_cb(cb_arg, &loaded, sizeof(loaded));

	if(read_len < 0)
	{
		return read_len;
	}
	if(read_len != sizeof(loaded))
	{
		return -EINVAL;
	}
	if(loaded.mode == 0U)
	{
		k_mutex_lock(&calibration_mutex, K_FOREVER);
		memset(&sides[side].pressure, 0, sizeof(loaded));
		sides[side].pressure_loaded = false;
		k_mutex_unlock(&calibration_mutex);
		LOG_INF("Loaded cleared %s pressure calibration",
		        side == CALIBRATION_RIGHT ? "right" : "left");
		return 0;
	}
	if(!calibration_pressure_valid(&loaded))
	{
		k_mutex_lock(&calibration_mutex, K_FOREVER);
		memset(&sides[side].pressure, 0, sizeof(loaded));
		sides[side].pressure_loaded = false;
		k_mutex_unlock(&calibration_mutex);
		LOG_WRN("Ignoring invalid %s pressure calibration: mode=%u min=%d max=%d scale=%u",
		        side == CALIBRATION_RIGHT ? "right" : "left", loaded.mode, loaded.min, loaded.max,
		        loaded.pressure_scale);
		return 0;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	sides[side].pressure = loaded;
	sides[side].pressure_loaded = true;
	k_mutex_unlock(&calibration_mutex);
	LOG_INF("Loaded %s pressure calibration: mode=%u min=%d max=%d scale=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", loaded.mode, loaded.min, loaded.max,
	        loaded.pressure_scale);
	return 0;
}

static int cal_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	if(strcmp(name, "trg_l") == 0)
	{
		return len == sizeof(struct trigger_calibration)
		           ? load_trigger_setting(CALIBRATION_LEFT, read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "trg_r") == 0)
	{
		return len == sizeof(struct trigger_calibration)
		           ? load_trigger_setting(CALIBRATION_RIGHT, read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "joy_l") == 0)
	{
		return len == sizeof(struct stick_calibration)
		           ? load_stick_setting(CALIBRATION_LEFT, read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "joy_r") == 0)
	{
		return len == sizeof(struct stick_calibration)
		           ? load_stick_setting(CALIBRATION_RIGHT, read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "prs_l") == 0)
	{
		return len == sizeof(struct pressure_calibration)
		           ? load_pressure_setting(CALIBRATION_LEFT, read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "prs_r") == 0)
	{
		return len == sizeof(struct pressure_calibration)
		           ? load_pressure_setting(CALIBRATION_RIGHT, read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "volt_offset") == 0)
	{
		return len == sizeof(battery_voltage_offset_mv)
		           ? load_battery_voltage_offset(read_cb, cb_arg)
		           : -EINVAL;
	}
	if(strcmp(name, "sensors/gyroscope/bias") == 0)
	{
		/* Loaded explicitly by the IMU module. */
		return 0;
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(cal, "cal", NULL, cal_settings_set, NULL, NULL);

int calibration_load_settings(void)
{
	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		return -ENOTSUP;
	}

	return settings_load_subtree("cal");
}

static int save_setting(const char *path, const void *value, size_t len)
{
	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		return -ENOTSUP;
	}

	return settings_save_one(path, value, len);
}

#if FIXED_PARTITION_EXISTS(valve_storage)
static int valve_storage_read(const void *context, uint32_t offset, void *data, size_t len)
{
	return flash_area_read(context, offset, data, len);
}

static int open_valve_storage(const struct flash_area **fa, struct valve_nvs *nvs)
{
	int err = flash_area_open(PARTITION_ID(valve_storage), fa);

	if(err)
	{
		LOG_WRN("Failed to open valve_storage flash area: %d", err);
		return err;
	}

	err = valve_nvs_open(nvs, valve_storage_read, *fa, (*fa)->fa_size);
	if(err)
	{
		LOG_INF("OFW valve_storage is unreadable: %d", err);
		flash_area_close(*fa);
	}
	return err;
}

int calibration_import_imu_from_valve_storage(void)
{
	uint8_t encoded_bias[3 * sizeof(uint32_t)];
	const struct flash_area *fa;
	struct valve_nvs ofw_nvs;
	float local_bias[3] = { 0 };
	float bias[3];
	ssize_t local_len;
	int err;

	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		return -ENOTSUP;
	}

	local_len = settings_load_one(IMU_GYRO_BIAS_PATH, local_bias, sizeof(local_bias));
	if(local_len == sizeof(local_bias) && imu_gyro_bias_valid(local_bias))
	{
		return 0;
	}
	if(local_len < 0 && local_len != -ENOENT)
	{
		return (int)local_len;
	}
	if(local_len > 0)
	{
		LOG_WRN("Replacing malformed or invalid persisted gyro bias");
	}

	err = open_valve_storage(&fa, &ofw_nvs);
	if(err)
	{
		return err;
	}
	err = valve_nvs_read_setting(&ofw_nvs, IMU_GYRO_BIAS_PATH, encoded_bias, sizeof(encoded_bias));
	flash_area_close(fa);
	if(err == -ENOENT)
	{
		return 0;
	}
	if(err != sizeof(encoded_bias))
	{
		LOG_WRN("Ignoring OFW gyro bias with invalid length: %d", err);
		return err < 0 ? err : -EINVAL;
	}

	for(size_t axis = 0; axis < ARRAY_SIZE(bias); ++axis)
	{
		uint32_t bits = sys_get_le32(&encoded_bias[axis * sizeof(uint32_t)]);

		memcpy(&bias[axis], &bits, sizeof(bits));
	}
	if(!imu_gyro_bias_valid(bias))
	{
		LOG_WRN("Ignoring invalid OFW gyro bias");
		return -ERANGE;
	}

	err = save_setting(IMU_GYRO_BIAS_PATH, bias, sizeof(bias));
	if(err)
	{
		LOG_WRN("Failed to migrate OFW gyro bias: %d", err);
		return err;
	}
	LOG_INF("Migrated OFW gyro bias");
	return 0;
}

int calibration_import_valve_storage(void)
{
	const struct flash_area *fa;
	struct valve_nvs ofw_nvs;
	bool trg_l_found = false;
	bool trg_r_found = false;
	bool joy_l_found = false;
	bool joy_r_found = false;
	bool prs_l_found = false;
	bool prs_r_found = false;
	bool volt_offset_found = false;
	struct trigger_calibration ofw_trg_l = { 0 };
	struct trigger_calibration ofw_trg_r = { 0 };
	struct stick_calibration ofw_joy_l = { 0 };
	struct stick_calibration ofw_joy_r = { 0 };
	struct pressure_calibration ofw_prs_l = { 0 };
	struct pressure_calibration ofw_prs_r = { 0 };
	int16_t ofw_volt_offset = 0;
	int err;

	err = open_valve_storage(&fa, &ofw_nvs);
	if(err)
	{
		return err;
	}

	LOG_INF("Scanning valve_storage for OFW calibration data...");
	err = valve_nvs_read_setting(&ofw_nvs, "cal/trg_l", &ofw_trg_l, sizeof(ofw_trg_l));
	trg_l_found = err == sizeof(ofw_trg_l);
	err = valve_nvs_read_setting(&ofw_nvs, "cal/trg_r", &ofw_trg_r, sizeof(ofw_trg_r));
	trg_r_found = err == sizeof(ofw_trg_r);
	err = valve_nvs_read_setting(&ofw_nvs, "cal/joy_l", &ofw_joy_l, sizeof(ofw_joy_l));
	joy_l_found = err == sizeof(ofw_joy_l);
	err = valve_nvs_read_setting(&ofw_nvs, "cal/joy_r", &ofw_joy_r, sizeof(ofw_joy_r));
	joy_r_found = err == sizeof(ofw_joy_r);
	err = valve_nvs_read_setting(&ofw_nvs, "cal/prs_l", &ofw_prs_l, sizeof(ofw_prs_l));
	prs_l_found = err == sizeof(ofw_prs_l);
	err = valve_nvs_read_setting(&ofw_nvs, "cal/prs_r", &ofw_prs_r, sizeof(ofw_prs_r));
	prs_r_found = err == sizeof(ofw_prs_r);
	err = valve_nvs_read_setting(&ofw_nvs, "cal/volt_offset", &ofw_volt_offset,
	                             sizeof(ofw_volt_offset));
	volt_offset_found = err == sizeof(ofw_volt_offset);
	flash_area_close(fa);

	if(volt_offset_found &&
	   !battery_voltage_offset_valid(ofw_volt_offset) &&
	   !calibration_battery_voltage_offset_loaded())
	{
		LOG_WRN("Ignoring implausible OFW cal/volt_offset: %d mV", ofw_volt_offset);
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	trg_l_found = trg_l_found &&
	              calibration_trigger_valid(&ofw_trg_l) &&
	              !sides[CALIBRATION_LEFT].trigger_loaded;
	if(trg_l_found)
	{
		sides[CALIBRATION_LEFT].trigger = ofw_trg_l;
		sides[CALIBRATION_LEFT].trigger_loaded = true;
	}
	trg_r_found = trg_r_found &&
	              calibration_trigger_valid(&ofw_trg_r) &&
	              !sides[CALIBRATION_RIGHT].trigger_loaded;
	if(trg_r_found)
	{
		sides[CALIBRATION_RIGHT].trigger = ofw_trg_r;
		sides[CALIBRATION_RIGHT].trigger_loaded = true;
	}
	joy_l_found =
	    joy_l_found && calibration_stick_valid(&ofw_joy_l) && !sides[CALIBRATION_LEFT].stick_loaded;
	if(joy_l_found)
	{
		sides[CALIBRATION_LEFT].stick = ofw_joy_l;
		sides[CALIBRATION_LEFT].stick_loaded = true;
	}
	joy_r_found = joy_r_found &&
	              calibration_stick_valid(&ofw_joy_r) &&
	              !sides[CALIBRATION_RIGHT].stick_loaded;
	if(joy_r_found)
	{
		sides[CALIBRATION_RIGHT].stick = ofw_joy_r;
		sides[CALIBRATION_RIGHT].stick_loaded = true;
	}
	prs_l_found = prs_l_found &&
	              calibration_pressure_valid(&ofw_prs_l) &&
	              !sides[CALIBRATION_LEFT].pressure_loaded;
	if(prs_l_found)
	{
		sides[CALIBRATION_LEFT].pressure = ofw_prs_l;
		sides[CALIBRATION_LEFT].pressure_loaded = true;
	}
	prs_r_found = prs_r_found &&
	              calibration_pressure_valid(&ofw_prs_r) &&
	              !sides[CALIBRATION_RIGHT].pressure_loaded;
	if(prs_r_found)
	{
		sides[CALIBRATION_RIGHT].pressure = ofw_prs_r;
		sides[CALIBRATION_RIGHT].pressure_loaded = true;
	}
	volt_offset_found = volt_offset_found &&
	                    battery_voltage_offset_valid(ofw_volt_offset) &&
	                    !battery_voltage_offset_loaded;
	if(volt_offset_found)
	{
		battery_voltage_offset_mv = ofw_volt_offset;
		battery_voltage_offset_loaded = true;
	}
	k_mutex_unlock(&calibration_mutex);

	if(trg_l_found)
	{
		err = save_setting("cal/trg_l", &ofw_trg_l, sizeof(ofw_trg_l));
		if(!err)
		{
			LOG_INF("Migrated cal/trg_l: pressed=%u, idle=%u, inverted=%u", ofw_trg_l.pressed,
			        ofw_trg_l.idle, ofw_trg_l.inverted);
		}
	}
	if(trg_r_found)
	{
		err = save_setting("cal/trg_r", &ofw_trg_r, sizeof(ofw_trg_r));
		if(!err)
		{
			LOG_INF("Migrated cal/trg_r: pressed=%u, idle=%u, inverted=%u", ofw_trg_r.pressed,
			        ofw_trg_r.idle, ofw_trg_r.inverted);
		}
	}
	if(joy_l_found)
	{
		err = save_setting("cal/joy_l", &ofw_joy_l, sizeof(ofw_joy_l));
		if(!err)
		{
			LOG_INF("Migrated cal/joy_l: x_min=%u, x_center=%u..%u, x_max=%u, y_min=%u, "
			        "y_center=%u..%u, y_max=%u",
			        ofw_joy_l.x_min, ofw_joy_l.x_center_min, ofw_joy_l.x_center_max,
			        ofw_joy_l.x_max, ofw_joy_l.y_min, ofw_joy_l.y_center_min,
			        ofw_joy_l.y_center_max, ofw_joy_l.y_max);
		}
	}
	if(joy_r_found)
	{
		err = save_setting("cal/joy_r", &ofw_joy_r, sizeof(ofw_joy_r));
		if(!err)
		{
			LOG_INF("Migrated cal/joy_r: x_min=%u, x_center=%u..%u, x_max=%u, y_min=%u, "
			        "y_center=%u..%u, y_max=%u",
			        ofw_joy_r.x_min, ofw_joy_r.x_center_min, ofw_joy_r.x_center_max,
			        ofw_joy_r.x_max, ofw_joy_r.y_min, ofw_joy_r.y_center_min,
			        ofw_joy_r.y_center_max, ofw_joy_r.y_max);
		}
	}
	if(prs_l_found)
	{
		err = save_setting("cal/prs_l", &ofw_prs_l, sizeof(ofw_prs_l));
		if(!err)
		{
			LOG_INF("Migrated cal/prs_l: mode=%u min=%d max=%d scale=%u", ofw_prs_l.mode,
			        ofw_prs_l.min, ofw_prs_l.max, ofw_prs_l.pressure_scale);
		}
	}
	if(prs_r_found)
	{
		err = save_setting("cal/prs_r", &ofw_prs_r, sizeof(ofw_prs_r));
		if(!err)
		{
			LOG_INF("Migrated cal/prs_r: mode=%u min=%d max=%d scale=%u", ofw_prs_r.mode,
			        ofw_prs_r.min, ofw_prs_r.max, ofw_prs_r.pressure_scale);
		}
	}
	if(volt_offset_found)
	{
		err = save_setting("cal/volt_offset", &ofw_volt_offset, sizeof(ofw_volt_offset));
		if(err)
		{
			LOG_WRN("Failed to persist migrated cal/volt_offset: %d", err);
		}
		else
		{
			LOG_INF("Migrated cal/volt_offset: %d mV", ofw_volt_offset);
		}
	}

	return 0;
}
#else
int calibration_import_imu_from_valve_storage(void)
{
	return -ENODEV;
}

int calibration_import_valve_storage(void)
{
	return -ENODEV;
}
#endif

bool calibration_read_trigger(enum calibration_side side, uint8_t *buf, size_t capacity,
                              size_t *len)
{
	struct trigger_calibration value;

	if(capacity < sizeof(value))
	{
		return false;
	}
	k_mutex_lock(&calibration_mutex, K_FOREVER);
	value = sides[side].trigger;
	k_mutex_unlock(&calibration_mutex);
	memcpy(buf, &value, sizeof(value));
	*len = sizeof(value);
	return true;
}

int calibration_stage_trigger(enum calibration_side side, const uint8_t *value, size_t len)
{
	struct trigger_calibration staged;

	if(len != sizeof(staged))
	{
		return -EINVAL;
	}
	memcpy(&staged, value, sizeof(staged));
	if(!calibration_trigger_valid(&staged))
	{
		return -EINVAL;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	sides[side].trigger = staged;
	sides[side].trigger_loaded = true;
	sides[side].trigger_dirty = true;
	k_mutex_unlock(&calibration_mutex);

	LOG_INF("staged %s trigger calibration: pressed=%u, idle=%u, inverted=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", staged.pressed, staged.idle,
	        staged.inverted);
	return 0;
}

int calibration_commit_trigger(enum calibration_side side)
{
	const char *path = side == CALIBRATION_RIGHT ? "cal/trg_r" : "cal/trg_l";
	struct trigger_calibration value;
	bool value_is_current;
	bool needs_commit;
	int err;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	needs_commit = sides[side].trigger_dirty;
	value = sides[side].trigger;
	k_mutex_unlock(&calibration_mutex);
	if(!needs_commit)
	{
		return 0;
	}

	err = save_setting(path, &value, sizeof(value));
	if(err)
	{
		LOG_ERR("failed to commit %s: %d", path, err);
		return err;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	value_is_current = memcmp(&sides[side].trigger, &value, sizeof(value)) == 0;
	if(value_is_current)
	{
		sides[side].trigger_dirty = false;
	}
	k_mutex_unlock(&calibration_mutex);
	LOG_INF("committed %s%s", path, value_is_current ? "" : ", newer value remains staged");
	return 0;
}

bool calibration_read_pressure(enum calibration_side side, uint8_t *buf, size_t capacity,
                               size_t *len)
{
	struct pressure_calibration value;
	bool loaded;

	if(capacity < sizeof(value))
	{
		return false;
	}
	k_mutex_lock(&calibration_mutex, K_FOREVER);
	loaded = sides[side].pressure_loaded;
	value = sides[side].pressure;
	k_mutex_unlock(&calibration_mutex);
	if(!loaded)
	{
		return false;
	}
	memcpy(buf, &value, sizeof(value));
	*len = sizeof(value);
	return true;
}

int calibration_stage_pressure(enum calibration_side side, const uint8_t *value, size_t len)
{
	struct pressure_calibration staged;
	bool clear;

	if(len != sizeof(staged))
	{
		return -EINVAL;
	}
	memcpy(&staged, value, sizeof(staged));
	clear = staged.mode == 0U;
	if(!clear && !calibration_pressure_valid(&staged))
	{
		return -EINVAL;
	}
	if(clear)
	{
		memset(&staged, 0, sizeof(staged));
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	sides[side].pressure = staged;
	sides[side].pressure_loaded = !clear;
	sides[side].pressure_dirty = true;
	k_mutex_unlock(&calibration_mutex);

	LOG_INF("staged %s pressure calibration: mode=%u min=%d max=%d scale=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", staged.mode, staged.min, staged.max,
	        staged.pressure_scale);
	return 0;
}

int calibration_commit_pressure(enum calibration_side side)
{
	const char *path = side == CALIBRATION_RIGHT ? "cal/prs_r" : "cal/prs_l";
	struct pressure_calibration value;
	bool value_is_current;
	bool needs_commit;
	int err;

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	needs_commit = sides[side].pressure_dirty;
	value = sides[side].pressure;
	k_mutex_unlock(&calibration_mutex);
	if(!needs_commit)
	{
		return 0;
	}
	err = save_setting(path, &value, sizeof(value));
	if(err)
	{
		LOG_ERR("failed to commit %s: %d", path, err);
		return err;
	}

	k_mutex_lock(&calibration_mutex, K_FOREVER);
	value_is_current = memcmp(&sides[side].pressure, &value, sizeof(value)) == 0;
	if(value_is_current)
	{
		sides[side].pressure_dirty = false;
	}
	k_mutex_unlock(&calibration_mutex);
	LOG_INF("committed %s%s", path, value_is_current ? "" : ", newer value remains staged");
	return 0;
}
