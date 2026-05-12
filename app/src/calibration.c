/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>

#include "calibration.h"

LOG_MODULE_REGISTER(calibration);

static struct trigger_calibration trigger_left = {
	.type = 129,
};
static struct trigger_calibration trigger_right = {
	.type = 129,
};
static struct stick_calibration stick_left = {
	.type = 129,
};
static struct stick_calibration stick_right = {
	.type = 129,
};

static bool trigger_left_loaded;
static bool trigger_right_loaded;
static bool stick_left_loaded;
static bool stick_right_loaded;
static bool trigger_left_dirty;
static bool trigger_right_dirty;

static struct trigger_calibration *trigger_for_side(enum calibration_side side)
{
	return side == CALIBRATION_RIGHT ? &trigger_right : &trigger_left;
}

static struct stick_calibration *stick_for_side(enum calibration_side side)
{
	return side == CALIBRATION_RIGHT ? &stick_right : &stick_left;
}

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

bool calibration_trigger_loaded(enum calibration_side side)
{
	return side == CALIBRATION_RIGHT ? trigger_right_loaded : trigger_left_loaded;
}

bool calibration_stick_loaded(enum calibration_side side)
{
	return side == CALIBRATION_RIGHT ? stick_right_loaded : stick_left_loaded;
}

const struct trigger_calibration *calibration_trigger(enum calibration_side side)
{
	return trigger_for_side(side);
}

const struct stick_calibration *calibration_stick(enum calibration_side side)
{
	return stick_for_side(side);
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

	*trigger_for_side(side) = loaded;
	if(side == CALIBRATION_RIGHT)
	{
		trigger_right_loaded = true;
	}
	else
	{
		trigger_left_loaded = true;
	}
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

	*stick_for_side(side) = loaded;
	if(side == CALIBRATION_RIGHT)
	{
		stick_right_loaded = true;
	}
	else
	{
		stick_left_loaded = true;
	}
	LOG_INF("Loaded %s stick calibration: x_min=%u, x_center=%u..%u, x_max=%u, y_min=%u, "
	        "y_center=%u..%u, y_max=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", loaded.x_min, loaded.x_center_min,
	        loaded.x_center_max, loaded.x_max, loaded.y_min, loaded.y_center_min,
	        loaded.y_center_max, loaded.y_max);
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
int calibration_import_valve_storage(void)
{
	const struct flash_area *fa;
	int err = flash_area_open(FLASH_AREA_ID(valve_storage), &fa);

	if(err)
	{
		LOG_WRN("Failed to open valve_storage flash area: %d", err);
		return err;
	}

	struct nvs_fs ofw_nvs = {
		.flash_device = fa->fa_dev,
		.offset = fa->fa_off,
		.sector_size = 4096,
		.sector_count = 3,
	};
	bool trg_l_found = false;
	bool trg_r_found = false;
	bool joy_l_found = false;
	bool joy_r_found = false;
	struct trigger_calibration ofw_trg_l = { 0 };
	struct trigger_calibration ofw_trg_r = { 0 };
	struct stick_calibration ofw_joy_l = { 0 };
	struct stick_calibration ofw_joy_r = { 0 };

	err = nvs_mount(&ofw_nvs);
	if(err)
	{
		LOG_INF("OFW valve_storage is uninitialized");
		flash_area_close(fa);
		return err;
	}

	LOG_INF("Scanning valve_storage for OFW calibration data...");
	for(uint16_t name_id = 0x8001; name_id < 0x8050; ++name_id)
	{
		char name[32];
		int rc = nvs_read(&ofw_nvs, name_id, name, sizeof(name) - 1);

		if(rc <= 0)
		{
			continue;
		}
		name[rc] = '\0';

		if(strcmp(name, "cal/trg_l") == 0)
		{
			rc = nvs_read(&ofw_nvs, name_id + 0x4000, &ofw_trg_l, sizeof(ofw_trg_l));
			trg_l_found = rc == sizeof(ofw_trg_l);
		}
		else if(strcmp(name, "cal/trg_r") == 0)
		{
			rc = nvs_read(&ofw_nvs, name_id + 0x4000, &ofw_trg_r, sizeof(ofw_trg_r));
			trg_r_found = rc == sizeof(ofw_trg_r);
		}
		else if(strcmp(name, "cal/joy_l") == 0)
		{
			rc = nvs_read(&ofw_nvs, name_id + 0x4000, &ofw_joy_l, sizeof(ofw_joy_l));
			joy_l_found = rc == sizeof(ofw_joy_l);
		}
		else if(strcmp(name, "cal/joy_r") == 0)
		{
			rc = nvs_read(&ofw_nvs, name_id + 0x4000, &ofw_joy_r, sizeof(ofw_joy_r));
			joy_r_found = rc == sizeof(ofw_joy_r);
		}
	}
	flash_area_close(fa);

	if(!trigger_left_loaded && trg_l_found && calibration_trigger_valid(&ofw_trg_l))
	{
		trigger_left = ofw_trg_l;
		trigger_left_loaded = true;
		err = save_setting("cal/trg_l", &ofw_trg_l, sizeof(ofw_trg_l));
		if(!err)
		{
			LOG_INF("Migrated cal/trg_l: pressed=%u, idle=%u, inverted=%u", ofw_trg_l.pressed,
			        ofw_trg_l.idle, ofw_trg_l.inverted);
		}
	}
	if(!trigger_right_loaded && trg_r_found && calibration_trigger_valid(&ofw_trg_r))
	{
		trigger_right = ofw_trg_r;
		trigger_right_loaded = true;
		err = save_setting("cal/trg_r", &ofw_trg_r, sizeof(ofw_trg_r));
		if(!err)
		{
			LOG_INF("Migrated cal/trg_r: pressed=%u, idle=%u, inverted=%u", ofw_trg_r.pressed,
			        ofw_trg_r.idle, ofw_trg_r.inverted);
		}
	}
	if(!stick_left_loaded && joy_l_found && calibration_stick_valid(&ofw_joy_l))
	{
		stick_left = ofw_joy_l;
		stick_left_loaded = true;
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
	if(!stick_right_loaded && joy_r_found && calibration_stick_valid(&ofw_joy_r))
	{
		stick_right = ofw_joy_r;
		stick_right_loaded = true;
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

	return 0;
}
#else
int calibration_import_valve_storage(void)
{
	return -ENODEV;
}
#endif

bool calibration_read_trigger(enum calibration_side side, uint8_t *buf, size_t capacity,
                              size_t *len)
{
	const struct trigger_calibration *value = trigger_for_side(side);

	if(capacity < sizeof(*value))
	{
		return false;
	}
	memcpy(buf, value, sizeof(*value));
	*len = sizeof(*value);
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

	if(side == CALIBRATION_LEFT)
	{
		trigger_left = staged;
		trigger_left_loaded = true;
		trigger_left_dirty = true;
	}
	else
	{
		trigger_right = staged;
		trigger_right_loaded = true;
		trigger_right_dirty = true;
	}

	LOG_INF("staged %s trigger calibration: pressed=%u, idle=%u, inverted=%u",
	        side == CALIBRATION_RIGHT ? "right" : "left", staged.pressed, staged.idle,
	        staged.inverted);
	return 0;
}

int calibration_commit_trigger(enum calibration_side side)
{
	const char *path;
	const void *value;
	bool *dirty;
	size_t len;
	int err;

	if(side == CALIBRATION_LEFT)
	{
		path = "cal/trg_l";
		value = &trigger_left;
		dirty = &trigger_left_dirty;
		len = sizeof(trigger_left);
	}
	else
	{
		path = "cal/trg_r";
		value = &trigger_right;
		dirty = &trigger_right_dirty;
		len = sizeof(trigger_right);
	}

	if(!*dirty)
	{
		return 0;
	}

	err = save_setting(path, value, len);
	if(err)
	{
		LOG_ERR("failed to commit %s: %d", path, err);
		return err;
	}

	*dirty = false;
	LOG_INF("committed %s", path);
	return 0;
}
