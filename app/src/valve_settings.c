/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>

#include "calibration.h"
#include "valve_settings.h"

LOG_MODULE_REGISTER(valve_settings);

#define VALVE_ESB_BOND_SIZE 24
#define VALVE_NVS_CHUNK_SIZE 48
#define VALVE_NVS_INFO_SIZE 8

static uint8_t esb_bond[VALVE_ESB_BOND_SIZE];
static bool esb_bond_valid;
static bool esb_bond_dirty;

static int load_esb_bond_cb(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg,
                            void *param)
{
	uint8_t *bond = param;
	ssize_t read_len;

	if(key != NULL && key[0] != '\0')
	{
		return 0;
	}
	if(len != sizeof(esb_bond))
	{
		return -EINVAL;
	}
	read_len = read_cb(cb_arg, bond, sizeof(esb_bond));
	if(read_len == sizeof(esb_bond))
	{
		esb_bond_valid = true;
	}
	return 1;
}

void valve_settings_load_feature_state(void)
{
	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load_subtree_direct("esb/bond", load_esb_bond_cb, esb_bond);
	}
}

static bool calibration_path_side(const char *path, enum calibration_side *side)
{
	if(strcmp(path, "cal/trg_l") == 0)
	{
		*side = CALIBRATION_LEFT;
		return true;
	}
	if(strcmp(path, "cal/trg_r") == 0)
	{
		*side = CALIBRATION_RIGHT;
		return true;
	}
	return false;
}

static bool nvs_offset_path(const char *path, off_t *offset)
{
#if FIXED_PARTITION_EXISTS(storage_partition)
	unsigned int parsed;
	char tail;

	if(sscanf(path, "nvs/%x%c", &parsed, &tail) != 1)
	{
		return false;
	}
	if(parsed >= FIXED_PARTITION_SIZE(storage_partition) || (parsed % sizeof(uint32_t)) != 0U)
	{
		return false;
	}

	*offset = parsed;
	return true;
#else
	ARG_UNUSED(path);
	ARG_UNUSED(offset);
	return false;
#endif
}

static bool nvs_read_path(const char *path, uint8_t *buf, size_t capacity, size_t *len)
{
#if FIXED_PARTITION_EXISTS(storage_partition)
	const struct flash_area *fa;
	off_t offset;
	size_t read_len;
	int err;

	if(strcmp(path, "nvs/info") == 0)
	{
		if(capacity < VALVE_NVS_INFO_SIZE)
		{
			return false;
		}
		sys_put_le32(FIXED_PARTITION_SIZE(storage_partition), &buf[0]);
		sys_put_le32(VALVE_NVS_CHUNK_SIZE, &buf[4]);
		*len = VALVE_NVS_INFO_SIZE;
		return true;
	}
	if(!nvs_offset_path(path, &offset) || capacity < VALVE_NVS_CHUNK_SIZE)
	{
		return false;
	}

	err = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
	if(err)
	{
		LOG_ERR("failed to open CFW NVS partition: %d", err);
		return false;
	}

	read_len = MIN(VALVE_NVS_CHUNK_SIZE, FIXED_PARTITION_SIZE(storage_partition) - offset);
	err = flash_area_read(fa, offset, buf, read_len);
	flash_area_close(fa);
	if(err)
	{
		LOG_ERR("failed to read CFW NVS offset 0x%04x: %d", (unsigned int)offset, err);
		return false;
	}

	*len = read_len;
	return true;
#else
	ARG_UNUSED(path);
	ARG_UNUSED(buf);
	ARG_UNUSED(capacity);
	ARG_UNUSED(len);
	return false;
#endif
}

bool valve_settings_read(const char *path, uint8_t *buf, size_t capacity, size_t *len)
{
	enum calibration_side side;

	if(strcmp(path, "esb/bond") == 0)
	{
		if(!esb_bond_valid || capacity < sizeof(esb_bond))
		{
			return false;
		}
		memcpy(buf, esb_bond, sizeof(esb_bond));
		*len = sizeof(esb_bond);
		return true;
	}

	if(nvs_read_path(path, buf, capacity, len))
	{
		return true;
	}

	if(calibration_path_side(path, &side))
	{
		return calibration_read_trigger(side, buf, capacity, len);
	}

	return false;
}

static int nvs_write_path(const char *path, const uint8_t *value, size_t len)
{
#if FIXED_PARTITION_EXISTS(storage_partition)
	const struct flash_area *fa;
	off_t offset;
	int err;

	if(!nvs_offset_path(path, &offset))
	{
		return -ENOENT;
	}
	if(len > VALVE_NVS_CHUNK_SIZE ||
	   offset + len > FIXED_PARTITION_SIZE(storage_partition) ||
	   (len % sizeof(uint32_t)) != 0U)
	{
		return -EINVAL;
	}

	err = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
	if(err)
	{
		return err;
	}
	err = flash_area_write(fa, offset, value, len);
	flash_area_close(fa);
	if(err)
	{
		LOG_ERR("failed to write CFW NVS offset 0x%04x: %d", (unsigned int)offset, err);
		return err;
	}
	return 0;
#else
	ARG_UNUSED(path);
	ARG_UNUSED(value);
	ARG_UNUSED(len);
	return -ENOENT;
#endif
}

int valve_settings_stage(const char *path, const uint8_t *value, size_t len)
{
	enum calibration_side side;
	int err;

	if(strcmp(path, "esb/bond") == 0)
	{
		if(len != sizeof(esb_bond))
		{
			return -EINVAL;
		}
		memcpy(esb_bond, value, sizeof(esb_bond));
		esb_bond_valid = true;
		esb_bond_dirty = true;
		LOG_INF("staged ESB bond provisioned over feature settings");
		return 0;
	}

	err = nvs_write_path(path, value, len);
	if(err != -ENOENT)
	{
		return err;
	}

	if(calibration_path_side(path, &side))
	{
		return calibration_stage_trigger(side, value, len);
	}

	return -ENOENT;
}

static int nvs_commit_path(const char *path)
{
#if FIXED_PARTITION_EXISTS(storage_partition)
	const struct flash_area *fa;
	int err;

	if(strcmp(path, "nvs/erase") != 0)
	{
		return -ENOENT;
	}

	err = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
	if(err)
	{
		return err;
	}
	err = flash_area_erase(fa, 0, FIXED_PARTITION_SIZE(storage_partition));
	flash_area_close(fa);
	if(err)
	{
		LOG_ERR("failed to erase CFW NVS partition: %d", err);
		return err;
	}
	LOG_INF("erased CFW NVS partition for feature restore");
	return 0;
#else
	ARG_UNUSED(path);
	return -ENOENT;
#endif
}

int valve_settings_commit(const char *path)
{
	enum calibration_side side;
	int err;

	if(strcmp(path, "esb/bond") == 0)
	{
		if(!esb_bond_dirty)
		{
			return 0;
		}
		err = settings_save_one("esb/bond", esb_bond, sizeof(esb_bond));
		if(err)
		{
			LOG_ERR("failed to commit ESB bond: %d", err);
			return err;
		}
		esb_bond_dirty = false;
		LOG_INF("committed ESB bond %08x/%08x", sys_get_le32(&esb_bond[0]),
		        sys_get_le32(&esb_bond[4]));
		return 0;
	}

	err = nvs_commit_path(path);
	if(err != -ENOENT)
	{
		return err;
	}

	if(calibration_path_side(path, &side))
	{
		return calibration_commit_trigger(side);
	}

	return -ENOENT;
}
