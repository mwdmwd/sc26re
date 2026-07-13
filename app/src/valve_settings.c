/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "calibration.h"
#include "ibex_settings_registry.h"
#include "imu.h"
#include "valve_settings.h"

LOG_MODULE_REGISTER(valve_settings);

#define VALVE_ESB_SLOT_0_TRANSPORT 2
#define VALVE_ESB_SLOT_1_TRANSPORT 3
#define VALVE_NVS_CHUNK_SIZE 48
#define VALVE_NVS_INFO_SIZE 8
#define VALVE_WIRELESS_TRANSPORT_PATH "user/wireless_transport"

static const char *const esb_bond_paths[VALVE_ESB_BOND_SLOT_COUNT] = {
	"esb/bond",
	"esb/bond_2",
};

struct esb_bond
{
	uint8_t data[VALVE_ESB_BOND_SIZE];
	bool valid;
};

struct esb_bond_slot
{
	struct esb_bond staged;
	struct esb_bond active;
	bool dirty;
};

static struct esb_bond_slot esb_bonds[VALVE_ESB_BOND_SLOT_COUNT];
static uint8_t wireless_transport;
static bool wireless_transport_valid;
static bool wireless_transport_dirty;
static struct k_spinlock active_esb_bond_lock;
static uint8_t active_wireless_transport;
static bool active_wireless_transport_valid;
static valve_settings_esb_bond_changed_cb_t esb_bond_changed_callback;

struct feature_state_load_target
{
	void *value;
	size_t value_size;
	bool *valid;
};

static int load_feature_state_cb(const char *key, size_t len, settings_read_cb read_cb,
                                 void *cb_arg, void *param)
{
	struct feature_state_load_target *target = param;
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
	if(read_len == (ssize_t)target->value_size)
	{
		*target->valid = true;
	}
	return 1;
}

static int esb_bond_slot(const char *path)
{
	for(size_t slot = 0; slot < ARRAY_SIZE(esb_bond_paths); ++slot)
	{
		if(strcmp(path, esb_bond_paths[slot]) == 0)
		{
			return (int)slot;
		}
	}
	return -1;
}

static int esb_bond_slot_for_transport(uint8_t transport)
{
	switch(transport)
	{
		case VALVE_ESB_SLOT_0_TRANSPORT:
			return 0;
		case VALVE_ESB_SLOT_1_TRANSPORT:
			return 1;
		default:
			return -1;
	}
}

static bool esb_bond_material_valid(const uint8_t *bond)
{
	/* The original transport considers a slot stored only when both UUID words are nonzero. */
	return sys_get_le32(&bond[0]) != 0U && sys_get_le32(&bond[4]) != 0U;
}

static bool esb_bond_available(size_t slot)
{
	const struct esb_bond *bond = &esb_bonds[slot].staged;

	return bond->valid && esb_bond_material_valid(bond->data);
}

static bool active_esb_bond_available_locked(size_t slot)
{
	const struct esb_bond *bond = &esb_bonds[slot].active;

	return bond->valid && esb_bond_material_valid(bond->data);
}

static void select_available_wireless_transport(void)
{
	int slot;

	/* Original firmware uses zero for Bluetooth, which intentionally selects no ESB slot. */
	if(wireless_transport_valid && wireless_transport == 0)
	{
		return;
	}

	slot = esb_bond_slot_for_transport(wireless_transport);
	if(wireless_transport_valid && slot >= 0 && esb_bond_available(slot))
	{
		return;
	}

	if(wireless_transport_valid)
	{
		LOG_WRN("wireless transport %u does not select an available ESB bond", wireless_transport);
	}
	wireless_transport_valid = false;

	for(size_t candidate = 0; candidate < ARRAY_SIZE(esb_bonds); ++candidate)
	{
		if(esb_bond_available(candidate))
		{
			wireless_transport =
			    candidate == 0 ? VALVE_ESB_SLOT_0_TRANSPORT : VALVE_ESB_SLOT_1_TRANSPORT;
			wireless_transport_valid = true;
			return;
		}
	}
}

static void notify_esb_bond_changed(void)
{
	if(esb_bond_changed_callback != NULL)
	{
		esb_bond_changed_callback();
	}
}

void valve_settings_load_feature_state(void)
{
	struct feature_state_load_target transport_target = {
		.value = &wireless_transport,
		.value_size = sizeof(wireless_transport),
		.valid = &wireless_transport_valid,
	};
	k_spinlock_key_t key;

	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		return;
	}

	for(size_t slot = 0; slot < ARRAY_SIZE(esb_bonds); ++slot)
	{
		struct esb_bond_slot *bond_slot = &esb_bonds[slot];
		struct feature_state_load_target target = {
			.value = bond_slot->staged.data,
			.value_size = sizeof(bond_slot->staged.data),
			.valid = &bond_slot->staged.valid,
		};

		bond_slot->staged.valid = false;
		(void)settings_load_subtree_direct(esb_bond_paths[slot], load_feature_state_cb, &target);
		bond_slot->dirty = false;
		if(bond_slot->staged.valid && !esb_bond_material_valid(bond_slot->staged.data))
		{
			LOG_WRN("ESB bond slot %u has no usable key material", (unsigned int)slot);
		}
	}

	wireless_transport_valid = false;
	(void)settings_load_subtree_direct(VALVE_WIRELESS_TRANSPORT_PATH, load_feature_state_cb,
	                                   &transport_target);
	select_available_wireless_transport();
	wireless_transport_dirty = false;
	key = k_spin_lock(&active_esb_bond_lock);
	for(size_t slot = 0; slot < ARRAY_SIZE(esb_bonds); ++slot)
	{
		esb_bonds[slot].active = esb_bonds[slot].staged;
	}
	active_wireless_transport = wireless_transport;
	active_wireless_transport_valid = wireless_transport_valid;
	k_spin_unlock(&active_esb_bond_lock, key);
	notify_esb_bond_changed();
}

bool valve_settings_copy_active_esb_bond(uint8_t *bond, size_t capacity, uint8_t *slot)
{
	k_spinlock_key_t key;
	int active_slot;
	bool available;

	if(bond == NULL || capacity < VALVE_ESB_BOND_SIZE)
	{
		return false;
	}

	key = k_spin_lock(&active_esb_bond_lock);
	if(!active_wireless_transport_valid)
	{
		k_spin_unlock(&active_esb_bond_lock, key);
		return false;
	}
	active_slot = esb_bond_slot_for_transport(active_wireless_transport);
	available = active_slot >= 0 && active_esb_bond_available_locked(active_slot);
	if(available)
	{
		memcpy(bond, esb_bonds[active_slot].active.data, VALVE_ESB_BOND_SIZE);
	}
	k_spin_unlock(&active_esb_bond_lock, key);
	if(!available)
	{
		return false;
	}
	if(slot != NULL)
	{
		*slot = (uint8_t)active_slot;
	}
	return true;
}

int valve_settings_register_esb_bond_changed_callback(valve_settings_esb_bond_changed_cb_t callback)
{
	if(callback == NULL)
	{
		return -EINVAL;
	}
	if(esb_bond_changed_callback != NULL && esb_bond_changed_callback != callback)
	{
		return -EALREADY;
	}

	esb_bond_changed_callback = callback;
	callback();
	return 0;
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

static bool pressure_path_side(const char *path, enum calibration_side *side)
{
	if(strcmp(path, "cal/prs_l") == 0)
	{
		*side = CALIBRATION_LEFT;
		return true;
	}
	if(strcmp(path, "cal/prs_r") == 0)
	{
		*side = CALIBRATION_RIGHT;
		return true;
	}
	return false;
}

static bool ibex_setting_path_id(const char *path, uint8_t *id)
{
	for(uint8_t candidate = 0; candidate < IBEX_SETTING_COUNT; ++candidate)
	{
		const char *candidate_path = ibex_setting_persist_path(candidate);

		if(candidate_path != NULL && strcmp(path, candidate_path) == 0)
		{
			*id = candidate;
			return true;
		}
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
	uint8_t setting_id;
	int16_t setting_value;
	int slot = esb_bond_slot(path);

	if(slot >= 0)
	{
		const struct esb_bond *bond = &esb_bonds[slot].staged;

		if(!bond->valid || capacity < sizeof(bond->data))
		{
			return false;
		}
		memcpy(buf, bond->data, sizeof(bond->data));
		*len = sizeof(bond->data);
		return true;
	}

	if(strcmp(path, VALVE_WIRELESS_TRANSPORT_PATH) == 0)
	{
		if(!wireless_transport_valid || capacity < sizeof(wireless_transport))
		{
			return false;
		}
		buf[0] = wireless_transport;
		*len = sizeof(wireless_transport);
		return true;
	}

	if(nvs_read_path(path, buf, capacity, len))
	{
		return true;
	}

	if(ibex_setting_path_id(path, &setting_id))
	{
		if(capacity < sizeof(setting_value) || !ibex_setting_get(setting_id, &setting_value))
		{
			return false;
		}
		sys_put_le16((uint16_t)setting_value, buf);
		*len = sizeof(setting_value);
		return true;
	}

	if(calibration_path_side(path, &side))
	{
		return calibration_read_trigger(side, buf, capacity, len);
	}
	if(pressure_path_side(path, &side))
	{
		return calibration_read_pressure(side, buf, capacity, len);
	}

	if(imu_settings_read(path, buf, capacity, len))
	{
		return true;
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
	uint8_t setting_id;
	int slot = esb_bond_slot(path);
	int err;

	if(slot >= 0)
	{
		struct esb_bond_slot *bond_slot = &esb_bonds[slot];

		if(len != sizeof(bond_slot->staged.data))
		{
			return -EINVAL;
		}
		memcpy(bond_slot->staged.data, value, sizeof(bond_slot->staged.data));
		bond_slot->staged.valid = true;
		bond_slot->dirty = true;
		LOG_INF("staged ESB bond slot %d over feature settings", slot);
		return 0;
	}

	if(strcmp(path, VALVE_WIRELESS_TRANSPORT_PATH) == 0)
	{
		k_spinlock_key_t key;
		int selected_slot;
		bool selected_bond_available;

		if(len != sizeof(wireless_transport))
		{
			return -EINVAL;
		}
		selected_slot = esb_bond_slot_for_transport(value[0]);
		key = k_spin_lock(&active_esb_bond_lock);
		selected_bond_available =
		    selected_slot >= 0 && active_esb_bond_available_locked(selected_slot);
		k_spin_unlock(&active_esb_bond_lock, key);
		if(value[0] != 0 && !selected_bond_available)
		{
			return -EINVAL;
		}
		wireless_transport = value[0];
		wireless_transport_valid = true;
		wireless_transport_dirty = true;
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
	if(pressure_path_side(path, &side))
	{
		return calibration_stage_pressure(side, value, len);
	}

	if(ibex_setting_path_id(path, &setting_id))
	{
		if(len != sizeof(int16_t))
		{
			return -EINVAL;
		}
		return ibex_setting_set(setting_id, (int16_t)sys_get_le16(value));
	}

	err = imu_settings_stage(path, value, len);
	if(err != -ENOENT)
	{
		return err;
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
	uint8_t setting_id;
	uint8_t encoded[sizeof(int16_t)];
	int16_t setting_value;
	int slot = esb_bond_slot(path);
	int err;

	if(slot >= 0)
	{
		struct esb_bond_slot *bond_slot = &esb_bonds[slot];
		k_spinlock_key_t key;

		if(!bond_slot->dirty)
		{
			return 0;
		}
		err = settings_save_one(esb_bond_paths[slot], bond_slot->staged.data,
		                        sizeof(bond_slot->staged.data));
		if(err)
		{
			LOG_ERR("failed to commit ESB bond slot %d: %d", slot, err);
			return err;
		}
		bond_slot->dirty = false;
		key = k_spin_lock(&active_esb_bond_lock);
		bond_slot->active = bond_slot->staged;
		k_spin_unlock(&active_esb_bond_lock, key);
		LOG_INF("committed ESB bond slot %d %08x/%08x", slot,
		        sys_get_le32(&bond_slot->staged.data[0]), sys_get_le32(&bond_slot->staged.data[4]));
		notify_esb_bond_changed();
		return 0;
	}

	if(strcmp(path, VALVE_WIRELESS_TRANSPORT_PATH) == 0)
	{
		k_spinlock_key_t key;

		if(!wireless_transport_dirty)
		{
			return 0;
		}
		err = settings_save_one(VALVE_WIRELESS_TRANSPORT_PATH, &wireless_transport,
		                        sizeof(wireless_transport));
		if(err)
		{
			LOG_ERR("failed to commit wireless transport: %d", err);
			return err;
		}
		wireless_transport_dirty = false;
		key = k_spin_lock(&active_esb_bond_lock);
		active_wireless_transport = wireless_transport;
		active_wireless_transport_valid = wireless_transport_valid;
		k_spin_unlock(&active_esb_bond_lock, key);
		notify_esb_bond_changed();
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
	if(pressure_path_side(path, &side))
	{
		return calibration_commit_pressure(side);
	}

	if(ibex_setting_path_id(path, &setting_id))
	{
		if(!ibex_setting_get(setting_id, &setting_value))
		{
			return -EINVAL;
		}
		sys_put_le16((uint16_t)setting_value, encoded);
		return settings_save_one(path, encoded, sizeof(encoded));
	}

	err = imu_settings_commit(path);
	if(err != -ENOENT)
	{
		return err;
	}

	return -ENOENT;
}

int valve_settings_save_esb_bond(uint8_t slot, const uint8_t *bond, size_t len, bool select)
{
	k_spinlock_key_t key;
	uint8_t transport;
	int err;

	if(slot >= ARRAY_SIZE(esb_bonds) ||
	   bond == NULL ||
	   len != VALVE_ESB_BOND_SIZE ||
	   (select && !esb_bond_material_valid(bond)))
	{
		return -EINVAL;
	}

	err = valve_settings_stage(esb_bond_paths[slot], bond, len);
	if(err)
	{
		return err;
	}
	if(!IS_ENABLED(CONFIG_SETTINGS))
	{
		struct esb_bond_slot *bond_slot = &esb_bonds[slot];

		bond_slot->dirty = false;
		key = k_spin_lock(&active_esb_bond_lock);
		bond_slot->active = bond_slot->staged;
		if(select)
		{
			transport = slot == 0 ? VALVE_ESB_SLOT_0_TRANSPORT : VALVE_ESB_SLOT_1_TRANSPORT;
			wireless_transport = transport;
			wireless_transport_valid = true;
			wireless_transport_dirty = false;
			active_wireless_transport = transport;
			active_wireless_transport_valid = true;
		}
		k_spin_unlock(&active_esb_bond_lock, key);
		notify_esb_bond_changed();
		return 0;
	}
	err = valve_settings_commit(esb_bond_paths[slot]);
	if(err || !select)
	{
		return err;
	}

	transport = slot == 0 ? VALVE_ESB_SLOT_0_TRANSPORT : VALVE_ESB_SLOT_1_TRANSPORT;
	err = valve_settings_stage(VALVE_WIRELESS_TRANSPORT_PATH, &transport, sizeof(transport));
	if(err)
	{
		return err;
	}
	return valve_settings_commit(VALVE_WIRELESS_TRANSPORT_PATH);
}
