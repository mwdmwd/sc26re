/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include "valve_nvs.h"

/*
 * Valve's firmware uses the Zephyr 3.7.99/NCS 2.9 NVS format. Data grows up
 * from the start of each sector while eight-byte allocation table entries
 * (ATEs) grow down from its end. This reader fixes that geometry and format
 * independently of the CFW's Zephyr version and never mounts the OFW
 * partition, avoiding mount-time repair, garbage collection, or erasure.
 */
#define VALVE_NVS_ATE_SIZE 8U
#define VALVE_NVS_WRITE_BLOCK_SIZE 4U
#define VALVE_NVS_ERASE_VALUE 0xffU
#define VALVE_NVS_SPECIAL_ID 0xffffU
#define VALVE_NVS_SECTOR_SHIFT 16U
#define VALVE_NVS_OFFSET_MASK 0xffffU
#define VALVE_NVS_SETTINGS_NAME_COUNT_ID 0x8000U
#define VALVE_NVS_SETTINGS_VALUE_ID_OFFSET 0x4000U
#define VALVE_NVS_SETTINGS_LAST_NAME_ID_MAX 0xbffeU
#define VALVE_NVS_SETTINGS_NAME_MAX 128U
#define VALVE_NVS_CLOSE_OFFSET (VALVE_NVS_SECTOR_SIZE - VALVE_NVS_ATE_SIZE)
#define VALVE_NVS_FIRST_ATE_OFFSET (VALVE_NVS_SECTOR_SIZE - (2U * VALVE_NVS_ATE_SIZE))
#define VALVE_NVS_MAX_ATE_STEPS \
	((VALVE_NVS_SECTOR_COUNT * VALVE_NVS_SECTOR_SIZE) / VALVE_NVS_ATE_SIZE)

struct valve_nvs_ate
{
	uint16_t id;
	uint16_t offset;
	uint16_t len;
	uint8_t part;
	uint8_t crc8;
	uint8_t raw[VALVE_NVS_ATE_SIZE];
};

static uint16_t get_le16(const uint8_t *data)
{
	return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint8_t ate_crc8(const uint8_t *data)
{
	uint8_t crc = 0xffU;

	for(size_t byte = 0; byte < VALVE_NVS_ATE_SIZE - 1U; ++byte)
	{
		crc ^= data[byte];
		for(unsigned int bit = 0; bit < 8U; ++bit)
		{
			crc = (crc & 0x80U) != 0U ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
		}
	}

	return crc;
}

static uint32_t nvs_address(uint16_t sector, uint16_t offset)
{
	return ((uint32_t)sector << VALVE_NVS_SECTOR_SHIFT) | offset;
}

static uint16_t address_sector(uint32_t address)
{
	return (uint16_t)(address >> VALVE_NVS_SECTOR_SHIFT);
}

static uint16_t address_offset(uint32_t address)
{
	return (uint16_t)(address & VALVE_NVS_OFFSET_MASK);
}

static int read_bytes(const struct valve_nvs *nvs, uint32_t address, void *data, size_t len)
{
	uint16_t sector = address_sector(address);
	uint16_t offset = address_offset(address);

	if(sector >= VALVE_NVS_SECTOR_COUNT ||
	   offset > VALVE_NVS_SECTOR_SIZE ||
	   len > VALVE_NVS_SECTOR_SIZE - offset)
	{
		return -EIO;
	}
	if(len == 0U)
	{
		return 0;
	}

	return nvs->read(nvs->context, (uint32_t)sector * VALVE_NVS_SECTOR_SIZE + offset, data, len);
}

static int read_ate(const struct valve_nvs *nvs, uint32_t address, struct valve_nvs_ate *ate)
{
	int err = read_bytes(nvs, address, ate->raw, sizeof(ate->raw));

	if(err)
	{
		return err;
	}

	ate->id = get_le16(&ate->raw[0]);
	ate->offset = get_le16(&ate->raw[2]);
	ate->len = get_le16(&ate->raw[4]);
	ate->part = ate->raw[6];
	ate->crc8 = ate->raw[7];
	if(ate->part != VALVE_NVS_ERASE_VALUE && ate_crc8(ate->raw) == ate->crc8)
	{
		/* Valve's NVS version has no multipart records. */
		return -EIO;
	}
	return 0;
}

static bool ate_erased(const struct valve_nvs_ate *ate)
{
	for(size_t i = 0; i < sizeof(ate->raw); ++i)
	{
		if(ate->raw[i] != VALVE_NVS_ERASE_VALUE)
		{
			return false;
		}
	}

	return true;
}

static bool ate_valid(uint32_t address, const struct valve_nvs_ate *ate)
{
	uint32_t data_end = (uint32_t)ate->offset + ate->len;

	/* Valve does not use the reserved multipart extension. */
	return ate->part == VALVE_NVS_ERASE_VALUE &&
	       ate_crc8(ate->raw) == ate->crc8 &&
	       ate->offset % VALVE_NVS_WRITE_BLOCK_SIZE == 0U &&
	       data_end < VALVE_NVS_CLOSE_OFFSET &&
	       data_end <= address_offset(address);
}

static bool close_ate_valid(uint32_t address, const struct valve_nvs_ate *ate)
{
	return ate_valid(address, ate) &&
	       ate->id == VALVE_NVS_SPECIAL_ID &&
	       ate->len == 0U &&
	       (VALVE_NVS_SECTOR_SIZE - ate->offset) % VALVE_NVS_ATE_SIZE == 0U;
}

static void advance_sector(uint32_t *address)
{
	uint16_t sector = (uint16_t)((address_sector(*address) + 1U) % VALVE_NVS_SECTOR_COUNT);

	*address = nvs_address(sector, address_offset(*address));
}

static void previous_sector(uint32_t *address)
{
	uint16_t sector = address_sector(*address);

	sector = sector == 0U ? VALVE_NVS_SECTOR_COUNT - 1U : sector - 1U;
	*address = nvs_address(sector, address_offset(*address));
}

/*
 * Recover the lowest-addressed valid ATE in a sector. This is the read-only
 * equivalent of Zephyr 3.7.99's interrupted-ATE recovery: corrupt entries are
 * ignored, but the data in the flash is not repaired.
 */
static int recover_last_ate(const struct valve_nvs *nvs, uint32_t *address)
{
	uint32_t sector_base = nvs_address(address_sector(*address), 0U);
	uint32_t scan;
	uint32_t data_end = sector_base;
	struct valve_nvs_ate ate;
	int err;

	if(address_offset(*address) < VALVE_NVS_ATE_SIZE)
	{
		return -EIO;
	}

	*address -= VALVE_NVS_ATE_SIZE;
	scan = *address;
	while(scan > data_end)
	{
		uint32_t candidate_end;

		err = read_ate(nvs, scan, &ate);
		if(err)
		{
			return err;
		}
		if(ate_valid(scan, &ate))
		{
			candidate_end = sector_base + ate.offset + ate.len;
			if(candidate_end < data_end)
			{
				return -EIO;
			}
			data_end = candidate_end;
			*address = scan;
		}

		if(address_offset(scan) < VALVE_NVS_ATE_SIZE)
		{
			break;
		}
		scan -= VALVE_NVS_ATE_SIZE;
	}

	return 0;
}

static int validate_closed_sector(const struct valve_nvs *nvs, uint32_t close_address,
                                  const struct valve_nvs_ate *close_ate)
{
	struct valve_nvs_ate newest_ate;
	uint32_t newest_address = close_address;
	int err;

	if(!close_ate_valid(close_address, close_ate))
	{
		return -EIO;
	}
	err = recover_last_ate(nvs, &newest_address);
	if(err)
	{
		return err;
	}
	if(address_offset(newest_address) != close_ate->offset)
	{
		return -EIO;
	}
	err = read_ate(nvs, newest_address, &newest_ate);
	if(err)
	{
		return err;
	}
	return ate_valid(newest_address, &newest_ate) ? 0 : -EIO;
}

/* Read the current ATE and advance the cursor toward older records. */
static int previous_ate(const struct valve_nvs *nvs, uint32_t *address, struct valve_nvs_ate *ate)
{
	struct valve_nvs_ate close_ate;
	int err = read_ate(nvs, *address, ate);

	if(err)
	{
		return err;
	}

	*address += VALVE_NVS_ATE_SIZE;
	if(address_offset(*address) != VALVE_NVS_CLOSE_OFFSET)
	{
		return 0;
	}

	previous_sector(address);
	err = read_ate(nvs, *address, &close_ate);
	if(err)
	{
		return err;
	}
	if(ate_erased(&close_ate))
	{
		if((nvs->closed_mask & (1U << address_sector(*address))) != 0U)
		{
			return -EIO;
		}
		*address = nvs->next_ate;
		return 0;
	}
	if((nvs->closed_mask & (1U << address_sector(*address))) != 0U &&
	   close_ate_valid(*address, &close_ate) &&
	   close_ate.offset == nvs->close_offsets[address_sector(*address)])
	{
		*address = nvs_address(address_sector(*address), close_ate.offset);
		return 0;
	}

	return -EIO;
}

static int range_erased(const struct valve_nvs *nvs, uint32_t address, size_t len, bool *erased)
{
	uint8_t buffer[32];

	*erased = true;
	while(len > 0U)
	{
		size_t chunk = len < sizeof(buffer) ? len : sizeof(buffer);
		int err = read_bytes(nvs, address, buffer, chunk);

		if(err)
		{
			return err;
		}
		for(size_t i = 0; i < chunk; ++i)
		{
			if(buffer[i] != VALVE_NVS_ERASE_VALUE)
			{
				*erased = false;
				return 0;
			}
		}
		address += (uint32_t)chunk;
		len -= chunk;
	}

	return 0;
}

int valve_nvs_open(struct valve_nvs *nvs, valve_nvs_read_cb_t read, const void *context,
                   size_t size)
{
	struct valve_nvs_ate ate;
	uint32_t address = 0U;
	uint32_t data_end;
	uint16_t active_sector = 0U;
	uint16_t closed_sectors = 0U;
	uint16_t sector;
	bool erased;
	int err;

	if(nvs == NULL)
	{
		return -EINVAL;
	}
	nvs->ready = false;
	if(read == NULL || size != VALVE_NVS_SIZE)
	{
		return -EINVAL;
	}

	nvs->read = read;
	nvs->context = context;
	nvs->next_ate = 0U;
	nvs->closed_mask = 0U;
	memset(nvs->close_offsets, 0, sizeof(nvs->close_offsets));

	/* Validate every closed sector before trusting its traversal pointer. */
	for(sector = 0U; sector < VALVE_NVS_SECTOR_COUNT; ++sector)
	{
		address = nvs_address(sector, VALVE_NVS_CLOSE_OFFSET);
		err = read_ate(nvs, address, &ate);
		if(err)
		{
			return err;
		}
		if(!ate_erased(&ate))
		{
			err = validate_closed_sector(nvs, address, &ate);
			if(err)
			{
				return err;
			}
			nvs->closed_mask |= (uint8_t)(1U << sector);
			nvs->close_offsets[sector] = ate.offset;
			++closed_sectors;
		}
	}

	if(closed_sectors == VALVE_NVS_SECTOR_COUNT)
	{
		return -EIO;
	}
	if(closed_sectors != 0U)
	{
		for(sector = 0U; sector < VALVE_NVS_SECTOR_COUNT; ++sector)
		{
			uint16_t next = (uint16_t)((sector + 1U) % VALVE_NVS_SECTOR_COUNT);

			if((nvs->closed_mask & (1U << sector)) != 0U && (nvs->closed_mask & (1U << next)) == 0U)
			{
				active_sector = next;
				break;
			}
		}
	}
	else
	{
		/* Zephyr 3.7 selects sector 0 unless sector 2 already has ATEs. */
		address = nvs_address(VALVE_NVS_SECTOR_COUNT - 1U, VALVE_NVS_FIRST_ATE_OFFSET);
		err = read_ate(nvs, address, &ate);
		if(err)
		{
			return err;
		}
		active_sector = ate_erased(&ate) ? 0U : VALVE_NVS_SECTOR_COUNT - 1U;
	}
	address = nvs_address(active_sector, VALVE_NVS_CLOSE_OFFSET);

	err = recover_last_ate(nvs, &address);
	if(err)
	{
		return err;
	}

	/* Locate the first erased ATE below the newest valid record. */
	nvs->next_ate = address;
	data_end = nvs_address(address_sector(address), 0U);
	while(nvs->next_ate >= data_end)
	{
		err = read_ate(nvs, nvs->next_ate, &ate);
		if(err)
		{
			return err;
		}
		if(ate_erased(&ate))
		{
			break;
		}
		if(ate_valid(nvs->next_ate, &ate))
		{
			uint32_t aligned_end = (uint32_t)ate.offset + ate.len;

			aligned_end = (aligned_end + VALVE_NVS_WRITE_BLOCK_SIZE - 1U) &
			              ~(VALVE_NVS_WRITE_BLOCK_SIZE - 1U);
			data_end = nvs_address(address_sector(address), (uint16_t)aligned_end);
			if(nvs->next_ate == data_end && ate.len != 0U)
			{
				return -EIO;
			}
		}
		if(address_offset(nvs->next_ate) < VALVE_NVS_ATE_SIZE)
		{
			return -EIO;
		}
		nvs->next_ate -= VALVE_NVS_ATE_SIZE;
	}

	/*
	 * A non-empty sector after the active one means GC was interrupted.
	 * Zephyr's mount repairs or erases it, this reader fails.
	 */
	address = nvs_address(address_sector(nvs->next_ate), 0U);
	advance_sector(&address);
	err = range_erased(nvs, address, VALVE_NVS_SECTOR_SIZE, &erased);
	if(err)
	{
		return err;
	}
	if(!erased)
	{
		return -EIO;
	}
	if(closed_sectors == 0U)
	{
		for(sector = 0U; sector < VALVE_NVS_SECTOR_COUNT; ++sector)
		{
			if(sector == active_sector || sector == address_sector(address))
			{
				continue;
			}
			address = nvs_address(sector, 0U);
			err = range_erased(nvs, address, VALVE_NVS_SECTOR_SIZE, &erased);
			if(err)
			{
				return err;
			}
			if(!erased)
			{
				return -EIO;
			}
		}
	}

	/* Reject an otherwise empty active sector containing orphaned data. */
	if(address_offset(nvs->next_ate) == VALVE_NVS_FIRST_ATE_OFFSET)
	{
		address = nvs_address(address_sector(nvs->next_ate), 0U);
		err = range_erased(nvs, address, VALVE_NVS_FIRST_ATE_OFFSET, &erased);
		if(err)
		{
			return err;
		}
		if(!erased)
		{
			return -EIO;
		}
	}

	nvs->ready = true;
	return 0;
}

int valve_nvs_read(const struct valve_nvs *nvs, uint16_t id, void *data, size_t capacity)
{
	struct valve_nvs_ate ate;
	uint32_t address;
	size_t steps = 0U;
	int err;

	if(nvs == NULL || !nvs->ready)
	{
		return -EACCES;
	}
	if((data == NULL && capacity != 0U) || id == VALVE_NVS_SPECIAL_ID)
	{
		return -EINVAL;
	}

	address = nvs->next_ate;
	do
	{
		uint32_t entry_address = address;

		err = previous_ate(nvs, &address, &ate);
		if(err)
		{
			return err;
		}
		if(ate_valid(entry_address, &ate) && ate.id == id)
		{
			size_t copied = capacity < ate.len ? capacity : ate.len;

			if(ate.len == 0U)
			{
				return -ENOENT;
			}
			err = read_bytes(nvs, nvs_address(address_sector(entry_address), ate.offset), data,
			                 copied);
			return err ? err : ate.len;
		}
		++steps;
	} while(address != nvs->next_ate && steps < VALVE_NVS_MAX_ATE_STEPS);

	return address == nvs->next_ate ? -ENOENT : -EIO;
}

int valve_nvs_read_setting(const struct valve_nvs *nvs, const char *name, void *data,
                           size_t capacity)
{
	uint8_t last_name_id_bytes[sizeof(uint16_t)];
	char stored_name[VALVE_NVS_SETTINGS_NAME_MAX];
	uint16_t last_name_id;
	size_t name_len;
	bool matched = false;
	int value_len = -ENOENT;
	int err;

	if(name == NULL || name[0] == '\0')
	{
		return -EINVAL;
	}

	err = valve_nvs_read(nvs, VALVE_NVS_SETTINGS_NAME_COUNT_ID, last_name_id_bytes,
	                     sizeof(last_name_id_bytes));
	if(err == -ENOENT)
	{
		return -ENOENT;
	}
	if(err != sizeof(last_name_id_bytes))
	{
		return err < 0 ? err : -EIO;
	}

	last_name_id = get_le16(last_name_id_bytes);
	if(last_name_id < VALVE_NVS_SETTINGS_NAME_COUNT_ID ||
	   last_name_id > VALVE_NVS_SETTINGS_LAST_NAME_ID_MAX)
	{
		return -EIO;
	}

	name_len = strlen(name);
	if(name_len > sizeof(stored_name))
	{
		return -ENAMETOOLONG;
	}

	for(uint16_t name_id = last_name_id; name_id > VALVE_NVS_SETTINGS_NAME_COUNT_ID; --name_id)
	{
		err = valve_nvs_read(nvs, name_id, stored_name, sizeof(stored_name));
		if(err < 0)
		{
			if(err == -ENOENT)
			{
				continue;
			}
			return err;
		}
		if((size_t)err != name_len || memcmp(stored_name, name, name_len) != 0)
		{
			continue;
		}
		if(matched)
		{
			/* Multiple IDs for one name are ambiguous. */
			return -EIO;
		}

		matched = true;
		value_len = valve_nvs_read(nvs, (uint16_t)(name_id + VALVE_NVS_SETTINGS_VALUE_ID_OFFSET),
		                           data, capacity);
		if(value_len < 0 && value_len != -ENOENT)
		{
			return value_len;
		}
	}

	return value_len;
}
