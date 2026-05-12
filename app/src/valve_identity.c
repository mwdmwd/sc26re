/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <ctype.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>

#include "valve_identity.h"

#define VALVE_UICR_CUSTOM_BASE 0x10001080u
#define VALVE_UICR_MAGIC 0xac32a429u
#define VALVE_UICR_UNIT_SERIAL_OFFSET 8
#define VALVE_UICR_BOARD_SERIAL_OFFSET 24

static char unit_serial[VALVE_IDENTITY_SERIAL_TEXT_SIZE + 1];
static char board_serial[VALVE_IDENTITY_SERIAL_TEXT_SIZE + 1];
static char synthetic_serial[VALVE_IDENTITY_SERIAL_TEXT_SIZE + 1];
static bool synthetic_serial_valid;

static bool valid_ofw_serial(const char *serial)
{
	for(size_t i = 0; i < VALVE_IDENTITY_SERIAL_TEXT_SIZE; ++i)
	{
		if(!isalnum((unsigned char)serial[i]))
		{
			return false;
		}
	}

	return serial[VALVE_IDENTITY_SERIAL_TEXT_SIZE] == '\0';
}

static const char *read_uicr_serial(size_t offset, char *buffer)
{
	const uint8_t *uicr = (const uint8_t *)VALVE_UICR_CUSTOM_BASE;
	const uint32_t *magic = (const uint32_t *)VALVE_UICR_CUSTOM_BASE;

	if(*magic != VALVE_UICR_MAGIC)
	{
		return VALVE_IDENTITY_SERIAL_PLACEHOLDER;
	}

	memcpy(buffer, &uicr[offset], VALVE_IDENTITY_SERIAL_TEXT_SIZE + 1);
	if(!valid_ofw_serial(buffer))
	{
		return VALVE_IDENTITY_SERIAL_PLACEHOLDER;
	}

	return buffer;
}

static uint32_t fnv1a32(const uint8_t *data, size_t len)
{
	uint32_t hash = 2166136261u;

	for(size_t i = 0; i < len; ++i)
	{
		hash ^= data[i];
		hash *= 16777619u;
	}

	return hash;
}

static void format_non_ibex_serial(uint32_t hash)
{
	static const char hex[] = "0123456789ABCDEF";

	memcpy(synthetic_serial, "MBIT0", 5);
	for(size_t i = 0; i < 8; ++i)
	{
		synthetic_serial[5 + i] = hex[(hash >> ((7 - i) * 4)) & 0xf];
	}
	synthetic_serial[VALVE_IDENTITY_SERIAL_TEXT_SIZE] = '\0';
	synthetic_serial_valid = true;
}

static const char *non_ibex_serial(void)
{
	uint8_t id[16];
	ssize_t id_len;

	if(synthetic_serial_valid)
	{
		return synthetic_serial;
	}

	if(!IS_ENABLED(CONFIG_HWINFO))
	{
		return VALVE_IDENTITY_SERIAL_PLACEHOLDER;
	}

	id_len = hwinfo_get_device_id(id, sizeof(id));
	if(id_len <= 0)
	{
		return VALVE_IDENTITY_SERIAL_PLACEHOLDER;
	}

	format_non_ibex_serial(fnv1a32(id, (size_t)id_len));
	return synthetic_serial;
}

const char *valve_identity_serial(enum valve_identity_serial which)
{
	if(!IS_ENABLED(CONFIG_BOARD_STEAM_CONTROLLER_IBEX))
	{
		return non_ibex_serial();
	}

	switch(which)
	{
		case VALVE_IDENTITY_BOARD_SERIAL:
			return read_uicr_serial(VALVE_UICR_BOARD_SERIAL_OFFSET, board_serial);
		case VALVE_IDENTITY_UNIT_SERIAL:
		default:
			return read_uicr_serial(VALVE_UICR_UNIT_SERIAL_OFFSET, unit_serial);
	}
}

const char *valve_identity_board_id(void)
{
	return VALVE_IDENTITY_BOARD_ID;
}

void valve_identity_copy_serial(enum valve_identity_serial which, uint8_t *dest, size_t dest_size)
{
	const char *serial = valve_identity_serial(which);
	size_t copy_len = MIN(strlen(serial), dest_size);

	memset(dest, 0, dest_size);
	memcpy(dest, serial, copy_len);
}

#if defined(CONFIG_USB_DEVICE_STACK)
uint8_t *usb_update_sn_string_descriptor(void)
{
	static uint8_t serial[VALVE_IDENTITY_SERIAL_MAX_TEXT_SIZE + 1];
	const char *identity_serial = valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL);

	memset(serial, 0, sizeof(serial));
	memcpy(serial, identity_serial, MIN(strlen(identity_serial), sizeof(serial) - 1));
	return serial;
}
#endif
