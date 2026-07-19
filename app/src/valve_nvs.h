/* SPDX-License-Identifier: AGPL-3.0-or-later */
#ifndef VALVE_NVS_H
#define VALVE_NVS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VALVE_NVS_SECTOR_SIZE 4096U
#define VALVE_NVS_SECTOR_COUNT 3U
#define VALVE_NVS_SIZE (VALVE_NVS_SECTOR_SIZE * VALVE_NVS_SECTOR_COUNT)

typedef int (*valve_nvs_read_cb_t)(const void *context, uint32_t offset, void *data, size_t len);

struct valve_nvs
{
	valve_nvs_read_cb_t read;
	const void *context;
	uint32_t next_ate;
	uint16_t close_offsets[VALVE_NVS_SECTOR_COUNT];
	uint8_t closed_mask;
	bool ready;
};

/*
 * Open Valve's Zephyr 3.7.99 NVS image without modifying it. The callback
 * follows flash_area_read() semantics: zero is success and errors are negative.
 */
int valve_nvs_open(struct valve_nvs *nvs, valve_nvs_read_cb_t read, const void *context,
                   size_t size);

/* Return the complete record length, even when only capacity bytes were copied. */
int valve_nvs_read(const struct valve_nvs *nvs, uint16_t id, void *data, size_t capacity);

/* Read a value by its legacy Zephyr settings name. */
int valve_nvs_read_setting(const struct valve_nvs *nvs, const char *name, void *data,
                           size_t capacity);

#endif
