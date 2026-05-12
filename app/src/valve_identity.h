/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define VALVE_IDENTITY_SERIAL_SIZE 16
#define VALVE_IDENTITY_SERIAL_MAX_TEXT_SIZE 16
#define VALVE_IDENTITY_SERIAL_TEXT_SIZE 13
#define VALVE_IDENTITY_SERIAL_PLACEHOLDER "XXXXXXXXXXXXX"
#define VALVE_IDENTITY_BOARD_ID "microbit-v2"

enum valve_identity_serial
{
	VALVE_IDENTITY_BOARD_SERIAL,
	VALVE_IDENTITY_UNIT_SERIAL,
};

const char *valve_identity_serial(enum valve_identity_serial which);
const char *valve_identity_board_id(void);
void valve_identity_copy_serial(enum valve_identity_serial which, uint8_t *dest, size_t dest_size);

#if defined(CONFIG_USB_DEVICE_STACK)
uint8_t *usb_update_sn_string_descriptor(void);
#endif
