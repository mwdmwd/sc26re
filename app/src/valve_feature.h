/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include "sdl/controller_constants.h"

#define VALVE_FEATURE_REPORT_ID 0x01
#define VALVE_FEATURE_REPORT_SIZE 64
enum valve_feature_link
{
	VALVE_FEATURE_LINK_BLE,
	VALVE_FEATURE_LINK_USB,
	VALVE_FEATURE_LINK_ESB,
	VALVE_FEATURE_LINK_PUCK,
};

enum valve_feature_opcode
{
	VALVE_FEATURE_SET_DIGITAL_MAPPINGS = ID_SET_DIGITAL_MAPPINGS,                 /* 0x80 */
	VALVE_FEATURE_CLEAR_DIGITAL_MAPPINGS = ID_CLEAR_DIGITAL_MAPPINGS,             /* 0x81 */
	VALVE_FEATURE_GET_DIGITAL_MAPPINGS = ID_GET_DIGITAL_MAPPINGS,                 /* 0x82 */
	VALVE_FEATURE_GET_ATTRIBUTES_VALUES = ID_GET_ATTRIBUTES_VALUES,               /* 0x83 */
	VALVE_FEATURE_SET_DEFAULT_DIGITAL_MAPPINGS = ID_SET_DEFAULT_DIGITAL_MAPPINGS, /* 0x85 */
	VALVE_FEATURE_FACTORY_RESET = ID_FACTORY_RESET,                               /* 0x86 */
	VALVE_FEATURE_SET_SETTINGS_VALUES = ID_SET_SETTINGS_VALUES,                   /* 0x87 */
	VALVE_FEATURE_CLEAR_SETTINGS_VALUES = ID_CLEAR_SETTINGS_VALUES,               /* 0x88 */
	VALVE_FEATURE_GET_SETTINGS_VALUES = ID_GET_SETTINGS_VALUES,                   /* 0x89 */
	VALVE_FEATURE_GET_SETTINGS_MAXS = ID_GET_SETTINGS_MAXS,                       /* 0x8B */
	VALVE_FEATURE_GET_SETTINGS_DEFAULTS = ID_GET_SETTINGS_DEFAULTS,               /* 0x8C */
	VALVE_FEATURE_LOAD_DEFAULT_SETTINGS = ID_LOAD_DEFAULT_SETTINGS,               /* 0x8E */
	VALVE_FEATURE_REBOOT_TO_ISP = 0x90,                                           /* 0x90 */
	VALVE_FEATURE_FIRMWARE_UPDATE_REBOOT = 0x95,                                  /* 0x95 */
	VALVE_FEATURE_TURN_OFF_CONTROLLER = ID_TURN_OFF_CONTROLLER,                   /* 0x9F */
	VALVE_FEATURE_GET_DEVICE_INFO = ID_GET_DEVICE_INFO,                           /* 0xA1 */
	VALVE_FEATURE_WRITE_CALIBRATION_DATA = 0xA2,                                  /* 0xA2 */
	VALVE_FEATURE_GET_STRING_ATTRIBUTE = ID_GET_STRING_ATTRIBUTE,                 /* 0xAE */
	VALVE_FEATURE_GET_CHIPID = ID_GET_CHIPID,                                     /* 0xBA */
	VALVE_FEATURE_GET_BATTERY_DATA = 0xBE,                                        /* 0xBE */
	VALVE_FEATURE_CALIBRATE_ANALOG_TRIGGERS = ID_CALIBRATE_ANALOG_TRIGGERS,       /* 0xC0 */
	VALVE_FEATURE_SET_AUDIO_MAPPING = ID_SET_AUDIO_MAPPING,                       /* 0xC1 */
	/* SDL name is generic but the stock handler calibrates pressure sensors */   /**/
	VALVE_FEATURE_CALIBRATE_PRESSURE_SENSORS = ID_CALIBRATE_ANALOG,               /* 0xC3 */
	VALVE_FEATURE_SET_LED_COLOR = 0xC5,                                           /* 0xC5 */
	VALVE_FEATURE_CALIBRATE_TRACKPAD_STICK = 0xD8,                                /* 0xD8 */
	VALVE_FEATURE_GET_USER_STORE = 0xDB,                                          /* 0xDB */
	VALVE_FEATURE_SET_USER_STORE = 0xDC,                                          /* 0xDC */
	VALVE_FEATURE_SET_TRACKPAD_SIDE = 0xE2,                                       /* 0xE2 */
	VALVE_FEATURE_GET_LED_COLOR = 0xE9,                                           /* 0xE9 */
	VALVE_FEATURE_READ_SETTING = 0xED,                                            /* 0xED */
	VALVE_FEATURE_STAGE_SETTING = 0xEE,                                           /* 0xEE */
	VALVE_FEATURE_COMMIT_SETTING = 0xEF,                                          /* 0xEF */
	VALVE_FEATURE_DELETE_SETTING = 0xF0,                                          /* 0xF0 */
	VALVE_FEATURE_GET_SYSTEM_INFO = 0xF2,                                         /* 0xF2 */
	VALVE_FEATURE_WRITE_PROVISIONING = 0xFE,                                      /* 0xFE */
};

int valve_feature_handle_request(enum valve_feature_link link, const uint8_t *request,
                                 size_t request_len);
ssize_t valve_feature_prepare_response(enum valve_feature_link link, const uint8_t *request,
                                       size_t request_len, uint8_t *response,
                                       size_t response_capacity);
ssize_t valve_feature_respond(enum valve_feature_link link, const uint8_t *request,
                              size_t request_len, uint8_t *response, size_t response_capacity);
