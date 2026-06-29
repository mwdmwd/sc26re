/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "controller.h"
#include "ibex_settings_registry.h"
#include "power.h"
#include "valve_feature.h"
#include "valve_identity.h"
#include "valve_settings.h"

LOG_MODULE_REGISTER(valve_feature);

#define VALVE_SETTINGS_PATH_MAX 24
#define VALVE_PROTOCOL_BUILD_TIMESTAMP 0x6a3bfe74
#define VALVE_PROTOCOL_BUILD_SHA "a371f66dd017"
#define VALVE_BLE_CAPABILITIES 0x68d2f92e
#define VALVE_BLE_HARDWARE_ID 0x49
#define VALVE_USB_PRODUCT_ID 0x1302
#define VALVE_USB_CAPABILITIES 0x4168BFFF
#define VALVE_USB_HARDWARE_ID 0x45
#define VALVE_USB_UNIQUE_ID 0x12345678
#define VALVE_FICR_DEVICEID_BASE 0x10000060u
#define VALVE_STRING_ATTRIBUTE_SIZE 20
#define VALVE_STRING_ATTRIBUTE_TEXT_SIZE (VALVE_STRING_ATTRIBUTE_SIZE - 1)
#define VALVE_TURN_OFF_DELAY_MS 150
#define VALVE_ESB_HANDOFF_SIGNATURE 0xa427af52
#define VALVE_ESB_HANDOFF_REBOOT_DELAY_MS 150
#define VALVE_FEATURE_LOG_LIMIT 128
#define VALVE_DIGITAL_MAPPING_CAPACITY 60

static bool esb_handoff_reboot_scheduled;
static uint32_t feature_log_count;
static uint32_t feature_response_log_count;
static uint8_t digital_mappings[VALVE_DIGITAL_MAPPING_CAPACITY];
static size_t digital_mappings_len;
static void turn_off_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(turn_off_work, turn_off_work_handler);

static void turn_off_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	power_off();
}

static void append_numeric_attr(uint8_t **cursor, uint8_t tag, uint32_t value)
{
	*(*cursor)++ = tag;
	sys_put_le32(value, *cursor);
	*cursor += sizeof(value);
}

static void append_string_attr(uint8_t **cursor, uint8_t tag, const char *value)
{
	*(*cursor)++ = tag;
	memset(*cursor, 0, VALVE_STRING_ATTRIBUTE_TEXT_SIZE);
	memcpy(*cursor, value, MIN(strlen(value), (size_t)VALVE_STRING_ATTRIBUTE_TEXT_SIZE));
	*cursor += VALVE_STRING_ATTRIBUTE_TEXT_SIZE;
}

static bool request_path(const uint8_t *request, size_t len, char *path, size_t path_size,
                         size_t *value_offset)
{
	const uint8_t *terminator;
	size_t body_len;
	size_t path_len;

	if(len < 2)
	{
		return false;
	}
	body_len = request[1];
	if(body_len == 0 || body_len + 2 > len)
	{
		return false;
	}
	terminator = memchr(&request[2], '\0', body_len);
	if(terminator == NULL)
	{
		return false;
	}
	path_len = terminator - &request[2] + 1;
	if(path_len > VALVE_SETTINGS_PATH_MAX || path_len > path_size)
	{
		return false;
	}
	memcpy(path, &request[2], path_len);
	*value_offset = 2 + path_len;
	return true;
}

static bool feature_request_body(const uint8_t *request, size_t len, const uint8_t **body,
                                 size_t *body_len)
{
	if(len < 2 || request[1] + 2U > len)
	{
		return false;
	}
	*body = &request[2];
	*body_len = request[1];
	return true;
}

static void strip_report_id(const uint8_t **request, size_t *len)
{
	if(*len > 1 && (*request)[0] == VALVE_FEATURE_REPORT_ID)
	{
		(*request)++;
		(*len)--;
	}
}

static const char *feature_link_name(enum valve_feature_link link)
{
	switch(link)
	{
		case VALVE_FEATURE_LINK_USB:
			return "usb";
		case VALVE_FEATURE_LINK_BLE:
			return "ble";
		case VALVE_FEATURE_LINK_ESB:
			return "esb";
		case VALVE_FEATURE_LINK_PUCK:
			return "puck";
		default:
			return "?";
	}
}

static const char *feature_opcode_name(uint8_t opcode)
{
#define NAME(x) case VALVE_FEATURE_##x: return #x;
	switch(opcode)
	{
		NAME(SET_DIGITAL_MAPPINGS)
		NAME(CLEAR_DIGITAL_MAPPINGS)
		NAME(GET_DIGITAL_MAPPINGS)
		NAME(GET_ATTRIBUTES_VALUES)
		NAME(SET_DEFAULT_DIGITAL_MAPPINGS)
		NAME(FACTORY_RESET)
		NAME(SET_SETTINGS_VALUES)
		NAME(CLEAR_SETTINGS_VALUES)
		NAME(GET_SETTINGS_VALUES)
		NAME(GET_SETTINGS_MAXS)
		NAME(GET_SETTINGS_DEFAULTS)
		NAME(LOAD_DEFAULT_SETTINGS)
		NAME(REBOOT_TO_ISP)
		NAME(FIRMWARE_UPDATE_REBOOT)
		NAME(TURN_OFF_CONTROLLER)
		NAME(GET_DEVICE_INFO)
		NAME(WRITE_CALIBRATION_DATA)
		NAME(GET_STRING_ATTRIBUTE)
		NAME(GET_CHIPID)
		NAME(GET_BATTERY_DATA)
		NAME(CALIBRATE_ANALOG_TRIGGERS)
		NAME(SET_AUDIO_MAPPING)
		NAME(CALIBRATE_PRESSURE_SENSORS)
		NAME(SET_LED_COLOR)
		NAME(CALIBRATE_TRACKPAD_STICK)
		NAME(GET_USER_STORE)
		NAME(SET_USER_STORE)
		NAME(SET_TRACKPAD_SIDE)
		NAME(GET_LED_COLOR)
		NAME(READ_SETTING)
		NAME(STAGE_SETTING)
		NAME(COMMIT_SETTING)
		NAME(DELETE_SETTING)
		NAME(GET_SYSTEM_INFO)
		NAME(WRITE_PROVISIONING)
#undef NAME
		default:
			return "?";
	}
}

static void log_settings_write(enum valve_feature_link link, const uint8_t *request, size_t len)
{
	const uint8_t *body;
	size_t body_len;

	if(!feature_request_body(request, len, &body, &body_len))
	{
		return;
	}

	for(size_t offset = 0; offset + 3 <= body_len && offset < 24; offset += 3)
	{
		uint8_t id = body[offset];
		int16_t value = (int16_t)sys_get_le16(&body[offset + 1]);
		const char *name = ibex_setting_name(id);

		LOG_INF("%s setting[%u] (%s) = %d", feature_link_name(link), id,
		        name != NULL ? name : "unknown", value);
	}
}

static void log_feature_request(enum valve_feature_link link, const uint8_t *request, size_t len)
{
	uint8_t opcode;

	if(len == 0 || feature_log_count >= VALVE_FEATURE_LOG_LIMIT)
	{
		return;
	}

	opcode = request[0];
	feature_log_count++;
	LOG_INF("%s feature 0x%02x %s len=%u", feature_link_name(link), opcode,
	        feature_opcode_name(opcode), len);
	if(opcode == VALVE_FEATURE_SET_SETTINGS_VALUES)
	{
		log_settings_write(link, request, len);
	}
}

static void prepare_device_attributes(enum valve_feature_link link, uint8_t **cursor)
{
	switch(link)
	{
		case VALVE_FEATURE_LINK_USB:
			append_numeric_attr(cursor, 0, VALVE_USB_UNIQUE_ID);
			append_numeric_attr(cursor, 1, VALVE_USB_PRODUCT_ID);
			append_numeric_attr(cursor, 2, VALVE_USB_CAPABILITIES);
			append_numeric_attr(cursor, 4, CONFIG_IBEX_BUILD_TIMESTAMP);
			append_numeric_attr(cursor, 9, VALVE_USB_HARDWARE_ID);
			append_numeric_attr(cursor, 11, 100);
			break;
		case VALVE_FEATURE_LINK_BLE:
		case VALVE_FEATURE_LINK_ESB:
		case VALVE_FEATURE_LINK_PUCK:
		default:
			append_numeric_attr(cursor, 1, VALVE_USB_PRODUCT_ID);
			append_numeric_attr(cursor, 2, 0);
			append_numeric_attr(cursor, 10, VALVE_BLE_CAPABILITIES);
			append_numeric_attr(cursor, 4, VALVE_PROTOCOL_BUILD_TIMESTAMP);
			append_numeric_attr(cursor, 9, VALVE_BLE_HARDWARE_ID);
			break;
	}
}

static void prepare_string_attribute(enum valve_feature_link link, const uint8_t *request,
                                     size_t len, uint8_t **cursor)
{
	uint8_t tag = len > 2 ? request[2] : 1;

	ARG_UNUSED(link);

	if(tag == 0)
	{
		append_string_attr(cursor, tag, valve_identity_serial(VALVE_IDENTITY_BOARD_SERIAL));
	}
	else if(tag == 1)
	{
		append_string_attr(cursor, tag, valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL));
	}
	else if(tag == 3)
	{
		append_string_attr(cursor, tag, VALVE_PROTOCOL_BUILD_SHA);
	}
	else if(tag == 4)
	{
		append_string_attr(cursor, tag, valve_identity_serial(VALVE_IDENTITY_BOARD_SERIAL));
	}
	else
	{
		append_string_attr(cursor, tag, "NA");
	}
}

static void prepare_version_attribute(const uint8_t *request, size_t len, uint8_t **cursor)
{
	if(len > 2 && request[2] == 0)
	{
		*(*cursor)++ = 0;
		sys_put_le32(VALVE_PROTOCOL_BUILD_TIMESTAMP, *cursor);
		*cursor += sizeof(uint32_t);
		sys_put_le32(VALVE_BLE_HARDWARE_ID, *cursor);
		*cursor += sizeof(uint32_t);
		memset(*cursor, 0, 16);
		memcpy(*cursor, VALVE_PROTOCOL_BUILD_SHA, MIN(strlen(VALVE_PROTOCOL_BUILD_SHA), 16));
		*cursor += 16;
		memset(*cursor, 0, 16);
		memcpy(*cursor, valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL),
		       MIN(strlen(valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL)), 16));
		*cursor += 16;
	}
	else if(len > 2 && request[2] == 1)
	{
		static const uint8_t variant_1[] = {
			0x01, 0x00, 0x74, 0xfe, 0x3b, 0x6a, 0x61, 0x33, 0x37, 0x31, 0x66, 0x36,
			0x36, 0x64, 0x64, 0x30, 0x31, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		};

		memcpy(*cursor, variant_1, sizeof(variant_1));
		*cursor += sizeof(variant_1);
	}
	else if(len > 2 && request[2] == 2)
	{
		*(*cursor)++ = 2;
		sys_put_le32(k_uptime_get_32(), *cursor);
		*cursor += sizeof(uint32_t);
		sys_put_le32(0, *cursor);
		*cursor += sizeof(uint32_t);
	}
}

static void prepare_digital_mappings(const uint8_t *request, size_t len, uint8_t **cursor,
                                     uint8_t *end)
{
	uint8_t start = 0;
	size_t remaining;
	size_t copy_len;

	if(len > 2)
	{
		start = request[2];
	}

	if(start >= digital_mappings_len)
	{
		if(*cursor < end)
		{
			*(*cursor)++ = 0xff;
		}
		return;
	}

	remaining = digital_mappings_len - start;
	copy_len = MIN(remaining, (size_t)(end - *cursor));
	memcpy(*cursor, &digital_mappings[start], copy_len);
	*cursor += copy_len;
}

static void prepare_chip_id(uint8_t **cursor, uint8_t *end)
{
	const uint32_t *device_id = (const uint32_t *)VALVE_FICR_DEVICEID_BASE;

	if(*cursor + 8 > end)
	{
		return;
	}

	sys_put_le32(device_id[0], *cursor);
	*cursor += sizeof(uint32_t);
	sys_put_le32(device_id[1], *cursor);
	*cursor += sizeof(uint32_t);
}

static void prepare_device_info(const uint8_t *request, size_t len, uint8_t **cursor, uint8_t *end)
{
	const uint32_t *device_id = (const uint32_t *)VALVE_FICR_DEVICEID_BASE;
	uint8_t id[16] = { 0 };
	uint8_t selector = len > 2 ? request[2] : 0;

	if(*cursor + 18 > end)
	{
		return;
	}

	if(selector != 1)
	{
		memset(*cursor, 0, 18);
		*cursor += 18;
		return;
	}

	*(*cursor)++ = selector;
	*(*cursor)++ = 0;
	if(hwinfo_get_device_id(id, sizeof(id)) <= 0)
	{
		sys_put_le32(device_id[0], id);
		sys_put_le32(device_id[1], &id[4]);
	}
	memcpy(*cursor, id, sizeof(id));
	*cursor += sizeof(id);
}

static void log_feature_response(enum valve_feature_link link, uint8_t opcode,
                                 const uint8_t *response)
{
	if(feature_response_log_count >= VALVE_FEATURE_LOG_LIMIT)
	{
		return;
	}

	switch(opcode)
	{
		case VALVE_FEATURE_GET_ATTRIBUTES_VALUES:
		case VALVE_FEATURE_GET_DIGITAL_MAPPINGS:
		case VALVE_FEATURE_GET_DEVICE_INFO:
		case VALVE_FEATURE_GET_CHIPID:
			feature_response_log_count++;
			LOG_INF("%s feature 0x%02x response len=%u", feature_link_name(link), opcode,
			        response[1]);
			break;
		default:
			break;
	}
}

static ssize_t build_feature_response(enum valve_feature_link link, const uint8_t *request,
                                      size_t len, uint8_t *response, size_t response_capacity)
{
	uint8_t *end = &response[response_capacity];
	char path[VALVE_SETTINGS_PATH_MAX];
	size_t value_offset;
	uint8_t opcode;

	if(response_capacity < 2)
	{
		return -ENOSPC;
	}

	uint8_t *cursor = &response[2];
	memset(response, 0, response_capacity);
	if(len == 0)
	{
		return -EINVAL;
	}
	strip_report_id(&request, &len);
	if(len == 0)
	{
		return -EINVAL;
	}

	opcode = request[0];
	response[0] = opcode;

	switch(opcode)
	{
		case VALVE_FEATURE_GET_ATTRIBUTES_VALUES:
			prepare_device_attributes(link, &cursor);
			break;
		case VALVE_FEATURE_GET_STRING_ATTRIBUTE:
			prepare_string_attribute(link, request, len, &cursor);
			break;
		case VALVE_FEATURE_GET_SYSTEM_INFO:
			prepare_version_attribute(request, len, &cursor);
			break;
		case VALVE_FEATURE_GET_DIGITAL_MAPPINGS:
			prepare_digital_mappings(request, len, &cursor, end);
			break;
		case VALVE_FEATURE_GET_DEVICE_INFO:
			prepare_device_info(request, len, &cursor, end);
			break;
		case VALVE_FEATURE_GET_CHIPID:
			prepare_chip_id(&cursor, end);
			break;
		case VALVE_FEATURE_READ_SETTING:
			if(request_path(request, len, path, sizeof(path), &value_offset))
			{
				size_t setting_len;

				if(valve_settings_read(path, cursor, end - cursor, &setting_len))
				{
					cursor += setting_len;
				}
				else
				{
					*cursor++ = 0;
				}
			}
			else
			{
				*cursor++ = 0;
			}
			break;
		case VALVE_FEATURE_CLEAR_SETTINGS_VALUES:
		case VALVE_FEATURE_GET_SETTINGS_VALUES:
		case VALVE_FEATURE_GET_SETTINGS_MAXS:
		case VALVE_FEATURE_GET_SETTINGS_DEFAULTS:
		case VALVE_FEATURE_LOAD_DEFAULT_SETTINGS:
		{
			size_t settings_len;

			if(!ibex_settings_feature_response(request, len, cursor, end - cursor, &settings_len))
			{
				return -EINVAL;
			}
			cursor += settings_len;
			break;
		}
		case VALVE_FEATURE_SET_DIGITAL_MAPPINGS:
		case VALVE_FEATURE_CLEAR_DIGITAL_MAPPINGS:
		case VALVE_FEATURE_SET_DEFAULT_DIGITAL_MAPPINGS:
		case VALVE_FEATURE_SET_SETTINGS_VALUES:
		case VALVE_FEATURE_REBOOT_TO_ISP:
		case VALVE_FEATURE_FIRMWARE_UPDATE_REBOOT:
		case VALVE_FEATURE_TURN_OFF_CONTROLLER:
		case VALVE_FEATURE_STAGE_SETTING:
		case VALVE_FEATURE_COMMIT_SETTING:
			break;
		default:
			return -ENOTSUP;
	}

	response[1] = cursor - &response[2];
	log_feature_response(link, opcode, response);
	LOG_HEXDUMP_DBG(response, MIN(response_capacity, 40), "feature response");
	return (ssize_t)(response[1] + 2);
}

static int handle_feature_request(enum valve_feature_link link, const uint8_t *request, size_t len)
{
	char path[VALVE_SETTINGS_PATH_MAX];
	size_t value_offset;

	if(len == 0)
	{
		return -EINVAL;
	}
	strip_report_id(&request, &len);
	if(len == 0)
	{
		return -EINVAL;
	}

	log_feature_request(link, request, len);

	if(request_path(request, len, path, sizeof(path), &value_offset))
	{
		size_t value_len = request[1] - (value_offset - 2);

		switch(request[0])
		{
			case VALVE_FEATURE_STAGE_SETTING:
				return valve_settings_stage(path, &request[value_offset], value_len);
			case VALVE_FEATURE_COMMIT_SETTING:
				return valve_settings_commit(path);
			default:
				break;
		}
	}

	if(ibex_settings_feature_write(request, len))
	{
		return 0;
	}

	switch(request[0])
	{
		case VALVE_FEATURE_SET_DIGITAL_MAPPINGS:
		{
			const uint8_t *body;
			size_t body_len;

			if(!feature_request_body(request, len, &body, &body_len))
			{
				return -EINVAL;
			}

			digital_mappings_len = MIN(body_len, sizeof(digital_mappings));
			memcpy(digital_mappings, body, digital_mappings_len);
			return 0;
		}
		case VALVE_FEATURE_CLEAR_DIGITAL_MAPPINGS:
			digital_mappings_len = 0;
			return 0;
		case VALVE_FEATURE_SET_DEFAULT_DIGITAL_MAPPINGS:
			digital_mappings_len = 0;
			return 0;
		case VALVE_FEATURE_REBOOT_TO_ISP:
			(void)power_reboot_to_valve_isp();
			return 0;
		case VALVE_FEATURE_TURN_OFF_CONTROLLER:
			LOG_INF("Steam requested controller power-off");
			(void)k_work_schedule(&turn_off_work, K_MSEC(VALVE_TURN_OFF_DELAY_MS));
			return 0;
		case VALVE_FEATURE_FIRMWARE_UPDATE_REBOOT:
			if(IS_ENABLED(CONFIG_IBEX_ESB) &&
			   len >= 6 &&
			   request[1] == sizeof(uint32_t) &&
			   sys_get_le32(&request[2]) == VALVE_ESB_HANDOFF_SIGNATURE &&
			   !esb_handoff_reboot_scheduled)
			{
				esb_handoff_reboot_scheduled = true;
				LOG_INF("Steam requested ESB handoff reboot (signature 0x%08x)",
				        VALVE_ESB_HANDOFF_SIGNATURE);
				radio_personality_reboot_into_after(RADIO_PERSONALITY_ESB,
				                                    VALVE_ESB_HANDOFF_REBOOT_DELAY_MS);
			}
			else
			{
				(void)power_reboot_normal();
			}
			return 0;
		case VALVE_FEATURE_GET_ATTRIBUTES_VALUES:
		case VALVE_FEATURE_GET_STRING_ATTRIBUTE:
		case VALVE_FEATURE_GET_SYSTEM_INFO:
		case VALVE_FEATURE_GET_DIGITAL_MAPPINGS:
		case VALVE_FEATURE_GET_DEVICE_INFO:
		case VALVE_FEATURE_GET_CHIPID:
		case VALVE_FEATURE_READ_SETTING:
		case VALVE_FEATURE_GET_SETTINGS_VALUES:
		case VALVE_FEATURE_GET_SETTINGS_MAXS:
		case VALVE_FEATURE_GET_SETTINGS_DEFAULTS:
			return 0;
		case VALVE_FEATURE_SET_SETTINGS_VALUES:
		case VALVE_FEATURE_STAGE_SETTING:
		case VALVE_FEATURE_COMMIT_SETTING:
			return -EINVAL;
		default:
			return -ENOTSUP;
	}
}

static size_t feature_response_min_capacity(enum valve_feature_link link)
{
	switch(link)
	{
		case VALVE_FEATURE_LINK_USB:
		case VALVE_FEATURE_LINK_ESB:
		case VALVE_FEATURE_LINK_PUCK:
			return VALVE_FEATURE_REPORT_SIZE - 1;
		/* XXX maybe the actual value is one lower? */
		case VALVE_FEATURE_LINK_BLE:
		default:
			return VALVE_FEATURE_REPORT_SIZE;
	}
}

static int feature_validate_request(const uint8_t *request, size_t request_len)
{
	if(request == NULL || request_len == 0 || request_len > VALVE_FEATURE_REPORT_SIZE)
	{
		return -EINVAL;
	}

	strip_report_id(&request, &request_len);
	if(request_len == 0)
	{
		return -EINVAL;
	}

	return 0;
}

int valve_feature_handle_request(enum valve_feature_link link, const uint8_t *request,
                                 size_t request_len)
{
	int err = feature_validate_request(request, request_len);

	if(err)
	{
		return err;
	}

	return handle_feature_request(link, request, request_len);
}

ssize_t valve_feature_prepare_response(enum valve_feature_link link, const uint8_t *request,
                                       size_t request_len, uint8_t *response,
                                       size_t response_capacity)
{
	int err = feature_validate_request(request, request_len);
	ssize_t response_len;

	if(err)
	{
		if(response != NULL)
		{
			memset(response, 0, response_capacity);
		}
		return err;
	}

	if(response == NULL || response_capacity < feature_response_min_capacity(link))
	{
		if(response != NULL)
		{
			memset(response, 0, response_capacity);
		}
		return -ENOSPC;
	}

	response_len = build_feature_response(link, request, request_len, response, response_capacity);
	if(response_len < 0)
	{
		memset(response, 0, response_capacity);
	}

	return response_len;
}

ssize_t valve_feature_respond(enum valve_feature_link link, const uint8_t *request,
                              size_t request_len, uint8_t *response, size_t response_capacity)
{
	int err = valve_feature_handle_request(link, request, request_len);

	if(err)
	{
		if(response != NULL)
		{
			memset(response, 0, response_capacity);
		}
		return err;
	}

	return valve_feature_prepare_response(link, request, request_len, response, response_capacity);
}
