/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#include "controller.h"
#include "sdl/controller_structs.h"
#include "triton_state_report.h"
#include "valve_feature.h"
#include "valve_hid_report_map.h"
#include "valve_identity.h"
#include "valve_settings.h"

#if defined(CONFIG_BT_BAS)
#include <zephyr/bluetooth/services/bas.h>
#endif

LOG_MODULE_REGISTER(transport_ble);

#define VALVE_DEVICE_NAME_PREFIX "Steam Ctrl (BT) "
#define VALVE_IDENTITY_COUNT 3
#define VALVE_PRIMARY_INPUT_ATTR 12
#define VALVE_BATTERY_INPUT_ATTR 16
#define VALVE_BLE_STATE_INPUT_ATTR 24
#define VALVE_HAPTIC_PCM_STEREO_REPORT 0x88
#define VALVE_HAPTIC_PCM_STEREO_SAMPLES 31

enum
{
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info
{
	uint16_t version;
	uint8_t country_code;
	uint8_t flags;
} __packed;

struct valve_report
{
	uint8_t id;
	uint8_t type;
	uint8_t size;
	uint8_t data[VALVE_FEATURE_REPORT_SIZE];
};

#define VALVE_REPORT(name, report_id, report_type, report_size) \
	static struct valve_report name = { \
		.id = report_id, \
		.type = report_type, \
		.size = report_size, \
	}

static struct hids_info hids_info = {
	.version = 0x0101,
	.flags = HIDS_REMOTE_WAKE | HIDS_NORMALLY_CONNECTABLE,
};
static uint8_t protocol_mode = 1;
static uint8_t control_point;
static uint8_t input_sequence;
static bool primary_input_notify_enabled;
static bool ble_state_input_notify_enabled;
static bool battery_input_notify_enabled;
static bool ble_started;
static struct bt_conn *active_conn;
static bool identity_has_bond[CONFIG_BT_ID_MAX];
static uint32_t input_no_subscription_logs;
static uint32_t input_notify_error_logs;
static uint32_t input_42_reports_sent;
static uint32_t input_45_reports_sent;

VALVE_REPORT(input_40, 0x40, HIDS_INPUT, 5);
VALVE_REPORT(input_41, 0x41, HIDS_INPUT, 8);
VALVE_REPORT(input_42, ID_TRITON_CONTROLLER_STATE, HIDS_INPUT, 53);
VALVE_REPORT(input_43, 0x43, HIDS_INPUT, 14);
VALVE_REPORT(input_44, 0x44, HIDS_INPUT, 5);
VALVE_REPORT(input_45, ID_TRITON_CONTROLLER_STATE_BLE, HIDS_INPUT, 45);
VALVE_REPORT(output_80, ID_OUT_REPORT_HAPTIC_RUMBLE, HIDS_OUTPUT, 9);
VALVE_REPORT(output_81, ID_OUT_REPORT_HAPTIC_PULSE, HIDS_OUTPUT, 7);
VALVE_REPORT(output_82, ID_OUT_REPORT_HAPTIC_COMMAND, HIDS_OUTPUT, 3);
VALVE_REPORT(output_83, ID_OUT_REPORT_HAPTIC_LFO_TONE, HIDS_OUTPUT, 9);
VALVE_REPORT(output_84, ID_OUT_REPORT_HAPTIC_LOG_SWEEP, HIDS_OUTPUT, 8);
VALVE_REPORT(output_85, ID_OUT_REPORT_HAPTIC_SCRIPT, HIDS_OUTPUT, 3);
VALVE_REPORT(output_86, 0x86, HIDS_OUTPUT, 3);
VALVE_REPORT(output_87, 0x87, HIDS_OUTPUT, 63);
VALVE_REPORT(output_88, 0x88, HIDS_OUTPUT, 63);
VALVE_REPORT(output_89, 0x89, HIDS_OUTPUT, 63);
VALVE_REPORT(feature_01, 0x01, HIDS_FEATURE, VALVE_FEATURE_REPORT_SIZE);

static uint8_t valve_ble_hid_report_map[VALVE_HID_REPORT_MAP_SIZE];

static ssize_t read_static(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                           uint16_t len, uint16_t offset)
{
	size_t size = 1;

	if(attr->user_data == valve_ble_hid_report_map)
	{
		size = sizeof(valve_ble_hid_report_map);
	}
	else if(attr->user_data == &hids_info)
	{
		size = sizeof(hids_info);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, size);
}

static ssize_t read_report(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                           uint16_t len, uint16_t offset)
{
	struct valve_report *report = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, report->data, report->size);
}

static ssize_t read_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, 2);
}

static void handle_output_report(const struct valve_report *report)
{
	switch(report->id)
	{
		case VALVE_HAPTIC_PCM_STEREO_REPORT:
			if(report->data[0] == 0 || report->data[0] > VALVE_HAPTIC_PCM_STEREO_SAMPLES)
			{
				LOG_WRN("invalid stereo PCM length %u", report->data[0]);
				return;
			}

			break;
		default:
			break;
	}
}

static ssize_t write_report(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                            uint16_t len, uint16_t offset, uint8_t flags)
{
	struct valve_report *report = attr->user_data;

	ARG_UNUSED(conn);

	if(report->type == HIDS_FEATURE && offset == 0 && flags == 0)
	{
		const uint8_t *request = buf;

		LOG_DBG("direct feature request: opcode 0x%02x, length %u", request[0], len);
		LOG_HEXDUMP_DBG(request, MIN(len, 40), "feature request");
	}

	if(offset + len > report->size)
	{
		LOG_WRN("reject: rpt=0x%02x off=%u+len=%u > size=%u", report->id, offset, len,
		        report->size);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if(flags & BT_GATT_WRITE_FLAG_PREPARE)
	{
		return 0;
	}

	if(flags & BT_GATT_WRITE_FLAG_EXECUTE)
	{
		LOG_DBG("executing long write: report 0x%02x, offset %u, length %u", report->id, offset,
		        len);
	}

	if(report->type == HIDS_FEATURE && offset == 0)
	{
		valve_feature_respond(VALVE_FEATURE_LINK_BLE, buf, len, feature_01.data,
		                      sizeof(feature_01.data));
		return len;
	}

	memcpy(report->data + offset, buf, len);
	if(report->type == HIDS_OUTPUT && offset + len == report->size)
	{
		handle_output_report(report);
	}
	return len;
}

static ssize_t write_byte(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                          uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(flags);

	if(offset != 0 || len != 1)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	*(uint8_t *)attr->user_data = *(const uint8_t *)buf;
	return len;
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	struct valve_report *report = attr[-1].user_data;
	bool enabled = value == BT_GATT_CCC_NOTIFY;

	if(report == &input_43)
	{
		battery_input_notify_enabled = enabled;
	}
	else if(report == &input_42)
	{
		primary_input_notify_enabled = enabled;
	}
	else if(report == &input_45)
	{
		ble_state_input_notify_enabled = enabled;
	}
	LOG_INF("BLE report 0x%02x notifications %s", report->id, enabled ? "enabled" : "disabled");
}

#define VALVE_INPUT(report) \
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, \
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, \
			       BT_GATT_PERM_READ_ENCRYPT, read_report, NULL, \
			       &report), \
	BT_GATT_CCC(input_ccc_changed, \
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT), \
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ, \
			   read_report_ref, NULL, &report)

#define VALVE_OUTPUT(report) \
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, \
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | \
				   BT_GATT_CHRC_WRITE_WITHOUT_RESP, \
			       BT_GATT_PERM_READ_ENCRYPT | \
				   BT_GATT_PERM_WRITE_ENCRYPT | \
				   BT_GATT_PERM_PREPARE_WRITE, \
			       read_report, write_report, &report), \
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ, \
			   read_report_ref, NULL, &report)

#define VALVE_OUTPUT_WNR(report) \
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, \
			       BT_GATT_CHRC_READ | \
				   BT_GATT_CHRC_WRITE_WITHOUT_RESP, \
			       BT_GATT_PERM_READ_ENCRYPT | \
				   BT_GATT_PERM_WRITE_ENCRYPT, \
			       read_report, write_report, &report), \
	BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ, \
			   read_report_ref, NULL, &report)

#define VALVE_FEATURE(report) VALVE_OUTPUT(report)

// clang-format off
BT_GATT_SERVICE_DEFINE(hog_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_PROTOCOL_MODE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT,
			       read_static, write_byte, &protocol_mode),
	VALVE_INPUT(input_40),
	VALVE_INPUT(input_41),
	VALVE_INPUT(input_42),
	VALVE_INPUT(input_43),
	VALVE_INPUT(input_44),
	VALVE_INPUT(input_45),
	VALVE_OUTPUT(output_80),
	VALVE_OUTPUT(output_81),
	VALVE_OUTPUT(output_82),
	VALVE_OUTPUT(output_83),
	VALVE_OUTPUT(output_84),
	VALVE_OUTPUT(output_85),
	VALVE_OUTPUT(output_86),
	VALVE_OUTPUT(output_87),
	VALVE_OUTPUT_WNR(output_88),
	VALVE_OUTPUT(output_89),
	VALVE_FEATURE(feature_01),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_static, NULL,
			       valve_ble_hid_report_map),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ, read_static, NULL, &hids_info),
	BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP,
			       BT_GATT_PERM_WRITE_ENCRYPT, NULL, write_byte,
			       &control_point));
// clang-format on

static const struct bt_data advertising[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xc4, 0x03),
	BT_DATA_BYTES(BT_DATA_TX_POWER, 0x06),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
	              BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0x5d, 0x05),
};

static void remember_bond(const struct bt_bond_info *info, void *user_data)
{
	ARG_UNUSED(info);
	*(bool *)user_data = true;
}

static int select_valve_identity(uint8_t *selected_id)
{
	bt_addr_le_t identities[CONFIG_BT_ID_MAX];
	size_t identity_count = ARRAY_SIZE(identities);

	bt_id_get(identities, &identity_count);
	while(identity_count < VALVE_IDENTITY_COUNT)
	{
		int id = bt_id_create(NULL, NULL);

		if(id < 0)
		{
			return id;
		}
		identity_count++;
	}

	for(uint8_t id = 1; id < VALVE_IDENTITY_COUNT; ++id)
	{
		bt_foreach_bond(id, remember_bond, &identity_has_bond[id]);
		if(identity_has_bond[id])
		{
			*selected_id = id;
			LOG_INF("using bonded Bluetooth identity %u", id);
			return 0;
		}
	}

	*selected_id = 1;
	LOG_INF("no existing Valve-style bond; using Bluetooth identity 1");
	return 0;
}

static int set_ble_identity_strings(void)
{
	char device_name[CONFIG_BT_DEVICE_NAME_MAX + 1];
	const char *serial = valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL);
	int err;

	snprintk(device_name, sizeof(device_name), VALVE_DEVICE_NAME_PREFIX "%s", serial);
	err = bt_set_name(device_name);
	if(err)
	{
		return err;
	}

	if(IS_ENABLED(CONFIG_BT_DIS_SETTINGS) && IS_ENABLED(CONFIG_SETTINGS_RUNTIME))
	{
		err = settings_runtime_set("bt/dis/serial", serial, strlen(serial));
		if(err)
		{
			LOG_WRN("failed to set BLE DIS serial at runtime: %d", err);
		}
	}

	return 0;
}

static void ble_mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                                struct bt_gatt_exchange_params *params)
{
	ARG_UNUSED(params);

	if(err)
	{
		LOG_WRN("ATT MTU exchange failed: 0x%02x", err);
		return;
	}

	LOG_DBG("ATT MTU exchanged: %u", bt_gatt_get_mtu(conn));
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = ble_mtu_exchange_cb,
};

/*
 * Request fast connection parameters as soon as a central connects.
 * The default connection interval chosen by Linux/BlueZ is typically
 * 30-50 ms, which is visibly laggy for HID input.
 */
static void ble_connected_cb(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;

	if(err)
	{
		return;
	}

	if(active_conn != NULL)
	{
		bt_conn_unref(active_conn);
	}
	active_conn = bt_conn_ref(conn);

	if(bt_conn_get_info(conn, &info) == 0)
	{
		LOG_INF("connected: CI=%u (%u.%02u ms)", info.le.interval, info.le.interval * 125 / 100,
		        (info.le.interval * 125) % 100);
	}

	/* 6..12 * 1.25 ms = 7.5..15 ms, latency 0, timeout 4 s */
	const struct bt_le_conn_param fast_param = BT_LE_CONN_PARAM_INIT(6, 12, 0, 400);
	int ret = bt_conn_le_param_update(conn, &fast_param);

	if(ret && ret != -EALREADY)
	{
		LOG_WRN("conn param update request failed: %d", ret);
	}
	else
	{
		LOG_INF("requested CI 7.5-15 ms");
	}

	ret = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if(ret)
	{
		LOG_WRN("ATT MTU exchange request failed: %d", ret);
	}
}

static void ble_param_updated_cb(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                                 uint16_t timeout)
{
	LOG_INF("conn params updated: CI=%u (%u.%02u ms) lat=%u to=%u", interval, interval * 125 / 100,
	        (interval * 125) % 100, latency, timeout);
}

static void ble_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("disconnected (reason 0x%02x)", reason);
	if(active_conn == conn)
	{
		bt_conn_unref(active_conn);
		active_conn = NULL;
	}
	primary_input_notify_enabled = false;
	ble_state_input_notify_enabled = false;
	battery_input_notify_enabled = false;
}

BT_CONN_CB_DEFINE(ble_conn_cbs) = {
	.connected = ble_connected_cb,
	.disconnected = ble_disconnected_cb,
	.le_param_updated = ble_param_updated_cb,
};

int transport_ble_init(void)
{
	struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
	    BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
	uint8_t selected_id;
	const char *device_name;
	int err;

	if(ble_started)
	{
		return 0;
	}

	valve_hid_report_map_copy_ble(valve_ble_hid_report_map, sizeof(valve_ble_hid_report_map));

	err = bt_enable(NULL);
	if(err && err != -EALREADY)
	{
		return err;
	}

	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
		valve_settings_load_feature_state();
	}

	err = set_ble_identity_strings();
	if(err)
	{
		return err;
	}

	err = select_valve_identity(&selected_id);
	if(err)
	{
		return err;
	}

	device_name = bt_get_name();
	const struct bt_data scan_response[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, device_name, strlen(device_name)),
	};

	adv_param.id = selected_id;
	LOG_INF("advertising as \"%s\" on identity %u with Valve HID map", device_name, selected_id);

	err = bt_le_adv_start(&adv_param, advertising, ARRAY_SIZE(advertising), scan_response,
	                      ARRAY_SIZE(scan_response));
	if(!err)
	{
		ble_started = true;
	}
	return err;
}

void transport_ble_deactivate(void)
{
	if(!ble_started)
	{
		return;
	}

	(void)bt_le_adv_stop();
	if(active_conn != NULL)
	{
		(void)bt_conn_disconnect(active_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	}
	primary_input_notify_enabled = false;
	ble_state_input_notify_enabled = false;
	battery_input_notify_enabled = false;
	ble_started = false;
}

static void fill_state_report(struct valve_report *state, const struct controller_report *report)
{
	triton_state_report_pack_body(state->data, state->size, input_sequence++, report,
	                              triton_state_report_timestamp_us());
}

int transport_ble_send(const struct controller_report *report)
{
	if(!primary_input_notify_enabled && !ble_state_input_notify_enabled)
	{
		if(input_no_subscription_logs < 8)
		{
			input_no_subscription_logs++;
			LOG_DBG("BLE input not sent: no 0x42/0x45 notification subscription");
		}
		return -ENOTCONN;
	}

	/*
	 * Compact BLE controller-state layout:
	 *   byte 0: sequence
	 *   bytes 1-4: little-endian button bitmap
	 *   bytes 5-16: little-endian trigger and stick axes
	 *   bytes 17-28: little-endian touchpad axes and pressures
	 *   bytes 29-32: little-endian IMU timestamp in microseconds
	 *   bytes 33-38: little-endian signed accelerometer XYZ, 16384 units/g
	 *
	 * Keep gyro and quaternion fields neutral until those sensors are implemented.
	 */
	if(ble_state_input_notify_enabled)
	{
		int err;

		fill_state_report(&input_45, report);
		err = bt_gatt_notify(NULL, &hog_service.attrs[VALVE_BLE_STATE_INPUT_ATTR], input_45.data,
		                     input_45.size);
		if(!err)
		{
			input_45_reports_sent++;
			if(input_45_reports_sent <= 8 || (input_45_reports_sent % 512U) == 0U)
			{
				LOG_DBG("BLE report 0x45 sent #%u buttons=0x%08x seq=%u", input_45_reports_sent,
				        report->buttons, input_45.data[TRITON_STATE_SEQUENCE]);
			}
		}
		if(err && input_notify_error_logs < 8)
		{
			input_notify_error_logs++;
			LOG_WRN("BLE report 0x45 notify failed: %d", err);
		}
		return err;
	}

	fill_state_report(&input_42, report);
	int err = bt_gatt_notify(NULL, &hog_service.attrs[VALVE_PRIMARY_INPUT_ATTR], input_42.data,
	                         input_42.size);

	if(!err)
	{
		input_42_reports_sent++;
		if(input_42_reports_sent <= 8 || (input_42_reports_sent % 512U) == 0U)
		{
			LOG_DBG("BLE report 0x42 sent #%u buttons=0x%08x seq=%u", input_42_reports_sent,
			        report->buttons, input_42.data[TRITON_STATE_SEQUENCE]);
		}
	}
	if(err && input_notify_error_logs < 8)
	{
		input_notify_error_logs++;
		LOG_WRN("BLE report 0x42 notify failed: %d", err);
	}
	return err;
}

int transport_ble_send_battery_status(const struct controller_battery_report *report)
{
	if(!report->valid)
	{
		return -EINVAL;
	}

#if defined(CONFIG_BT_BAS)
	(void)bt_bas_set_battery_level(report->level_percent);
#endif
	if(!battery_input_notify_enabled)
	{
		return -ENOTCONN;
	}

	memset(input_43.data, 0, input_43.size);
	input_43.data[0] = report->charge_state;
	input_43.data[1] = report->level_percent;
	sys_put_le16(report->battery_mv, &input_43.data[2]);
	sys_put_le16(report->system_mv, &input_43.data[4]);
	sys_put_le16(report->input_mv, &input_43.data[6]);
	sys_put_le16(report->current_ma, &input_43.data[8]);
	sys_put_le16(report->input_current_ma, &input_43.data[10]);
	sys_put_le16(report->temperature_c, &input_43.data[12]);

	return bt_gatt_notify(NULL, &hog_service.attrs[VALVE_BATTERY_INPUT_ATTR], input_43.data,
	                      input_43.size);
}
