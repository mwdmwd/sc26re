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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>

#include "controller.h"
#include "haptics.h"
#include "lizard.h"
#include "sdl/controller_structs.h"
#include "triton_state_report.h"
#include "valve_feature.h"
#include "valve_hid_report_map.h"
#include "valve_identity.h"

#if defined(CONFIG_BT_BAS)
#include <zephyr/bluetooth/services/bas.h>
#endif

LOG_MODULE_REGISTER(transport_ble);

#define VALVE_DEVICE_NAME_PREFIX "Steam Ctrl (BT) "
#define VALVE_IDENTITY_COUNT 3
#define VALVE_MOUSE_INPUT_ATTR 4
#define VALVE_KEYBOARD_INPUT_ATTR 8
#define VALVE_PRIMARY_INPUT_ATTR 12
#define VALVE_BATTERY_INPUT_ATTR 16
#define VALVE_BLE_STATE_INPUT_ATTR 24
#define BLE_LIZARD_INPUT_QUEUE_DEPTH 16
#define BLE_LIZARD_INPUT_MAX_SIZE 8
#define BLE_DISCONNECT_RETRY_DELAY_MS 100
#define BLE_ADVERTISING_RETRY_DELAY_MS 1000

BUILD_ASSERT(CONFIG_BT_ID_MAX >= VALVE_IDENTITY_COUNT,
             "CONFIG_BT_ID_MAX must be at least as large as VALVE_IDENTITY_COUNT");
BUILD_ASSERT(IS_ENABLED(CONFIG_BT_FILTER_ACCEPT_LIST),
             "BLE peer authorization requires the filter accept list");
BUILD_ASSERT(IS_ENABLED(CONFIG_BT_SMP_APP_PAIRING_ACCEPT),
             "BLE pairing must be controlled by the application");
BUILD_ASSERT(!IS_ENABLED(CONFIG_BT_SMP_ALLOW_UNAUTH_OVERWRITE),
             "unauthenticated BLE peers must not overwrite existing bonds");
BUILD_ASSERT(CONFIG_BT_MAX_CONN == 1,
             "the BLE transport owns one active connection and subscription set");

enum
{
	HIDS_INPUT = 0x01,
	HIDS_OUTPUT = 0x02,
	HIDS_FEATURE = 0x03,
	HIDS_REMOTE_WAKE = BIT(0),
	HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

enum
{
	BLE_SUBSCRIPTION_MOUSE = BIT(0),
	BLE_SUBSCRIPTION_KEYBOARD = BIT(1),
	BLE_SUBSCRIPTION_PRIMARY = BIT(2),
	BLE_SUBSCRIPTION_BATTERY = BIT(3),
	BLE_SUBSCRIPTION_STATE = BIT(4),
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

struct ble_lizard_input_entry
{
	uint8_t id;
	uint8_t len;
	uint8_t data[BLE_LIZARD_INPUT_MAX_SIZE];
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
static bool ble_started;
static bool ble_settings_loaded;
static bool ble_pairing_mode_initialized;
static bool ble_restart_after_recycle;
static bool ble_advertising_suspended;
static uint8_t ble_identity;
static int64_t ble_pairing_deadline_ms;
static struct bt_conn *active_conn;
static uint32_t ble_subscriptions;
static bool ble_disconnect_pending;
static atomic_t ble_pairing_allowed;
static uint32_t input_no_subscription_logs;
static uint32_t input_notify_error_logs;
static uint32_t input_42_reports_sent;
static uint32_t input_45_reports_sent;
K_MSGQ_DEFINE(lizard_input_queue, sizeof(struct ble_lizard_input_entry),
              BLE_LIZARD_INPUT_QUEUE_DEPTH, 1);
static atomic_t lizard_input_queue_draining;
/* Lock lifecycle before state when both are needed. active_conn owns one reference. */
static K_MUTEX_DEFINE(ble_state_mutex);
static K_MUTEX_DEFINE(ble_lifecycle_mutex);
static void ble_pairing_expiry_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(ble_pairing_expiry_work, ble_pairing_expiry_handler);

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

struct ble_connection_snapshot
{
	struct bt_conn *conn;
	uint32_t subscriptions;
};

static struct ble_connection_snapshot ble_connection_snapshot_take(void)
{
	struct ble_connection_snapshot snapshot = { 0 };

	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(ble_started && !ble_advertising_suspended && !ble_disconnect_pending && active_conn != NULL)
	{
		snapshot.conn = bt_conn_ref(active_conn);
		snapshot.subscriptions = ble_subscriptions;
	}
	k_mutex_unlock(&ble_state_mutex);
	return snapshot;
}

static bool ble_conn_is_bonded(struct bt_conn *conn)
{
	struct bt_conn_info info;

	return conn != NULL &&
	       bt_conn_get_info(conn, &info) == 0 &&
	       info.type == BT_CONN_TYPE_LE &&
	       bt_le_bond_exists(info.id, info.le.dst);
}

static bool ble_peer_is_authorized(struct bt_conn *conn)
{
	bool active;

	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	active =
	    ble_started && !ble_advertising_suspended && !ble_disconnect_pending && active_conn == conn;
	k_mutex_unlock(&ble_state_mutex);

	return active && ble_conn_is_bonded(conn);
}

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

	if(report == &feature_01 && !ble_peer_is_authorized(conn))
	{
		return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset, report->data, report->size);
}

static ssize_t read_report_ref(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, attr->user_data, 2);
}

static void handle_output_report(const struct valve_report *report)
{
	int err = haptics_handle_output_report(report->id, report->data, report->size);

	if(err && err != -ENOTSUP && err != -ENODEV && err != -EBUSY)
	{
		LOG_WRN("BLE output report 0x%02x rejected: %d", report->id, err);
	}
}

static ssize_t write_report(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                            uint16_t len, uint16_t offset, uint8_t flags)
{
	struct valve_report *report = attr->user_data;
	bool feature = report->type == HIDS_FEATURE;

	if(feature && !ble_peer_is_authorized(conn))
	{
		return BT_GATT_ERR(BT_ATT_ERR_AUTHORIZATION);
	}

	if(flags & (BT_GATT_WRITE_FLAG_PREPARE | BT_GATT_WRITE_FLAG_EXECUTE))
	{
		return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
	}
	if(offset != 0U)
	{
		LOG_WRN("reject: rpt=0x%02x offset=%u", report->id, offset);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if((feature && (len == 0U || len > report->size)) || (!feature && len != report->size))
	{
		LOG_WRN("reject: rpt=0x%02x len=%u expected%s=%u", report->id, len, feature ? "<" : "",
		        report->size);
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if(feature)
	{
		const uint8_t *request = buf;

		LOG_DBG("direct feature request: opcode 0x%02x, length %u", request[0], len);
		LOG_HEXDUMP_DBG(request, MIN(len, 40), "feature request");
		valve_feature_respond(VALVE_FEATURE_LINK_BLE, buf, len, feature_01.data,
		                      sizeof(feature_01.data));
		return len;
	}

	memcpy(report->data, buf, len);
	handle_output_report(report);
	return len;
}

static ssize_t write_byte(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                          uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);

	if(flags & (BT_GATT_WRITE_FLAG_PREPARE | BT_GATT_WRITE_FLAG_EXECUTE))
	{
		return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
	}
	if(offset != 0)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if(len != 1)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	*(uint8_t *)attr->user_data = *(const uint8_t *)buf;
	return len;
}

static uint32_t ble_report_subscription(const struct valve_report *report)
{
	if(report == &input_40)
	{
		return BLE_SUBSCRIPTION_MOUSE;
	}
	if(report == &input_41)
	{
		return BLE_SUBSCRIPTION_KEYBOARD;
	}
	if(report == &input_42)
	{
		return BLE_SUBSCRIPTION_PRIMARY;
	}
	if(report == &input_43)
	{
		return BLE_SUBSCRIPTION_BATTERY;
	}
	if(report == &input_45)
	{
		return BLE_SUBSCRIPTION_STATE;
	}
	return 0;
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	struct valve_report *report = attr[-1].user_data;
	uint32_t subscription = ble_report_subscription(report);
	bool enabled = value == BT_GATT_CCC_NOTIFY;

	if(subscription != 0U)
	{
		k_mutex_lock(&ble_state_mutex, K_FOREVER);
		if(enabled)
		{
			ble_subscriptions |= subscription;
		}
		else
		{
			ble_subscriptions &= ~subscription;
		}
		k_mutex_unlock(&ble_state_mutex);
	}
	if(enabled && (report == &input_40 || report == &input_41))
	{
		lizard_transport_reset();
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
				   BT_GATT_PERM_WRITE_ENCRYPT, \
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

static struct valve_report *ble_lizard_report(uint8_t report_id, uint8_t *attr_index,
                                              uint32_t *subscription)
{
	switch(report_id)
	{
		case 0x40:
			*attr_index = VALVE_MOUSE_INPUT_ATTR;
			*subscription = BLE_SUBSCRIPTION_MOUSE;
			return &input_40;
		case 0x41:
			*attr_index = VALVE_KEYBOARD_INPUT_ATTR;
			*subscription = BLE_SUBSCRIPTION_KEYBOARD;
			return &input_41;
		default:
			return NULL;
	}
}

static void ble_lizard_input_queue_reset(void)
{
	struct ble_lizard_input_entry entry;

	while(k_msgq_get(&lizard_input_queue, &entry, K_NO_WAIT) == 0)
	{
	}
}

static int ble_lizard_input_queue_drain(struct bt_conn *conn, uint32_t subscriptions)
{
	int result = 0;

	if(!atomic_cas(&lizard_input_queue_draining, 0, 1))
	{
		return 0;
	}

	for(;;)
	{
		struct ble_lizard_input_entry entry;
		struct valve_report *report;
		uint32_t subscription = 0;
		uint8_t attr_index = 0;
		int err;

		if(k_msgq_peek(&lizard_input_queue, &entry) != 0)
		{
			break;
		}

		report = ble_lizard_report(entry.id, &attr_index, &subscription);
		if(report == NULL || entry.len != report->size || (subscriptions & subscription) == 0U)
		{
			err = -ENOTCONN;
		}
		else
		{
			memcpy(report->data, entry.data, entry.len);
			err = bt_gatt_notify(conn, &hog_service.attrs[attr_index], report->data, report->size);
		}

		if(err == -ENOMEM)
		{
			result = -EAGAIN;
			break;
		}

		(void)k_msgq_get(&lizard_input_queue, &entry, K_NO_WAIT);
		if(err)
		{
			result = err;
		}
		else
		{
			result = 0;
		}
	}

	atomic_clear(&lizard_input_queue_draining);
	return result;
}

static uint8_t advertising_flags;
static const struct bt_data advertising[] = {
	BT_DATA(BT_DATA_FLAGS, &advertising_flags, sizeof(advertising_flags)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0xc4, 0x03),
	BT_DATA_BYTES(BT_DATA_TX_POWER, 0x06),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL),
	              BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0x5d, 0x05),
};

struct ble_filter_context
{
	size_t count;
	int err;
};

static bool ble_pairing_window_open(void)
{
	return atomic_get(&ble_pairing_allowed) != 0 && k_uptime_get() < ble_pairing_deadline_ms;
}

static void add_bond_to_filter(const struct bt_bond_info *info, void *user_data)
{
	struct ble_filter_context *context = user_data;

	if(context->err != 0)
	{
		return;
	}
	context->err = bt_le_filter_accept_list_add(&info->addr);
	if(context->err == 0)
	{
		context->count++;
	}
}

static bool ble_advertising_allowed(void)
{
	bool allowed;

	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	allowed = ble_started && !ble_advertising_suspended && active_conn == NULL;
	k_mutex_unlock(&ble_state_mutex);
	return allowed;
}

static int ble_start_advertising_locked(void)
{
	struct ble_filter_context filter = { 0 };
	bool pairing;
	uint32_t options = BT_LE_ADV_OPT_CONN;
	const char *device_name = bt_get_name();
	int err;

	if(!ble_advertising_allowed())
	{
		return 0;
	}
	pairing = ble_pairing_window_open();

	err = bt_le_filter_accept_list_clear();
	if(err)
	{
		LOG_ERR("failed to clear BLE filter accept list: %d", err);
		return err;
	}

	if(pairing)
	{
		advertising_flags = BT_LE_AD_LIMITED | BT_LE_AD_NO_BREDR;
		LOG_INF("advertising in physical-chord pairing mode");
	}
	else
	{
		bt_foreach_bond(ble_identity, add_bond_to_filter, &filter);
		if(filter.err)
		{
			LOG_ERR("failed to populate BLE filter accept list: %d", filter.err);
			return filter.err;
		}
		if(filter.count == 0U)
		{
			LOG_INF("BLE idle: no bond and no A+B boot pairing chord");
			return 0;
		}
		advertising_flags = BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR;
		options |= BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ;
		LOG_INF("advertising to %u bonded peer(s)", (unsigned int)filter.count);
	}

	struct bt_le_adv_param adv_param =
	    BT_LE_ADV_PARAM_INIT(options, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
	const struct bt_data scan_response[] = {
		BT_DATA(BT_DATA_NAME_COMPLETE, device_name, strlen(device_name)),
	};

	adv_param.id = ble_identity;
	return bt_le_adv_start(&adv_param, advertising, ARRAY_SIZE(advertising), scan_response,
	                       ARRAY_SIZE(scan_response));
}

static void ble_stop_advertising_locked(void)
{
	(void)bt_le_adv_stop();
}

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
		bool has_bond = false;

		bt_foreach_bond(id, remember_bond, &has_bond);
		if(has_bond)
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

#if defined(CONFIG_BT_SMP)
#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
static enum bt_security_err ble_pairing_accept_cb(struct bt_conn *conn,
                                                  const struct bt_conn_pairing_feat *const feat)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(feat);

	return ble_pairing_window_open() ? BT_SECURITY_ERR_SUCCESS : BT_SECURITY_ERR_PAIR_NOT_ALLOWED;
}

static const struct bt_conn_auth_cb ble_auth_cb = {
	.pairing_accept = ble_pairing_accept_cb,
};
#endif

static void ble_pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
	struct bt_conn_info info;

	if(bt_conn_get_info(conn, &info) == 0)
	{
		LOG_DBG("pairing complete: id=%u bonded=%u", info.id, bonded);
	}
	else
	{
		LOG_DBG("pairing complete: bonded=%u", bonded);
	}
	if(bonded)
	{
		atomic_clear(&ble_pairing_allowed);
		(void)k_work_cancel_delayable(&ble_pairing_expiry_work);
		LOG_INF("BLE pairing complete; future connections restricted to bonds");
	}
}

static void ble_pairing_failed_cb(struct bt_conn *conn, enum bt_security_err reason)
{
	struct bt_conn_info info;

	if(bt_conn_get_info(conn, &info) == 0)
	{
		LOG_WRN("pairing failed: id=%u reason=%u (%s)", info.id, reason,
		        bt_security_err_to_str(reason));
	}
	else
	{
		LOG_WRN("pairing failed: reason=%u (%s)", reason, bt_security_err_to_str(reason));
	}
}

static struct bt_conn_auth_info_cb ble_auth_info_cb = {
	.pairing_complete = ble_pairing_complete_cb,
	.pairing_failed = ble_pairing_failed_cb,
};

static int register_auth_callbacks(void)
{
	static bool auth_registered;
	static bool auth_info_registered;
	int err;

#if defined(CONFIG_BT_SMP_APP_PAIRING_ACCEPT)
	if(!auth_registered)
	{
		err = bt_conn_auth_cb_register(&ble_auth_cb);
		if(err)
		{
			return err;
		}
		auth_registered = true;
	}
#endif

	if(!auth_info_registered)
	{
		err = bt_conn_auth_info_cb_register(&ble_auth_info_cb);
		if(err && err != -EALREADY)
		{
			return err;
		}

		auth_info_registered = true;
	}
	return 0;
}
#else
static int register_auth_callbacks(void)
{
	return 0;
}
#endif

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

static int ble_request_disconnect(struct bt_conn *conn)
{
	int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

	if(err == -ENOTCONN)
	{
		return 0;
	}
	return err;
}

static void ble_disconnect_retry_handler(struct k_work *work)
{
	struct bt_conn *conn = NULL;
	int err = 0;

	k_mutex_lock(&ble_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(ble_disconnect_pending && active_conn != NULL)
	{
		conn = bt_conn_ref(active_conn);
	}
	k_mutex_unlock(&ble_state_mutex);

	if(conn != NULL)
	{
		err = ble_request_disconnect(conn);
		bt_conn_unref(conn);
	}
	k_mutex_unlock(&ble_lifecycle_mutex);

	if(err)
	{
		(void)k_work_reschedule(k_work_delayable_from_work(work),
		                        K_MSEC(BLE_DISCONNECT_RETRY_DELAY_MS));
	}
}

K_WORK_DELAYABLE_DEFINE(ble_disconnect_retry_work, ble_disconnect_retry_handler);

static void ble_disconnect_or_retry(struct bt_conn *conn)
{
	int err = ble_request_disconnect(conn);

	if(err)
	{
		LOG_WRN("BLE disconnect request failed: %d; retrying", err);
		(void)k_work_reschedule(&ble_disconnect_retry_work, K_MSEC(BLE_DISCONNECT_RETRY_DELAY_MS));
	}
}

/*
 * Request fast connection parameters as soon as a central connects.
 * The default connection interval chosen by Linux/BlueZ is typically
 * 30-50 ms, which is visibly laggy for HID input.
 */
static void ble_connected_cb(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info;
	struct bt_conn *old_conn = NULL;
	struct bt_conn *new_conn;
	bool accepted;

	if(err)
	{
		k_mutex_lock(&ble_state_mutex, K_FOREVER);
		ble_restart_after_recycle = ble_started;
		k_mutex_unlock(&ble_state_mutex);
		return;
	}

	new_conn = bt_conn_ref(conn);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	accepted = ble_started && !ble_advertising_suspended && !ble_disconnect_pending;
	if(accepted)
	{
		old_conn = active_conn;
		active_conn = new_conn;
		new_conn = NULL;
		ble_disconnect_pending = false;
		ble_restart_after_recycle = false;
	}
	else if(active_conn == NULL)
	{
		active_conn = new_conn;
		new_conn = NULL;
		ble_subscriptions = 0;
		ble_disconnect_pending = true;
	}
	k_mutex_unlock(&ble_state_mutex);

	if(!accepted)
	{
		ble_disconnect_or_retry(conn);
		if(new_conn != NULL)
		{
			bt_conn_unref(new_conn);
		}
		return;
	}
	if(old_conn != NULL)
	{
		bt_conn_unref(old_conn);
	}

	if(bt_conn_get_info(conn, &info) == 0)
	{
		LOG_INF("connected: CI=%u us (%u.%03u ms)", info.le.interval_us, info.le.interval_us / 1000,
		        info.le.interval_us % 1000);
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

#if defined(CONFIG_BT_SMP)
static void ble_security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                    enum bt_security_err err)
{
	struct bt_conn_info info;

	if(bt_conn_get_info(conn, &info) == 0)
	{
		if(err)
		{
			LOG_WRN("security failed: id=%u level=%u err=%u (%s)", info.id, level, err,
			        bt_security_err_to_str(err));
		}
		else
		{
			LOG_DBG("security changed: id=%u level=%u", info.id, level);
		}
	}
	else if(err)
	{
		LOG_WRN("security failed: level=%u err=%u (%s)", level, err, bt_security_err_to_str(err));
	}
	else
	{
		LOG_DBG("security changed: level=%u", level);
	}
}
#endif

static void ble_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn *detached = NULL;

	LOG_INF("disconnected (reason 0x%02x)", reason);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(active_conn == conn)
	{
		detached = active_conn;
		active_conn = NULL;
		ble_subscriptions = 0;
		ble_disconnect_pending = false;
		ble_restart_after_recycle = ble_started;
	}
	k_mutex_unlock(&ble_state_mutex);

	if(detached != NULL)
	{
		bt_conn_unref(detached);
		ble_lizard_input_queue_reset();
	}
}

static void ble_restart_advertising_handler(struct k_work *work)
{
	bool restart;
	bool retry;
	int err;

	k_mutex_lock(&ble_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	restart = ble_started &&
	          ble_restart_after_recycle &&
	          !ble_advertising_suspended &&
	          active_conn == NULL;
	if(restart)
	{
		ble_restart_after_recycle = false;
	}
	k_mutex_unlock(&ble_state_mutex);

	if(!restart)
	{
		k_mutex_unlock(&ble_lifecycle_mutex);
		return;
	}
	err = ble_start_advertising_locked();
	if(err == -EALREADY)
	{
		err = 0;
	}
	if(err)
	{
		LOG_ERR("failed to restart BLE advertising: %d", err);
		k_mutex_lock(&ble_state_mutex, K_FOREVER);
		retry = ble_started && !ble_advertising_suspended && active_conn == NULL;
		if(retry)
		{
			ble_restart_after_recycle = true;
		}
		k_mutex_unlock(&ble_state_mutex);
	}
	else
	{
		retry = false;
	}
	k_mutex_unlock(&ble_lifecycle_mutex);

	if(retry)
	{
		(void)k_work_reschedule(k_work_delayable_from_work(work),
		                        K_MSEC(BLE_ADVERTISING_RETRY_DELAY_MS));
	}
}

K_WORK_DELAYABLE_DEFINE(ble_restart_advertising_work, ble_restart_advertising_handler);

static void ble_pairing_expiry_handler(struct k_work *work)
{
	struct bt_conn *conn = NULL;
	bool retry = false;
	int err = 0;

	ARG_UNUSED(work);

	if(!atomic_cas(&ble_pairing_allowed, 1, 0))
	{
		return;
	}
	LOG_INF("BLE pairing window expired");

	k_mutex_lock(&ble_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(ble_started && !ble_advertising_suspended && active_conn != NULL)
	{
		conn = bt_conn_ref(active_conn);
	}
	k_mutex_unlock(&ble_state_mutex);

	if(conn != NULL && !ble_conn_is_bonded(conn))
	{
		k_mutex_lock(&ble_state_mutex, K_FOREVER);
		if(ble_started && !ble_advertising_suspended && active_conn == conn)
		{
			ble_disconnect_pending = true;
		}
		k_mutex_unlock(&ble_state_mutex);
		LOG_INF("disconnecting unbonded peer at pairing-window expiry");
		ble_disconnect_or_retry(conn);
	}
	else if(conn == NULL)
	{
		ble_stop_advertising_locked();
		err = ble_start_advertising_locked();
		if(err == -EALREADY)
		{
			err = 0;
		}
		if(err)
		{
			LOG_ERR("failed to enter bonded advertising after pairing expiry: %d", err);
			k_mutex_lock(&ble_state_mutex, K_FOREVER);
			retry = ble_started && !ble_advertising_suspended && active_conn == NULL;
			if(retry)
			{
				ble_restart_after_recycle = true;
			}
			k_mutex_unlock(&ble_state_mutex);
		}
	}
	if(conn != NULL)
	{
		bt_conn_unref(conn);
	}
	k_mutex_unlock(&ble_lifecycle_mutex);

	if(retry)
	{
		(void)k_work_reschedule(&ble_restart_advertising_work,
		                        K_MSEC(BLE_ADVERTISING_RETRY_DELAY_MS));
	}
}

static void ble_recycled_cb(void)
{
	(void)k_work_reschedule(&ble_restart_advertising_work, K_NO_WAIT);
}

bool transport_ble_connected(void)
{
	bool connected;

	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	connected =
	    ble_started && !ble_advertising_suspended && !ble_disconnect_pending && active_conn != NULL;
	k_mutex_unlock(&ble_state_mutex);
	return connected;
}

BT_CONN_CB_DEFINE(ble_conn_cbs) = {
	.connected = ble_connected_cb,
	.disconnected = ble_disconnected_cb,
	.recycled = ble_recycled_cb,
	.le_param_updated = ble_param_updated_cb,
#if defined(CONFIG_BT_SMP)
	.security_changed = ble_security_changed_cb,
#endif
};

int transport_ble_init(void)
{
	uint8_t selected_id;
	bool retry_advertising;
	int64_t pairing_delay_ms;
	int err = 0;

	k_mutex_lock(&ble_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(ble_started)
	{
		k_mutex_unlock(&ble_state_mutex);
		goto out;
	}
	if(!ble_pairing_mode_initialized)
	{
		bool pairing_requested = hardware_pairing_chord_pressed();

		atomic_set(&ble_pairing_allowed, pairing_requested ? 1 : 0);
		ble_pairing_deadline_ms =
		    k_uptime_get() + (int64_t)CONFIG_BT_LIM_ADV_TIMEOUT * MSEC_PER_SEC;
		ble_pairing_mode_initialized = true;
	}
	k_mutex_unlock(&ble_state_mutex);

	valve_hid_report_map_copy_ble(valve_ble_hid_report_map, sizeof(valve_ble_hid_report_map));

	err = bt_enable(NULL);
	if(err && err != -EALREADY)
	{
		goto out;
	}

	err = register_auth_callbacks();
	if(err)
	{
		goto out;
	}

	if(IS_ENABLED(CONFIG_SETTINGS) && !ble_settings_loaded)
	{
		err = settings_load_subtree("bt");
		if(err)
		{
			LOG_ERR("failed to load Bluetooth settings: %d", err);
			goto out;
		}
		ble_settings_loaded = true;
	}

	err = set_ble_identity_strings();
	if(err)
	{
		goto out;
	}

	err = select_valve_identity(&selected_id);
	if(err)
	{
		goto out;
	}

	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	ble_identity = selected_id;
	ble_restart_after_recycle = false;
	ble_advertising_suspended = false;
	ble_started = true;
	k_mutex_unlock(&ble_state_mutex);
	LOG_INF("starting \"%s\" on identity %u with Valve HID map", bt_get_name(), selected_id);
	err = ble_start_advertising_locked();
	if(err == -ENOMEM)
	{
		k_mutex_lock(&ble_state_mutex, K_FOREVER);
		retry_advertising = ble_started && active_conn == NULL;
		if(retry_advertising)
		{
			ble_restart_after_recycle = true;
		}
		k_mutex_unlock(&ble_state_mutex);
		if(retry_advertising)
		{
			(void)k_work_reschedule(&ble_restart_advertising_work,
			                        K_MSEC(BLE_ADVERTISING_RETRY_DELAY_MS));
		}
		err = 0;
	}
	else if(err)
	{
		k_mutex_lock(&ble_state_mutex, K_FOREVER);
		ble_started = false;
		ble_advertising_suspended = true;
		k_mutex_unlock(&ble_state_mutex);
	}
	if(!err && atomic_get(&ble_pairing_allowed) != 0)
	{
		pairing_delay_ms = MAX(ble_pairing_deadline_ms - k_uptime_get(), 0);
		(void)k_work_reschedule(&ble_pairing_expiry_work, K_MSEC(pairing_delay_ms));
	}

out:
	k_mutex_unlock(&ble_lifecycle_mutex);
	return err;
}

int transport_ble_clear_bonds(uint8_t id)
{
	struct bt_conn *conn = NULL;
	bool restart_advertising;
	int err = 0;
	int advertising_err;

	if(!IS_ENABLED(CONFIG_BT_SMP))
	{
		return -ENOTSUP;
	}

	k_mutex_lock(&ble_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(!ble_started)
	{
		k_mutex_unlock(&ble_state_mutex);
		err = -ENOTCONN;
		goto out;
	}
	if(id != TRANSPORT_BLE_ID_ALL && id >= CONFIG_BT_ID_MAX)
	{
		k_mutex_unlock(&ble_state_mutex);
		err = -EINVAL;
		goto out;
	}
	ble_advertising_suspended = true;
	ble_restart_after_recycle = false;
	if(active_conn != NULL && (id == TRANSPORT_BLE_ID_ALL || id == ble_identity))
	{
		ble_disconnect_pending = true;
		conn = bt_conn_ref(active_conn);
	}
	k_mutex_unlock(&ble_state_mutex);
	ble_stop_advertising_locked();

	if(id == TRANSPORT_BLE_ID_ALL)
	{
		for(uint8_t valve_id = 1; valve_id < VALVE_IDENTITY_COUNT; ++valve_id)
		{
			int unpair_err = bt_unpair(valve_id, NULL);

			if(unpair_err && err == 0)
			{
				err = unpair_err;
			}
		}
	}
	else
	{
		err = bt_unpair(id, NULL);
	}
	if(conn != NULL)
	{
		ble_disconnect_or_retry(conn);
		bt_conn_unref(conn);
	}

	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(ble_started)
	{
		ble_advertising_suspended = false;
	}
	restart_advertising = ble_started && active_conn == NULL;
	if(restart_advertising)
	{
		ble_restart_after_recycle = false;
	}
	k_mutex_unlock(&ble_state_mutex);
	if(restart_advertising)
	{
		advertising_err = ble_start_advertising_locked();
		if(advertising_err)
		{
			bool retry;

			k_mutex_lock(&ble_state_mutex, K_FOREVER);
			retry = ble_started && !ble_advertising_suspended && active_conn == NULL;
			if(retry)
			{
				ble_restart_after_recycle = true;
			}
			k_mutex_unlock(&ble_state_mutex);
			if(!err)
			{
				err = advertising_err;
			}
			if(retry)
			{
				(void)k_work_reschedule(&ble_restart_advertising_work,
				                        K_MSEC(BLE_ADVERTISING_RETRY_DELAY_MS));
			}
		}
	}

out:
	k_mutex_unlock(&ble_lifecycle_mutex);
	return err;
}

void transport_ble_deactivate(void)
{
	struct bt_conn *conn = NULL;

	(void)k_work_cancel_delayable(&ble_pairing_expiry_work);
	k_mutex_lock(&ble_lifecycle_mutex, K_FOREVER);
	k_mutex_lock(&ble_state_mutex, K_FOREVER);
	if(!ble_started && active_conn == NULL)
	{
		k_mutex_unlock(&ble_state_mutex);
		k_mutex_unlock(&ble_lifecycle_mutex);
		return;
	}

	ble_started = false;
	ble_restart_after_recycle = false;
	ble_advertising_suspended = true;
	if(active_conn != NULL)
	{
		ble_disconnect_pending = true;
		conn = bt_conn_ref(active_conn);
	}
	k_mutex_unlock(&ble_state_mutex);

	ble_stop_advertising_locked();
	if(conn != NULL)
	{
		ble_disconnect_or_retry(conn);
		bt_conn_unref(conn);
	}
	ble_lizard_input_queue_reset();
	k_mutex_unlock(&ble_lifecycle_mutex);
}

static void fill_state_report(struct valve_report *state, const struct controller_report *report)
{
	triton_state_report_pack_body(state->data, state->size, input_sequence++, report,
	                              triton_state_report_timestamp_us());
}

int transport_ble_send(const struct controller_report *report)
{
	struct ble_connection_snapshot snapshot = ble_connection_snapshot_take();
	int err;

	if(snapshot.conn == NULL)
	{
		return -ENOTCONN;
	}

	(void)ble_lizard_input_queue_drain(snapshot.conn, snapshot.subscriptions);

	if((snapshot.subscriptions & (BLE_SUBSCRIPTION_PRIMARY | BLE_SUBSCRIPTION_STATE)) == 0U)
	{
		if(input_no_subscription_logs < 8)
		{
			input_no_subscription_logs++;
			LOG_DBG("BLE input not sent: no 0x42/0x45 notification subscription");
		}
		bt_conn_unref(snapshot.conn);
		return -ENOTCONN;
	}

	/*
	 * Compact BLE controller-state layout:
	 *   byte 0: sequence
	 *   bytes 1-4: little-endian button bitmap
	 *   bytes 5-16: little-endian trigger and stick axes
	 *   bytes 17-28: little-endian touchpad axes and pressures
	 *   bytes 29-32: little-endian IMU timestamp in microseconds
	 *   bytes 33-38: little-endian signed accelerometer XYZ, 61 ug/LSB
	 *   bytes 39-44: little-endian signed gyro XYZ, 16.384 units/degree/s
	 *   report 0x42 only: bytes 45-52 quaternion WXYZ, Q15
	 */
	if((snapshot.subscriptions & BLE_SUBSCRIPTION_STATE) != 0U)
	{
		fill_state_report(&input_45, report);
		err = bt_gatt_notify(snapshot.conn, &hog_service.attrs[VALVE_BLE_STATE_INPUT_ATTR],
		                     input_45.data, input_45.size);
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
		bt_conn_unref(snapshot.conn);
		return err == -ENOMEM ? -EAGAIN : err;
	}

	fill_state_report(&input_42, report);
	err = bt_gatt_notify(snapshot.conn, &hog_service.attrs[VALVE_PRIMARY_INPUT_ATTR], input_42.data,
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
	bt_conn_unref(snapshot.conn);
	return err == -ENOMEM ? -EAGAIN : err;
}

int transport_ble_send_input_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	struct ble_lizard_input_entry entry = {
		.id = report_id,
		.len = (uint8_t)len,
	};
	struct ble_connection_snapshot snapshot;
	struct valve_report *report;
	uint32_t subscription = 0;
	uint8_t attr_index = 0;
	int err;

	report = ble_lizard_report(report_id, &attr_index, &subscription);
	if(report == NULL || data == NULL || len != report->size || len > BLE_LIZARD_INPUT_MAX_SIZE)
	{
		return -EINVAL;
	}

	snapshot = ble_connection_snapshot_take();
	if(snapshot.conn == NULL || (snapshot.subscriptions & subscription) == 0U)
	{
		if(snapshot.conn != NULL)
		{
			bt_conn_unref(snapshot.conn);
		}
		return -ENOTCONN;
	}
	memcpy(entry.data, data, len);

	err = k_msgq_put(&lizard_input_queue, &entry, K_NO_WAIT);
	if(err)
	{
		bt_conn_unref(snapshot.conn);
		return err == -ENOMSG ? -ENOMEM : err;
	}

	err = ble_lizard_input_queue_drain(snapshot.conn, snapshot.subscriptions);
	bt_conn_unref(snapshot.conn);
	return err == -EAGAIN ? 0 : err;
}

int transport_ble_send_battery_status(const struct controller_battery_report *report)
{
	struct ble_connection_snapshot snapshot;
	int err;

	if(!report->valid)
	{
		return -EINVAL;
	}

#if defined(CONFIG_BT_BAS)
	(void)bt_bas_set_battery_level(report->level_percent);
#endif
	snapshot = ble_connection_snapshot_take();
	if(snapshot.conn == NULL || (snapshot.subscriptions & BLE_SUBSCRIPTION_BATTERY) == 0U)
	{
		if(snapshot.conn != NULL)
		{
			bt_conn_unref(snapshot.conn);
		}
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
	sys_put_le16(report->temperature_mc, &input_43.data[12]);

	err = bt_gatt_notify(snapshot.conn, &hog_service.attrs[VALVE_BATTERY_INPUT_ATTR], input_43.data,
	                     input_43.size);
	bt_conn_unref(snapshot.conn);
	return err;
}
