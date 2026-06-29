/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#include "controller.h"
#include "esb_backend.h"
#include "sdl/controller_constants.h"
#include "sdl/controller_structs.h"
#include "triton_state_report.h"
#include "valve_feature.h"
#include "valve_identity.h"

LOG_MODULE_REGISTER(transport_esb);

enum
{
	ESB_DISCOVERY_CHANNEL = 2,
	ESB_HOST_FRAME = 0xE1,
	ESB_HOST_IDLE = 0xE2,
	ESB_HOST_POLL = 0xE3,
	ESB_HOST_CHANNEL_MAP = 0xE4,
	ESB_HOST_AWAKE = 0xE7,
	ESB_CONTROLLER_INPUT = 0xF1,
	ESB_CONTROLLER_STATUS = 0xF3,
	ESB_APP_REPORT_WRITE = 0x01,
	ESB_APP_REPORT_READ = 0x03,
	ESB_APP_LEGACY_SET = 0x05,
	ESB_TLV_WRITE_RESULT = 0x02,
	ESB_TLV_FEATURE_RESPONSE = 0x04,
	ESB_REPORT_TLV = 0x06,
	ESB_FEATURE_REPORT_SIZE = VALVE_FEATURE_REPORT_SIZE - 1,
	ESB_REPORT_42_ID = ID_TRITON_CONTROLLER_STATE,
	ESB_REPORT_42_SIZE = 54,
	ESB_REPORT_43_ID = ID_TRITON_BATTERY_STATUS,
	ESB_REPORT_43_SIZE = 15,
	ESB_REPORT_45_ID = ID_TRITON_CONTROLLER_STATE_BLE,
	ESB_REPORT_45_SIZE = 46,
	ESB_MAX_PAYLOAD_SIZE = 128,
	ESB_CHANNEL_MAP_SIZE = 4,
	ESB_CHANNEL_UNUSED = 0xff,
	ESB_CHANNEL_SCAN_INITIAL_MS = 328,
	ESB_CHANNEL_SCAN_INTERVAL_MS = 426,
};

static const uint8_t discovery_base[] = { 'i', 'b', 'e', 'x' };
static const uint8_t discovery_prefix[] = { 0x10 };
static struct valve_esb_payload latest_input_42;
static struct valve_esb_payload latest_battery_43;
static struct valve_esb_payload latest_input_45;
static struct valve_esb_payload rx_payload;
static struct k_spinlock latest_input_lock;
static uint8_t pending_feature_response[ESB_FEATURE_REPORT_SIZE];
static size_t pending_feature_response_len;
static uint8_t esb_feature_response[ESB_FEATURE_REPORT_SIZE];
static uint8_t last_feature_response[ESB_FEATURE_REPORT_SIZE];
static size_t last_feature_response_len;
static int32_t pending_write_result;
static bool pending_write_result_valid;
static uint32_t pending_feature_generation;
static uint32_t pending_write_generation;
static uint8_t report_sequence;
static bool latest_input_valid;
static bool latest_battery_valid;
static bool session_pending;
static uint8_t pending_channel;
static uint8_t pending_base[4];
static uint8_t pending_prefix;
static uint8_t current_channel = ESB_DISCOVERY_CHANNEL;
static uint8_t channel_map[ESB_CHANNEL_MAP_SIZE] = {
	ESB_DISCOVERY_CHANNEL,
	ESB_CHANNEL_UNUSED,
	ESB_CHANNEL_UNUSED,
	ESB_CHANNEL_UNUSED,
};
static uint8_t channel_map_index;
static uint8_t pending_map_channel = ESB_CHANNEL_UNUSED;
static int64_t last_host_rx_ms;
static bool session_connected;
static bool esb_started;
static uint32_t host_frames_received;
static uint32_t channel_maps_received;
static uint32_t awake_frames_received;
static uint32_t polls_received;
static uint32_t replies_sent;
static uint8_t pairing_bond[24];
static bool pairing_bond_valid;

enum
{
	ESB_BOND_PREFIX_SIZE = 20,
	ESB_BOND_SERIAL_OFFSET = 8,
	ESB_BOND_SERIAL_STORED_SIZE = ESB_BOND_PREFIX_SIZE - ESB_BOND_SERIAL_OFFSET,
};
static uint8_t rejected_host_frame_logs_remaining = 8;
static uint8_t unknown_host_opcode_logs_remaining = 8;

static int esb_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	ssize_t read_len;

	if(strcmp(name, "bond") != 0)
	{
		return -ENOENT;
	}
	if(len != sizeof(pairing_bond))
	{
		return -EINVAL;
	}

	read_len = read_cb(cb_arg, pairing_bond, sizeof(pairing_bond));
	if(read_len < 0)
	{
		return read_len;
	}
	if(read_len != sizeof(pairing_bond))
	{
		return -EINVAL;
	}

	pairing_bond_valid = true;
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(esb, "esb", NULL, esb_settings_set, NULL, NULL);

static int queue_payload(const struct valve_esb_payload *payload)
{
	int err;

	err = valve_esb_backend_write_payload(payload);
	if(err && err != -ENOMEM)
	{
		LOG_DBG("failed to queue ESB ACK payload: %d", err);
	}
	return err;
}

static void stop_and_flush(void)
{
	(void)valve_esb_backend_stop_rx();
	(void)valve_esb_backend_flush_rx();
	(void)valve_esb_backend_flush_tx();
}

static void configure_address(const uint8_t *base, const uint8_t *prefix)
{
	stop_and_flush();
	(void)valve_esb_backend_set_rf_channel(ESB_DISCOVERY_CHANNEL);
	(void)valve_esb_backend_set_base_address_0(base);
	(void)valve_esb_backend_set_prefixes(prefix, 1);
	current_channel = ESB_DISCOVERY_CHANNEL;
}

static void switch_channel(uint8_t channel)
{
	if(channel == current_channel || channel == ESB_CHANNEL_UNUSED)
	{
		return;
	}

	(void)valve_esb_backend_stop_rx();
	(void)valve_esb_backend_set_rf_channel(channel);
	current_channel = channel;
	(void)valve_esb_backend_start_rx();
}

static void channel_map_work_handler(struct k_work *work)
{
	uint8_t channel = pending_map_channel;

	ARG_UNUSED(work);

	if(!esb_started)
	{
		return;
	}

	/* The original controller gives the puck's E4 reply time to finish. */
	k_usleep(500);

	if(!esb_started)
	{
		return;
	}

	switch_channel(channel);
}

K_WORK_DEFINE(channel_map_work, channel_map_work_handler);

static bool is_esb_input_report_id(uint8_t report_id)
{
	switch(report_id)
	{
		case ESB_REPORT_42_ID:
		case ESB_REPORT_43_ID:
		case ESB_REPORT_45_ID:
			return true;
		default:
			return false;
	}
}

static void queue_latest_input(uint8_t requested_report_id)
{
	struct valve_esb_payload payload;
	k_spinlock_key_t key;
	size_t feature_len;
	uint32_t feature_generation;
	uint32_t write_generation;
	bool write_valid;
	bool write_included = false;
	bool report_included = false;
	int err;

	if(requested_report_id == 0)
	{
		requested_report_id = ESB_REPORT_45_ID;
	}

	key = k_spin_lock(&latest_input_lock);
	payload = (struct valve_esb_payload){
		.length = 1,
		.pipe = 0,
	};
	payload.data[0] = ESB_CONTROLLER_INPUT;
	feature_len = pending_feature_response_len;
	feature_generation = pending_feature_generation;
	write_generation = pending_write_generation;
	write_valid = pending_write_result_valid;
	if(feature_len != 0 && payload.length + feature_len + 2 <= ESB_MAX_PAYLOAD_SIZE)
	{
		payload.data[payload.length++] = feature_len;
		payload.data[payload.length++] = ESB_TLV_FEATURE_RESPONSE;
		memcpy(&payload.data[payload.length], pending_feature_response, feature_len);
		payload.length += feature_len;
	}
	if(feature_len == 0 &&
	   write_valid &&
	   payload.length + sizeof(pending_write_result) + 2 <= ESB_MAX_PAYLOAD_SIZE)
	{
		payload.data[payload.length++] = sizeof(pending_write_result);
		payload.data[payload.length++] = ESB_TLV_WRITE_RESULT;
		sys_put_le32(pending_write_result, &payload.data[payload.length]);
		payload.length += sizeof(pending_write_result);
		write_included = true;
	}
	if(requested_report_id == ESB_REPORT_42_ID &&
	   latest_input_valid &&
	   payload.length + latest_input_42.length - 1 <= ESB_MAX_PAYLOAD_SIZE)
	{
		memcpy(&payload.data[payload.length], &latest_input_42.data[1], latest_input_42.length - 1);
		payload.length += latest_input_42.length - 1;
		report_included = true;
	}
	if(requested_report_id == ESB_REPORT_45_ID &&
	   latest_input_valid &&
	   payload.length + latest_input_45.length - 1 <= ESB_MAX_PAYLOAD_SIZE)
	{
		memcpy(&payload.data[payload.length], &latest_input_45.data[1], latest_input_45.length - 1);
		payload.length += latest_input_45.length - 1;
		report_included = true;
	}
	if(requested_report_id == ESB_REPORT_43_ID &&
	   latest_battery_valid &&
	   payload.length + latest_battery_43.length - 1 <= ESB_MAX_PAYLOAD_SIZE)
	{
		memcpy(&payload.data[payload.length], &latest_battery_43.data[1],
		       latest_battery_43.length - 1);
		payload.length += latest_battery_43.length - 1;
		report_included = true;
	}
	k_spin_unlock(&latest_input_lock, key);
	if(feature_len == 0 && !write_included && !report_included)
	{
		return;
	}

	err = queue_payload(&payload);
	if(err)
	{
		return;
	}

	key = k_spin_lock(&latest_input_lock);
	if(feature_len != 0 && feature_generation == pending_feature_generation)
	{
		pending_feature_response_len = 0;
	}
	if(write_included && write_generation == pending_write_generation)
	{
		pending_write_result_valid = false;
	}
	k_spin_unlock(&latest_input_lock, key);
}

static void session_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if(!esb_started || !session_pending)
	{
		return;
	}

	(void)valve_esb_backend_stop_rx();
	(void)valve_esb_backend_flush_rx();
	(void)valve_esb_backend_flush_tx();
	(void)valve_esb_backend_set_rf_channel(pending_channel);
	(void)valve_esb_backend_set_base_address_0(pending_base);
	(void)valve_esb_backend_set_prefixes(&pending_prefix, 1);
	current_channel = pending_channel;
	channel_map[0] = pending_channel;
	for(size_t i = 1; i < ARRAY_SIZE(channel_map); ++i)
	{
		channel_map[i] = ESB_CHANNEL_UNUSED;
	}
	channel_map_index = 0;
	last_host_rx_ms = k_uptime_get();
	session_connected = true;
	session_pending = false;
	queue_latest_input(ESB_REPORT_45_ID);
	(void)valve_esb_backend_start_rx();

	LOG_INF("ESB session: channel %u, base %02x%02x%02x%02x, prefix %02x", pending_channel,
	        pending_base[0], pending_base[1], pending_base[2], pending_base[3], pending_prefix);
}

K_WORK_DEFINE(session_work, session_work_handler);

static void channel_scan_work_handler(struct k_work *work)
{
	int64_t silence_ms;

	ARG_UNUSED(work);

	if(!esb_started)
	{
		return;
	}

	if(!session_connected || session_pending)
	{
		goto out;
	}

	silence_ms = k_uptime_get() - last_host_rx_ms;
	if(silence_ms < ESB_CHANNEL_SCAN_INITIAL_MS)
	{
		goto out;
	}

	for(size_t i = 0; i < ARRAY_SIZE(channel_map); ++i)
	{
		channel_map_index = (channel_map_index + 1) % ARRAY_SIZE(channel_map);
		if(channel_map[channel_map_index] != ESB_CHANNEL_UNUSED)
		{
			switch_channel(channel_map[channel_map_index]);
			break;
		}
	}

out:
	if(esb_started)
	{
		k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(ESB_CHANNEL_SCAN_INTERVAL_MS));
	}
}

K_WORK_DELAYABLE_DEFINE(channel_scan_work, channel_scan_work_handler);

static void handle_host_frame(const struct valve_esb_payload *payload)
{
	if(payload->length < 18)
	{
		if(rejected_host_frame_logs_remaining != 0)
		{
			LOG_INF("short ESB E1 frame length %u", payload->length);
			rejected_host_frame_logs_remaining--;
		}
		return;
	}
	if(!pairing_bond_valid || memcmp(&payload->data[1], pairing_bond, 8) != 0)
	{
		if(rejected_host_frame_logs_remaining != 0)
		{
			LOG_INF("ignoring ESB E1 %08x/%08x; bond loaded=%u expected=%08x/%08x",
			        sys_get_le32(&payload->data[1]), sys_get_le32(&payload->data[5]),
			        pairing_bond_valid, sys_get_le32(&pairing_bond[0]),
			        sys_get_le32(&pairing_bond[4]));
			rejected_host_frame_logs_remaining--;
		}
		return;
	}

	host_frames_received++;
	pending_channel = payload->data[9];
	memcpy(pending_base, &payload->data[13], sizeof(pending_base));
	pending_prefix = payload->data[17];
	session_pending = true;
	k_work_submit(&session_work);
}

static void stage_feature_response(const uint8_t *response, size_t response_len)
{
	k_spinlock_key_t key;

	if(response_len > sizeof(pending_feature_response))
	{
		response_len = sizeof(pending_feature_response);
	}

	key = k_spin_lock(&latest_input_lock);
	memcpy(pending_feature_response, response, response_len);
	pending_feature_response_len = response_len;
	pending_feature_generation++;
	k_spin_unlock(&latest_input_lock, key);
}

static void remember_feature_response(const uint8_t *response, size_t response_len)
{
	if(response_len > sizeof(last_feature_response))
	{
		response_len = sizeof(last_feature_response);
	}

	memcpy(last_feature_response, response, response_len);
	last_feature_response_len = response_len;
}

static void clear_last_feature_response(void)
{
	memset(last_feature_response, 0, sizeof(last_feature_response));
	last_feature_response_len = 0;
}

static void stage_write_result(int32_t result)
{
	k_spinlock_key_t key;

	key = k_spin_lock(&latest_input_lock);
	pending_write_result = result;
	pending_write_result_valid = true;
	pending_write_generation++;
	k_spin_unlock(&latest_input_lock, key);
}

static bool feature_request_expects_response(const uint8_t *request, size_t request_len)
{
	if(request_len == 0)
	{
		return false;
	}

	switch(request[0])
	{
		case VALVE_FEATURE_GET_DIGITAL_MAPPINGS:
		case VALVE_FEATURE_GET_ATTRIBUTES_VALUES:
		case VALVE_FEATURE_GET_SETTINGS_VALUES:
		case VALVE_FEATURE_GET_SETTINGS_MAXS:
		case VALVE_FEATURE_GET_SETTINGS_DEFAULTS:
		case VALVE_FEATURE_GET_DEVICE_INFO:
		case VALVE_FEATURE_GET_STRING_ATTRIBUTE:
		case VALVE_FEATURE_GET_CHIPID:
		case VALVE_FEATURE_GET_SYSTEM_INFO:
		case VALVE_FEATURE_READ_SETTING:
			return true;
		default:
			return false;
	}
}

static void handle_host_poll(const struct valve_esb_payload *payload)
{
	static uint8_t diagnostic_logs_remaining = 20;
	size_t index = 1;
	uint8_t requested_report_id = 0;

	polls_received++;
	if(diagnostic_logs_remaining != 0 && !(payload->length == 5 &&
	                                       payload->data[1] == 2 &&
	                                       payload->data[2] == ESB_APP_REPORT_WRITE &&
	                                       payload->data[3] == ID_TRITON_CONTROLLER_STATE_BLE))
	{
		LOG_HEXDUMP_INF(payload->data, payload->length, "non-input ESB E3");
		diagnostic_logs_remaining--;
	}

	while(index + 1 < payload->length)
	{
		const uint8_t body_len = payload->data[index];
		const uint8_t subtype = payload->data[index + 1];
		const uint8_t *body = &payload->data[index + 2];

		if(index + body_len + 2 > payload->length)
		{
			LOG_DBG("truncated E3 APP_MSG subtype %u", subtype);
			break;
		}

		switch(subtype)
		{
			case ESB_APP_REPORT_WRITE:
			{
				ssize_t n;

				if(body_len > 0 && is_esb_input_report_id(body[0]))
				{
					requested_report_id = body[0];
					break;
				}

				n = valve_feature_respond(VALVE_FEATURE_LINK_ESB, body, body_len,
				                          esb_feature_response, sizeof(esb_feature_response));

				if(n >= 0)
				{
					remember_feature_response(esb_feature_response, n);
					if(feature_request_expects_response(body, body_len) ||
					   esb_feature_response[1] != 0)
					{
						stage_feature_response(esb_feature_response, n);
					}
					else
					{
						stage_write_result(0);
					}
				}
				else
				{
					clear_last_feature_response();
					stage_write_result((int32_t)n);
				}
				break;
			}
			case ESB_APP_REPORT_READ:
				if(last_feature_response_len != 0)
				{
					stage_feature_response(last_feature_response, last_feature_response_len);
				}
				else
				{
					LOG_DBG("E3 feature read requested with no pending response");
				}
				break;
			case ESB_APP_LEGACY_SET:
			{
				ssize_t n;

				n = valve_feature_respond(VALVE_FEATURE_LINK_ESB, body, body_len,
				                          esb_feature_response, sizeof(esb_feature_response));
				stage_write_result(n >= 0 ? 0 : (int32_t)n);
				break;
			}
			default:
				LOG_DBG("unknown E3 APP_MSG subtype %u", subtype);
				break;
		}
		index += body_len + 2;
	}

	queue_latest_input(requested_report_id);
}

static void handle_host_channel_map(const struct valve_esb_payload *payload)
{
	size_t count;

	if(payload->length < 2)
	{
		return;
	}

	count = MIN(payload->length - 1, ARRAY_SIZE(channel_map));
	for(size_t i = 0; i < count; ++i)
	{
		channel_map[i] = payload->data[i + 1];
	}
	for(size_t i = count; i < ARRAY_SIZE(channel_map); ++i)
	{
		channel_map[i] = ESB_CHANNEL_UNUSED;
	}
	channel_maps_received++;
	if(channel_map[0] != current_channel && channel_map[0] != ESB_CHANNEL_UNUSED)
	{
		pending_map_channel = channel_map[0];
		k_work_submit(&channel_map_work);
	}

	LOG_DBG("ESB channel map: %u %u %u %u", channel_map[0], channel_map[1], channel_map[2],
	        channel_map[3]);
}

static void handle_host_awake(const struct valve_esb_payload *payload)
{
	struct valve_esb_payload status = {
		.length = 2,
		.pipe = 0,
	};

	status.data[0] = ESB_CONTROLLER_STATUS;
	status.data[1] = payload->length >= 3 && payload->data[2] != 0 ? 1 : 0;
	awake_frames_received++;
	queue_payload(&status);
}

static void event_handler(const struct valve_esb_backend_event *event)
{
	switch(event->evt_id)
	{
		case VALVE_ESB_EVENT_TX_SUCCESS:
			replies_sent++;
			break;
		case VALVE_ESB_EVENT_RX_RECEIVED:
			while(valve_esb_backend_read_rx_payload(&rx_payload) == 0)
			{
				if(rx_payload.length == 0)
				{
					continue;
				}
				last_host_rx_ms = k_uptime_get();
				switch(rx_payload.data[0])
				{
					case ESB_HOST_FRAME:
						handle_host_frame(&rx_payload);
						break;
					case ESB_HOST_IDLE:
						break;
					case ESB_HOST_POLL:
						handle_host_poll(&rx_payload);
						break;
					case ESB_HOST_CHANNEL_MAP:
						handle_host_channel_map(&rx_payload);
						break;
					case ESB_HOST_AWAKE:
						handle_host_awake(&rx_payload);
						break;
					default:
						if(unknown_host_opcode_logs_remaining != 0)
						{
							LOG_INF("unknown ESB host opcode 0x%02x length %u", rx_payload.data[0],
							        rx_payload.length);
							unknown_host_opcode_logs_remaining--;
						}
						break;
				}
			}
			break;
		case VALVE_ESB_EVENT_TX_FAILED:
			break;
	}
}

int transport_esb_init(void)
{
	struct valve_esb_backend_config config = VALVE_ESB_BACKEND_DEFAULT_CONFIG;
	int err;

	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		err = settings_load_subtree("esb");
		if(err)
		{
			LOG_WRN("failed to load ESB settings: %d", err);
		}
		else if(pairing_bond_valid)
		{
			LOG_INF("loaded ESB bond %08x/%08x for puck %.*s", sys_get_le32(&pairing_bond[0]),
			        sys_get_le32(&pairing_bond[4]), 16, &pairing_bond[8]);
		}
	}

	config.event_handler = event_handler;
	config.retransmit_delay = 600;
	config.retransmit_count = 0;
	config.payload_length = 128;
	config.selective_auto_ack = true;
	config.use_fast_ramp_up = true;

	err = valve_esb_backend_init(&config);
	if(err)
	{
		return err;
	}
	esb_started = true;
	if((err = valve_esb_backend_set_address_length(5)) != 0)
	{
		return err;
	}

	configure_address(discovery_base, discovery_prefix);
	LOG_INF("ESB listening on discovery address ibex/10, channel 2");
	err = valve_esb_backend_start_rx();
	if(!err)
	{
		k_work_schedule(&channel_scan_work, K_MSEC(ESB_CHANNEL_SCAN_INTERVAL_MS));
	}
	return err;
}

void transport_esb_deactivate(void)
{
	if(!esb_started)
	{
		return;
	}

	esb_started = false;

	/* Cancel pending and in-flight work before touching the backend. */
	if(k_current_get() == k_work_queue_thread_get(&k_sys_work_q))
	{
		k_work_cancel(&channel_map_work);
		k_work_cancel(&session_work);
		k_work_cancel_delayable(&channel_scan_work);
	}
	else
	{
		struct k_work_sync sync;
		k_work_cancel_sync(&channel_map_work, &sync);
		k_work_cancel_sync(&session_work, &sync);
		k_work_cancel_delayable_sync(&channel_scan_work, &sync);
	}

	stop_and_flush();
	valve_esb_backend_disable();
	session_connected = false;
	session_pending = false;
	latest_input_valid = false;
	latest_battery_valid = false;
}

static void build_input_payload(struct valve_esb_payload *payload, uint8_t report_id,
                                uint8_t report_size, uint8_t sequence,
                                const struct controller_report *report)
{
	uint8_t *input_report;

	*payload = (struct valve_esb_payload){
		.length = 3 + report_size,
		.pipe = 0,
	};
	payload->data[0] = ESB_CONTROLLER_INPUT;
	payload->data[1] = report_size;
	payload->data[2] = ESB_REPORT_TLV;

	input_report = &payload->data[3];
	input_report[0] = report_id;
	triton_state_report_pack_body(&input_report[1], report_size - 1, sequence, report,
	                              triton_state_report_timestamp_us());
}

int transport_esb_send(const struct controller_report *report)
{
	struct valve_esb_payload input_42;
	struct valve_esb_payload input_45;
	k_spinlock_key_t key;
	uint8_t sequence = report_sequence++;

	build_input_payload(&input_42, ESB_REPORT_42_ID, ESB_REPORT_42_SIZE, sequence, report);
	build_input_payload(&input_45, ESB_REPORT_45_ID, ESB_REPORT_45_SIZE, sequence, report);

	key = k_spin_lock(&latest_input_lock);
	latest_input_42 = input_42;
	latest_input_45 = input_45;
	latest_input_valid = true;
	k_spin_unlock(&latest_input_lock, key);

	return 0;
}

int transport_esb_send_battery_status(const struct controller_battery_report *report)
{
	struct valve_esb_payload battery_43 = {
		.length = 3 + ESB_REPORT_43_SIZE,
		.pipe = 0,
	};
	uint8_t *input_report = &battery_43.data[3];
	k_spinlock_key_t key;

	if(!report->valid)
	{
		return -EINVAL;
	}

	battery_43.data[0] = ESB_CONTROLLER_INPUT;
	battery_43.data[1] = ESB_REPORT_43_SIZE;
	battery_43.data[2] = ESB_REPORT_TLV;
	input_report[0] = ESB_REPORT_43_ID;
	input_report[1] = report->charge_state;
	input_report[2] = report->level_percent;
	sys_put_le16(report->battery_mv, &input_report[3]);
	sys_put_le16(report->system_mv, &input_report[5]);
	sys_put_le16(report->input_mv, &input_report[7]);
	sys_put_le16(report->current_ma, &input_report[9]);
	sys_put_le16(report->input_current_ma, &input_report[11]);
	sys_put_le16(report->temperature_c, &input_report[13]);

	key = k_spin_lock(&latest_input_lock);
	latest_battery_43 = battery_43;
	latest_battery_valid = true;
	k_spin_unlock(&latest_input_lock, key);

	return 0;
}

bool transport_esb_connected(void)
{
	return session_connected;
}

uint8_t transport_esb_channel(void)
{
	return current_channel;
}

uint32_t transport_esb_host_frames_received(void)
{
	return host_frames_received;
}

uint32_t transport_esb_channel_maps_received(void)
{
	return channel_maps_received;
}

uint32_t transport_esb_awake_frames_received(void)
{
	return awake_frames_received;
}

uint32_t transport_esb_polls_received(void)
{
	return polls_received;
}

uint32_t transport_esb_replies_sent(void)
{
	return replies_sent;
}

uint32_t transport_esb_backend_end_events(void)
{
	return valve_esb_backend_end_events();
}

uint32_t transport_esb_backend_crc_ok_events(void)
{
	return valve_esb_backend_crc_ok_events();
}

uint32_t transport_esb_backend_crc_bad_events(void)
{
	return valve_esb_backend_crc_bad_events();
}

uint32_t transport_esb_backend_rx_dropped_events(void)
{
	return valve_esb_backend_rx_dropped_events();
}

bool transport_esb_bond_loaded(void)
{
	return pairing_bond_valid;
}

int transport_esb_provision_bond(uint32_t proteus_uuid, uint32_t ibex_uuid, const char *serial)
{
	memset(pairing_bond, 0, sizeof(pairing_bond));
	sys_put_le32(proteus_uuid, &pairing_bond[0]);
	sys_put_le32(ibex_uuid, &pairing_bond[4]);
	if(serial != NULL && serial[0] != '\0')
	{
		memcpy(&pairing_bond[ESB_BOND_SERIAL_OFFSET], serial,
		       MIN(strlen(serial), ESB_BOND_SERIAL_STORED_SIZE));
	}
	else
	{
		valve_identity_copy_serial(VALVE_IDENTITY_UNIT_SERIAL,
		                           &pairing_bond[ESB_BOND_SERIAL_OFFSET],
		                           ESB_BOND_SERIAL_STORED_SIZE);
	}

	pairing_bond_valid = true;
	if(IS_ENABLED(CONFIG_SETTINGS))
	{
		int err = settings_save_one("esb/bond", pairing_bond, sizeof(pairing_bond));

		if(err)
		{
			LOG_ERR("failed to provision ESB bond: %d", err);
			return err;
		}
	}

	LOG_INF("provisioned ESB bond %08x/%08x for %.*s", sys_get_le32(&pairing_bond[0]),
	        sys_get_le32(&pairing_bond[4]), 16, &pairing_bond[8]);
	return 0;
}

int transport_esb_get_debug(struct valve_esb_backend_debug *debug)
{
	return valve_esb_backend_get_debug(debug);
}
