/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <esb.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/util.h>

#include "esb_backend.h"

static valve_esb_backend_event_handler backend_event_handler;

static int clocks_start(void)
{
	struct onoff_manager *manager;
	struct onoff_client client;
	int result;
	int err;

	manager = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if(!manager)
	{
		return -ENXIO;
	}

	sys_notify_init_spinwait(&client.notify);
	err = onoff_request(manager, &client);
	if(err < 0)
	{
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&client.notify, &result);
	} while(err == -EAGAIN);

	return err ? err : result;
}

static void forward_event(const struct esb_evt *event)
{
	struct valve_esb_backend_event backend_event;

	if(!backend_event_handler)
	{
		return;
	}

	switch(event->evt_id)
	{
		case ESB_EVENT_TX_SUCCESS:
			backend_event.evt_id = VALVE_ESB_EVENT_TX_SUCCESS;
			break;
		case ESB_EVENT_RX_RECEIVED:
			backend_event.evt_id = VALVE_ESB_EVENT_RX_RECEIVED;
			break;
		case ESB_EVENT_TX_FAILED:
		default:
			backend_event.evt_id = VALVE_ESB_EVENT_TX_FAILED;
			break;
	}
	backend_event_handler(&backend_event);
}

static void to_ncs_payload(struct esb_payload *dst, const struct valve_esb_payload *src)
{
	memset(dst, 0, sizeof(*dst));
	dst->length = src->length;
	dst->pipe = src->pipe;
	dst->rssi = src->rssi;
	dst->noack = src->noack;
	dst->pid = src->pid;
	memcpy(dst->data, src->data, MIN(src->length, (uint8_t)CONFIG_ESB_MAX_PAYLOAD_LENGTH));
}

static void from_ncs_payload(struct valve_esb_payload *dst, const struct esb_payload *src)
{
	memset(dst, 0, sizeof(*dst));
	dst->length = MIN(src->length, (uint8_t)sizeof(dst->data));
	dst->pipe = src->pipe;
	dst->rssi = src->rssi;
	dst->noack = src->noack;
	dst->pid = src->pid;
	memcpy(dst->data, src->data, dst->length);
}

int valve_esb_backend_init(const struct valve_esb_backend_config *config)
{
	struct esb_config ncs_config = ESB_DEFAULT_CONFIG;
	int err;

	err = clocks_start();
	if(err)
	{
		return err;
	}

	backend_event_handler = config->event_handler;
	ncs_config.protocol = ESB_PROTOCOL_ESB_DPL;
	ncs_config.mode = ESB_MODE_PRX;
	ncs_config.event_handler = forward_event;
	ncs_config.bitrate = ESB_BITRATE_2MBPS_BLE;
	ncs_config.crc = ESB_CRC_16BIT;
	ncs_config.retransmit_delay = config->retransmit_delay;
	ncs_config.retransmit_count = config->retransmit_count;
	ncs_config.payload_length = config->payload_length;
	ncs_config.selective_auto_ack = config->selective_auto_ack;
	ncs_config.use_fast_ramp_up = config->use_fast_ramp_up;
	return esb_init(&ncs_config);
}

int valve_esb_backend_set_address_length(uint8_t length)
{
	return esb_set_address_length(length);
}

int valve_esb_backend_set_rf_channel(uint8_t channel)
{
	return esb_set_rf_channel(channel);
}

int valve_esb_backend_set_base_address_0(const uint8_t *base)
{
	return esb_set_base_address_0(base);
}

int valve_esb_backend_set_prefixes(const uint8_t *prefixes, uint8_t count)
{
	return esb_set_prefixes(prefixes, count);
}

int valve_esb_backend_start_rx(void)
{
	return esb_start_rx();
}

int valve_esb_backend_stop_rx(void)
{
	return esb_stop_rx();
}

int valve_esb_backend_flush_rx(void)
{
	return esb_flush_rx();
}

int valve_esb_backend_flush_tx(void)
{
	return esb_flush_tx();
}

int valve_esb_backend_write_payload(const struct valve_esb_payload *payload)
{
	struct esb_payload ncs_payload;

	to_ncs_payload(&ncs_payload, payload);
	return esb_write_payload(&ncs_payload);
}

int valve_esb_backend_read_rx_payload(struct valve_esb_payload *payload)
{
	struct esb_payload ncs_payload;
	int err;

	err = esb_read_rx_payload(&ncs_payload);
	if(err)
	{
		return err;
	}

	from_ncs_payload(payload, &ncs_payload);
	return 0;
}

void valve_esb_backend_disable(void)
{
	esb_disable();
	backend_event_handler = NULL;
}

int valve_esb_backend_get_debug(struct valve_esb_backend_debug *debug)
{
	ARG_UNUSED(debug);

	return -ENOTSUP;
}

uint32_t valve_esb_backend_end_events(void)
{
	return 0;
}

uint32_t valve_esb_backend_crc_ok_events(void)
{
	return 0;
}

uint32_t valve_esb_backend_crc_bad_events(void)
{
	return 0;
}

uint32_t valve_esb_backend_rx_dropped_events(void)
{
	return 0;
}
