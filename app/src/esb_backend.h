/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE 128

enum valve_esb_backend_event_id
{
	VALVE_ESB_EVENT_TX_SUCCESS,
	VALVE_ESB_EVENT_RX_RECEIVED,
	VALVE_ESB_EVENT_TX_FAILED,
};

struct valve_esb_backend_event
{
	enum valve_esb_backend_event_id evt_id;
};

typedef void (*valve_esb_backend_event_handler)(const struct valve_esb_backend_event *event);

struct valve_esb_payload
{
	uint8_t length;
	uint8_t pipe;
	int8_t rssi;
	uint8_t noack;
	uint8_t pid;
	uint8_t data[VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE];
};

struct valve_esb_backend_config
{
	valve_esb_backend_event_handler event_handler;
	uint16_t retransmit_delay;
	uint16_t retransmit_count;
	uint8_t payload_length;
	bool selective_auto_ack;
	bool use_fast_ramp_up;
};

struct valve_esb_backend_debug
{
	uint8_t channel;
	uint8_t prefix;
	uint8_t base[4];
	bool rx_running;
	bool tx_active;
	uint32_t mode;
	uint32_t frequency;
	uint32_t pcnf0;
	uint32_t pcnf1;
	uint32_t base0;
	uint32_t prefix0;
	uint32_t rxaddresses;
	uint32_t txaddress;
	uint32_t crccnf;
	uint32_t crcpoly;
	uint32_t crcinit;
	uint32_t crcstatus;
	uint32_t shorts;
	uint32_t intenset;
	uint32_t state;
	uint32_t packetptr;
	uint32_t events_ready;
	uint32_t events_address;
	uint32_t events_end;
	uint32_t events_disabled;
};

#define VALVE_ESB_BACKEND_DEFAULT_CONFIG \
	{ \
		.payload_length = VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE, \
		.selective_auto_ack = true, .use_fast_ramp_up = true, \
	}

int valve_esb_backend_init(const struct valve_esb_backend_config *config);
int valve_esb_backend_set_address_length(uint8_t length);
int valve_esb_backend_set_rf_channel(uint8_t channel);
int valve_esb_backend_set_base_address_0(const uint8_t *base);
int valve_esb_backend_set_prefixes(const uint8_t *prefixes, uint8_t count);
int valve_esb_backend_start_rx(void);
int valve_esb_backend_stop_rx(void);
int valve_esb_backend_flush_rx(void);
int valve_esb_backend_flush_tx(void);
int valve_esb_backend_write_payload(const struct valve_esb_payload *payload);
int valve_esb_backend_read_rx_payload(struct valve_esb_payload *payload);
void valve_esb_backend_disable(void);
int valve_esb_backend_get_debug(struct valve_esb_backend_debug *debug);
uint32_t valve_esb_backend_end_events(void);
uint32_t valve_esb_backend_crc_ok_events(void);
uint32_t valve_esb_backend_crc_bad_events(void);
uint32_t valve_esb_backend_rx_dropped_events(void);
