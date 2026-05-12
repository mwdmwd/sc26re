/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <hal/nrf_radio.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "esb_backend.h"

#define RX_FIFO_SIZE 8
#define TX_FIFO_SIZE 2
#define NRF_RADIO_BASE_FREQUENCY_MHZ 2400
#define RADIO_DISABLED_TIMEOUT_US 3000

static valve_esb_backend_event_handler backend_event_handler;
static uint8_t rx_dma[VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE + 2];
static uint8_t tx_dma[VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE + 2];
static struct valve_esb_payload rx_fifo[RX_FIFO_SIZE];
static struct valve_esb_payload tx_fifo[TX_FIFO_SIZE];
static uint8_t rx_head;
static uint8_t rx_tail;
static uint8_t rx_count;
static uint8_t tx_head;
static uint8_t tx_tail;
static uint8_t tx_count;
static uint8_t current_channel;
static uint8_t current_base[4] = { 'i', 'b', 'e', 'x' };
static uint8_t current_prefix = 0x10;
static uint8_t last_rx_s1;
static bool initialized;
static bool rx_running;
static bool tx_active;
static bool ack_payload_active;
static bool rx_duplicate_valid;
static uint8_t rx_duplicate_pid;
static uint32_t rx_duplicate_crc;
static struct k_spinlock fifo_lock;
static volatile uint32_t end_events;
static volatile uint32_t crc_ok_events;
static volatile uint32_t crc_bad_events;
static volatile uint32_t rx_dropped_events;

static uint8_t bitrev8(uint8_t value)
{
	value = ((value & 0xf0) >> 4) | ((value & 0x0f) << 4);
	value = ((value & 0xcc) >> 2) | ((value & 0x33) << 2);
	value = ((value & 0xaa) >> 1) | ((value & 0x55) << 1);
	return value;
}

static int wait_disabled(void)
{
	uint32_t start = k_cycle_get_32();
	uint32_t timeout = k_us_to_cyc_ceil32(RADIO_DISABLED_TIMEOUT_US);

	while(!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED))
	{
		if(k_cycle_get_32() - start > timeout)
		{
			return -ETIMEDOUT;
		}
	}

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	return 0;
}

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

static void emit_event(enum valve_esb_backend_event_id event_id)
{
	struct valve_esb_backend_event event = {
		.evt_id = event_id,
	};

	if(backend_event_handler)
	{
		backend_event_handler(&event);
	}
}

static void apply_address(void)
{
	uint8_t base[4];
	uint8_t prefix;

	for(size_t i = 0; i < ARRAY_SIZE(base); ++i)
	{
		base[i] = bitrev8(current_base[i]);
	}
	prefix = bitrev8(current_prefix);

	nrf_radio_base0_set(
	    NRF_RADIO,
	    ((uint32_t)base[0] << 24) | ((uint32_t)base[1] << 16) | ((uint32_t)base[2] << 8) | base[3]);
	nrf_radio_prefix0_set(NRF_RADIO, prefix);
	nrf_radio_txaddress_set(NRF_RADIO, 0);
	nrf_radio_rxaddresses_set(NRF_RADIO, BIT(0));
}

static void configure_radio(void)
{
	nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_BLE_2MBIT);
	nrf_radio_frequency_set(NRF_RADIO, NRF_RADIO_BASE_FREQUENCY_MHZ + current_channel);
	nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_0DBM);
#if defined(RADIO_MODECNF0_RU_Fast)
	nrf_radio_fast_ramp_up_enable_set(NRF_RADIO, true);
#endif

	nrf_radio_packet_conf_t packet_conf = {
		.lflen = 8,
		.s0len = 0,
		.s1len = 3,
		.s1incl = false,
		.cilen = 0,
		.plen = NRF_RADIO_PREAMBLE_LENGTH_8BIT,
		.maxlen = VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE,
		.statlen = 0,
		.balen = 4,
		.big_endian = true,
		.whiteen = false,
	};

	nrf_radio_packet_configure(NRF_RADIO, &packet_conf);
	nrf_radio_crc_configure(NRF_RADIO, 2, NRF_RADIO_CRC_ADDR_INCLUDE, 0x11021);
	nrf_radio_crcinit_set(NRF_RADIO, 0xffff);
	apply_address();
}

static void prepare_rx(void)
{
	memset(rx_dma, 0, sizeof(rx_dma));
	nrf_radio_packetptr_set(NRF_RADIO, rx_dma);
	nrf_radio_shorts_set(NRF_RADIO,
	                     NRF_RADIO_SHORT_READY_START_MASK | NRF_RADIO_SHORT_END_DISABLE_MASK);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);
}

static void restart_rx(void)
{
	(void)wait_disabled();
	prepare_rx();
}

static bool peek_tx_payload(struct valve_esb_payload *payload)
{
	k_spinlock_key_t key = k_spin_lock(&fifo_lock);

	if(tx_count == 0)
	{
		k_spin_unlock(&fifo_lock, key);
		return false;
	}

	*payload = tx_fifo[tx_tail];
	k_spin_unlock(&fifo_lock, key);
	return true;
}

static void discard_tx_payload(void)
{
	k_spinlock_key_t key = k_spin_lock(&fifo_lock);

	if(tx_count == 0)
	{
		k_spin_unlock(&fifo_lock, key);
		return;
	}

	tx_tail = (tx_tail + 1) % TX_FIFO_SIZE;
	tx_count--;
	k_spin_unlock(&fifo_lock, key);
}

static void start_tx_payload(const struct valve_esb_payload *payload)
{
	memset(tx_dma, 0, sizeof(tx_dma));
	tx_dma[0] = payload->length;
	tx_dma[1] = last_rx_s1 & 0x07;
	memcpy(&tx_dma[2], payload->data, MIN(payload->length, (uint8_t)sizeof(payload->data)));

	tx_active = true;
	nrf_radio_packetptr_set(NRF_RADIO, tx_dma);
	nrf_radio_shorts_set(NRF_RADIO,
	                     NRF_RADIO_SHORT_READY_START_MASK | NRF_RADIO_SHORT_END_DISABLE_MASK);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
}

static void radio_isr(const void *arg)
{
	struct valve_esb_payload ack_payload = { 0 };
	uint8_t rx_s1;
	uint8_t rx_pid;
	uint32_t rx_crc;
	bool ack_requested;
	bool retransmit_payload;
	bool rx_received = false;
	bool ack_started = false;

	ARG_UNUSED(arg);

	if(!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_END))
	{
		return;
	}

	nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
	end_events++;
	if(tx_active)
	{
		tx_active = false;
		emit_event(VALVE_ESB_EVENT_TX_SUCCESS);
		if(rx_running)
		{
			restart_rx();
		}
		return;
	}

	if((nrf_radio_crc_status_check(NRF_RADIO) != 0) &&
	   rx_dma[0] <= VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE)
	{
		k_spinlock_key_t key;

		crc_ok_events++;
		rx_s1 = rx_dma[1] & 0x07;
		rx_pid = (rx_s1 >> 1) & 0x03;
		rx_crc = nrf_radio_rxcrc_get(NRF_RADIO);
		ack_requested = (rx_s1 & BIT(0)) != 0;
		retransmit_payload =
		    rx_duplicate_valid && rx_duplicate_pid == rx_pid && rx_duplicate_crc == rx_crc;
		last_rx_s1 = rx_s1;
		rx_duplicate_valid = true;
		rx_duplicate_pid = rx_pid;
		rx_duplicate_crc = rx_crc;

		if(ack_requested)
		{
			if(ack_payload_active && !retransmit_payload)
			{
				discard_tx_payload();
			}
			ack_payload_active = peek_tx_payload(&ack_payload);
			if(wait_disabled() == 0)
			{
				start_tx_payload(&ack_payload);
				ack_started = true;
			}
		}

		if(!retransmit_payload)
		{
			key = k_spin_lock(&fifo_lock);
			if(rx_count < RX_FIFO_SIZE)
			{
				struct valve_esb_payload *payload = &rx_fifo[rx_head];

				memset(payload, 0, sizeof(*payload));
				payload->length = rx_dma[0];
				payload->pipe = 0;
				payload->noack = !ack_requested;
				payload->pid = rx_pid;
				memcpy(payload->data, &rx_dma[2], payload->length);
				rx_head = (rx_head + 1) % RX_FIFO_SIZE;
				rx_count++;
				rx_received = true;
			}
			else
			{
				rx_dropped_events++;
			}
			k_spin_unlock(&fifo_lock, key);
		}
	}
	else
	{
		crc_bad_events++;
	}

	if(rx_running && !ack_started)
	{
		restart_rx();
	}

	if(rx_received)
	{
		emit_event(VALVE_ESB_EVENT_RX_RECEIVED);
	}
}

int valve_esb_backend_init(const struct valve_esb_backend_config *config)
{
	int err = clocks_start();

	if(err)
	{
		return err;
	}

	backend_event_handler = config->event_handler;
	IRQ_CONNECT(RADIO_IRQn, 1, radio_isr, NULL, 0);
	irq_enable(RADIO_IRQn);
	nrf_radio_int_enable(NRF_RADIO, NRF_RADIO_INT_END_MASK);
	configure_radio();
	initialized = true;
	return 0;
}

int valve_esb_backend_set_address_length(uint8_t length)
{
	return length == 5 ? 0 : -ENOTSUP;
}

int valve_esb_backend_set_rf_channel(uint8_t channel)
{
	current_channel = channel;
	if(initialized)
	{
		nrf_radio_frequency_set(NRF_RADIO, NRF_RADIO_BASE_FREQUENCY_MHZ + current_channel);
	}
	return 0;
}

int valve_esb_backend_set_base_address_0(const uint8_t *base)
{
	memcpy(current_base, base, sizeof(current_base));
	if(initialized)
	{
		apply_address();
	}
	return 0;
}

int valve_esb_backend_set_prefixes(const uint8_t *prefixes, uint8_t count)
{
	if(count == 0)
	{
		return -EINVAL;
	}

	current_prefix = prefixes[0];
	if(initialized)
	{
		apply_address();
	}
	return 0;
}

int valve_esb_backend_start_rx(void)
{
	if(!initialized)
	{
		return -EACCES;
	}

	rx_running = true;
	prepare_rx();
	return 0;
}

int valve_esb_backend_stop_rx(void)
{
	if(!initialized)
	{
		return 0;
	}

	rx_running = false;
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	return wait_disabled();
}

int valve_esb_backend_flush_rx(void)
{
	k_spinlock_key_t key = k_spin_lock(&fifo_lock);

	rx_head = 0;
	rx_tail = 0;
	rx_count = 0;
	rx_duplicate_valid = false;
	k_spin_unlock(&fifo_lock, key);
	return 0;
}

int valve_esb_backend_flush_tx(void)
{
	k_spinlock_key_t key = k_spin_lock(&fifo_lock);

	tx_head = 0;
	tx_tail = 0;
	tx_count = 0;
	ack_payload_active = false;
	k_spin_unlock(&fifo_lock, key);
	return 0;
}

int valve_esb_backend_write_payload(const struct valve_esb_payload *payload)
{
	k_spinlock_key_t key;

	if(!initialized)
	{
		return -EACCES;
	}
	if(!payload || payload->length == 0 || payload->length > VALVE_ESB_BACKEND_MAX_PAYLOAD_SIZE)
	{
		return -EINVAL;
	}

	key = k_spin_lock(&fifo_lock);
	if(tx_count >= TX_FIFO_SIZE)
	{
		k_spin_unlock(&fifo_lock, key);
		return -ENOMEM;
	}

	tx_fifo[tx_head] = *payload;
	tx_head = (tx_head + 1) % TX_FIFO_SIZE;
	tx_count++;
	k_spin_unlock(&fifo_lock, key);
	return 0;
}

int valve_esb_backend_read_rx_payload(struct valve_esb_payload *payload)
{
	k_spinlock_key_t key;

	if(!payload)
	{
		return -EINVAL;
	}

	key = k_spin_lock(&fifo_lock);
	if(rx_count == 0)
	{
		k_spin_unlock(&fifo_lock, key);
		return -ENOMSG;
	}

	*payload = rx_fifo[rx_tail];
	rx_tail = (rx_tail + 1) % RX_FIFO_SIZE;
	rx_count--;
	k_spin_unlock(&fifo_lock, key);
	return 0;
}

void valve_esb_backend_disable(void)
{
	if(!initialized)
	{
		return;
	}

	irq_disable(RADIO_IRQn);
	nrf_radio_int_disable(NRF_RADIO, NRF_RADIO_INT_END_MASK);
	rx_running = false;
	tx_active = false;
	nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
	(void)wait_disabled();
	initialized = false;
	backend_event_handler = NULL;
}

int valve_esb_backend_get_debug(struct valve_esb_backend_debug *debug)
{
	if(!debug)
	{
		return -EINVAL;
	}

	*debug = (struct valve_esb_backend_debug){
		.channel = current_channel,
		.prefix = current_prefix,
		.rx_running = rx_running,
		.tx_active = tx_active,
		.mode = NRF_RADIO->MODE,
		.frequency = NRF_RADIO->FREQUENCY,
		.pcnf0 = NRF_RADIO->PCNF0,
		.pcnf1 = NRF_RADIO->PCNF1,
		.base0 = NRF_RADIO->BASE0,
		.prefix0 = NRF_RADIO->PREFIX0,
		.rxaddresses = NRF_RADIO->RXADDRESSES,
		.txaddress = NRF_RADIO->TXADDRESS,
		.crccnf = NRF_RADIO->CRCCNF,
		.crcpoly = NRF_RADIO->CRCPOLY,
		.crcinit = NRF_RADIO->CRCINIT,
		.crcstatus = NRF_RADIO->CRCSTATUS,
		.shorts = NRF_RADIO->SHORTS,
		.intenset = NRF_RADIO->INTENSET,
		.state = NRF_RADIO->STATE,
		.packetptr = NRF_RADIO->PACKETPTR,
		.events_ready = NRF_RADIO->EVENTS_READY,
		.events_address = NRF_RADIO->EVENTS_ADDRESS,
		.events_end = NRF_RADIO->EVENTS_END,
		.events_disabled = NRF_RADIO->EVENTS_DISABLED,
	};
	memcpy(debug->base, current_base, sizeof(debug->base));
	return 0;
}

uint32_t valve_esb_backend_end_events(void)
{
	return end_events;
}

uint32_t valve_esb_backend_crc_ok_events(void)
{
	return crc_ok_events;
}

uint32_t valve_esb_backend_crc_bad_events(void)
{
	return crc_bad_events;
}

uint32_t valve_esb_backend_rx_dropped_events(void)
{
	return rx_dropped_events;
}
