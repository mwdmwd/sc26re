/* SPDX-License-Identifier: AGPL-3.0-or-later */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include "controller.h"
#include "puck_interface.h"
#include "triton_state_report.h"
#include "valve_feature.h"
#include "valve_identity.h"

LOG_MODULE_REGISTER(puck_interface);

#if CONFIG_IBEX_PUCK_INTERFACE && DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay)

#define PUCK_UART_NODE DT_NODELABEL(uart0)
#define PUCK_FRAME_SIZE 73
#define PUCK_FRAME_PAYLOAD_OFFSET 8
#define PUCK_FRAME_CRC_OFFSET 71
#define PUCK_RESPONSE_CRC_SIZE 71
#define PUCK_RESPONSE_MARKER 0xac
#define PUCK_FEATURE_BODY_SIZE 63
#define PUCK_IDENTITY_PAYLOAD_SIZE 57
#define PUCK_STATUS_PAYLOAD_SIZE 53
#define PUCK_RX_BUFFER_BYTES 256
#define PUCK_FRAME_IDLE_TIMEOUT_MS 1000
#define PUCK_IDLE_TIMEOUTS_BEFORE_INACTIVE 2
#define PUCK_THREAD_STACK_SIZE 2048
#define PUCK_THREAD_PRIORITY 9
#define PUCK_OFW_MAGIC 0x69fe17ffu

struct puck_frame_reader
{
	uint8_t window[PUCK_FRAME_SIZE];
	size_t used;
};

enum puck_request_type
{
	PUCK_REQUEST_IDENTITY = 0,
	PUCK_REQUEST_FEATURE_READ = 1,
	PUCK_REQUEST_FEATURE_WRITE = 2,
	PUCK_REQUEST_STATUS = 3,
};

static const struct device *const puck_uart = DEVICE_DT_GET(PUCK_UART_NODE);
static atomic_t puck_initialized;
static atomic_t puck_active;
static atomic_t puck_frame_seen;
static atomic_t puck_rx_overflow;
static uint8_t puck_status_sequence;
static uint8_t puck_feature_request[PUCK_FEATURE_BODY_SIZE];
static size_t puck_feature_request_len;
static uint8_t puck_last_tx_frame[PUCK_FRAME_SIZE];
static bool puck_last_tx_frame_valid;

RING_BUF_DECLARE(puck_rx_ringbuf, PUCK_RX_BUFFER_BYTES);
K_SEM_DEFINE(puck_rx_ready, 0, 1);
K_THREAD_STACK_DEFINE(puck_thread_stack, PUCK_THREAD_STACK_SIZE);
static struct k_thread puck_thread;

static uint16_t puck_crc16(uint16_t crc, const uint8_t *data, size_t len)
{
	for(size_t i = 0; i < len; ++i)
	{
		uint8_t folded = (uint8_t)(data[i] ^ crc);

		folded ^= (uint8_t)(folded << 4);
		crc = (uint16_t)(((uint16_t)folded << 8) | ((crc >> 8) ^ (folded >> 4)));
		crc ^= (uint16_t)folded << 3;
	}

	return crc;
}

static void puck_uart_isr(const struct device *dev, void *user_data)
{
	uint8_t data[16];
	bool received = false;
	int len;

	uart_irq_update(dev);
	ARG_UNUSED(user_data);

	if(!uart_irq_rx_ready(dev))
	{
		return;
	}

	do
	{
		uint32_t stored;

		len = uart_fifo_read(dev, data, sizeof(data));
		if(len <= 0)
		{
			break;
		}

		received = true;
		stored = ring_buf_put(&puck_rx_ringbuf, data, (uint32_t)len);
		if(stored != (uint32_t)len)
		{
			atomic_set(&puck_rx_overflow, 1);
		}
	} while(len == (int)sizeof(data));

	if(received)
	{
		atomic_set(&puck_active, 1);
		k_sem_give(&puck_rx_ready);
	}
}

static void puck_fill_identity(uint8_t *payload)
{
	memset(payload, 0, PUCK_IDENTITY_PAYLOAD_SIZE);
	sys_put_le32(PUCK_OFW_MAGIC, &payload[4]);
	sys_put_le32(CONFIG_IBEX_BUILD_TIMESTAMP, &payload[12]);
	valve_identity_copy_serial(VALVE_IDENTITY_UNIT_SERIAL, &payload[16], 14);
	valve_identity_copy_serial(VALVE_IDENTITY_BOARD_SERIAL, &payload[30], 14);
	memcpy(&payload[44], CONFIG_IBEX_FIRMWARE_IDENTITY,
	       MIN(strlen(CONFIG_IBEX_FIRMWARE_IDENTITY), PUCK_IDENTITY_PAYLOAD_SIZE - 44));
}

static void puck_fill_status(uint8_t *payload)
{
	struct controller_report report;

	hardware_read_report(&report);
	triton_state_report_pack_body(payload, PUCK_STATUS_PAYLOAD_SIZE, puck_status_sequence++,
	                              &report, triton_state_report_timestamp_us());
}

static int puck_feature_body_len(const uint8_t *body, size_t *body_len)
{
	size_t len = (size_t)body[1] + 2U;

	if(len > PUCK_FEATURE_BODY_SIZE)
	{
		return -EINVAL;
	}

	*body_len = len;
	return 0;
}

static int32_t puck_handle_feature_write(const uint8_t *payload)
{
	size_t body_len;
	int err;

	err = puck_feature_body_len(payload, &body_len);
	if(err)
	{
		return err;
	}

	puck_feature_request_len = 0;
	memset(puck_feature_request, 0, sizeof(puck_feature_request));

	err = valve_feature_handle_request(VALVE_FEATURE_LINK_PUCK, payload, body_len);
	if(err)
	{
		return err;
	}

	memcpy(puck_feature_request, payload, body_len);
	puck_feature_request_len = body_len;
	return 0;
}

static int32_t puck_handle_feature_read(uint8_t *payload)
{
	ssize_t response_len;

	if(puck_feature_request_len == 0)
	{
		return -EINVAL;
	}

	response_len =
	    valve_feature_prepare_response(VALVE_FEATURE_LINK_PUCK, puck_feature_request,
	                                   puck_feature_request_len, payload, PUCK_FEATURE_BODY_SIZE);
	if(response_len < 0)
	{
		return (int32_t)response_len;
	}

	return PUCK_FEATURE_BODY_SIZE;
}

static void puck_write_frame(const uint8_t frame[static PUCK_FRAME_SIZE])
{
	memcpy(puck_last_tx_frame, frame, PUCK_FRAME_SIZE);
	puck_last_tx_frame_valid = true;

	for(size_t i = 0; i < PUCK_FRAME_SIZE; ++i)
	{
		uart_poll_out(puck_uart, frame[i]);
	}
}

static void puck_write_response(const uint8_t *request)
{
	uint8_t response[PUCK_FRAME_SIZE] = { 0 };
	uint16_t crc;
	int32_t status = 0;

	response[0] = PUCK_RESPONSE_MARKER;
	response[1] = request[1];

	switch(request[1])
	{
		case PUCK_REQUEST_IDENTITY:
			puck_fill_identity(&response[PUCK_FRAME_PAYLOAD_OFFSET]);
			break;
		case PUCK_REQUEST_STATUS:
			puck_fill_status(&response[PUCK_FRAME_PAYLOAD_OFFSET]);
			break;
		case PUCK_REQUEST_FEATURE_READ:
			status = puck_handle_feature_read(&response[PUCK_FRAME_PAYLOAD_OFFSET]);
			break;
		case PUCK_REQUEST_FEATURE_WRITE:
			status = puck_handle_feature_write(&request[PUCK_FRAME_PAYLOAD_OFFSET]);
			break;
		default:
			status = -EINVAL;
			break;
	}

	sys_put_le32((uint32_t)status, &response[4]);
	crc = puck_crc16(0, response, PUCK_RESPONSE_CRC_SIZE);
	sys_put_le16(crc, &response[PUCK_FRAME_CRC_OFFSET]);

	puck_write_frame(response);
}

static void puck_frame_reader_reset(struct puck_frame_reader *reader)
{
	reader->used = 0;
}

static bool puck_frame_reader_push(struct puck_frame_reader *reader, uint8_t byte,
                                   uint8_t frame[static PUCK_FRAME_SIZE])
{
	if(reader->used == PUCK_FRAME_SIZE)
	{
		memmove(reader->window, &reader->window[1], PUCK_FRAME_SIZE - 1);
		reader->used--;
	}

	reader->window[reader->used++] = byte;
	if(reader->used != PUCK_FRAME_SIZE || puck_crc16(0, reader->window, PUCK_FRAME_SIZE) != 0)
	{
		return false;
	}

	if(puck_last_tx_frame_valid && memcmp(reader->window, puck_last_tx_frame, PUCK_FRAME_SIZE) == 0)
	{
		puck_last_tx_frame_valid = false;
		puck_frame_reader_reset(reader);
		return false;
	}

	memcpy(frame, reader->window, PUCK_FRAME_SIZE);
	puck_frame_reader_reset(reader);
	return true;
}

static int puck_read_frame(struct puck_frame_reader *reader, uint8_t frame[static PUCK_FRAME_SIZE],
                           k_timeout_t timeout)
{
	for(;;)
	{
		uint8_t byte;

		if(atomic_cas(&puck_rx_overflow, 1, 0))
		{
			puck_frame_reader_reset(reader);
			LOG_WRN("puck UART RX buffer overflow; resynchronizing");
		}

		while(ring_buf_get(&puck_rx_ringbuf, &byte, 1) != 0)
		{
			if(puck_frame_reader_push(reader, byte, frame))
			{
				atomic_set(&puck_frame_seen, 1);
				return 0;
			}
		}

		if(k_sem_take(&puck_rx_ready, timeout) != 0)
		{
			return -ETIMEDOUT;
		}
	}
}

static void puck_thread_entry(void *p1, void *p2, void *p3)
{
	struct puck_frame_reader reader = { 0 };
	uint8_t frame[PUCK_FRAME_SIZE];
	const k_timeout_t timeout =
	    K_MSEC(PUCK_FRAME_IDLE_TIMEOUT_MS * PUCK_IDLE_TIMEOUTS_BEFORE_INACTIVE);

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for(;;)
	{
		int err = puck_read_frame(&reader, frame, timeout);

		if(err == -ETIMEDOUT)
		{
			atomic_clear(&puck_active);
			puck_last_tx_frame_valid = false;
			puck_frame_reader_reset(&reader);
			continue;
		}
		if(err)
		{
			LOG_WRN("puck frame read failed: %d", err);
			continue;
		}

		atomic_set(&puck_active, 1);
		puck_write_response(frame);
	}
}

int puck_interface_init(void)
{
	int err;

	if(!atomic_cas(&puck_initialized, 0, 1))
	{
		return 0;
	}

	if(!device_is_ready(puck_uart))
	{
		atomic_clear(&puck_initialized);
		return -ENODEV;
	}

	err = uart_irq_callback_user_data_set(puck_uart, puck_uart_isr, NULL);
	if(err)
	{
		atomic_clear(&puck_initialized);
		return err;
	}
	uart_irq_rx_enable(puck_uart);

	k_thread_create(&puck_thread, puck_thread_stack, K_THREAD_STACK_SIZEOF(puck_thread_stack),
	                puck_thread_entry, NULL, NULL, NULL, PUCK_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&puck_thread, "puck_uart");

	return 0;
}

bool puck_interface_active(void)
{
	return atomic_get(&puck_active) != 0 && atomic_get(&puck_frame_seen) != 0;
}

#else

int puck_interface_init(void)
{
	return 0;
}

bool puck_interface_active(void)
{
	return false;
}

#endif
