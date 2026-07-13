/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/usb/usb_device.h>

#include <hal/nrf_power.h>

#include "controller.h"
#include "haptics.h"
#include "lizard.h"
#include "power.h"
#include "sdl/controller_structs.h"
#include "triton_state_report.h"
#include "valve_feature.h"
#include "valve_hid_report_map.h"

#if CONFIG_IBEX_RGBW_LED
#include "rgbw_led.h"
#endif

LOG_MODULE_REGISTER(transport_usb);

#define VALVE_INPUT_42_SIZE 53
#define VALVE_INPUT_43_SIZE 14
#define HID_REPORT_TYPE_OUTPUT 2
#define HID_REPORT_TYPE_FEATURE 3
#define USB_UNPLUG_POWEROFF_DELAY_MS 250
#define USB_INPUT_REPORT_MAX_SIZE VALVE_FEATURE_REPORT_SIZE
#define USB_INPUT_QUEUE_DEPTH 32

struct usb_input_queue_entry
{
	uint8_t len;
	uint8_t data[USB_INPUT_REPORT_MAX_SIZE];
};

static const struct device *hid_dev;
static uint8_t input_sequence;
static uint8_t feature_response[VALVE_FEATURE_REPORT_SIZE];
static uint8_t output_report[VALVE_FEATURE_REPORT_SIZE];
static struct usb_input_queue_entry input_queue[USB_INPUT_QUEUE_DEPTH];
static struct k_spinlock input_queue_lock;
static uint8_t input_queue_head;
static uint8_t input_queue_count;
static bool input_transfer_active;
static uint8_t input_transfer_report[USB_INPUT_REPORT_MAX_SIZE];
static uint8_t input_transfer_len;
static atomic_t usb_initialized;
static atomic_t usb_attached;
static atomic_t usb_configured;
static atomic_t usb_suspended;
static atomic_t usb_radio_off_mode;

static void usb_unplug_poweroff_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(usb_unplug_poweroff_work, usb_unplug_poweroff_work_handler);

static bool usb_vbus_present(void)
{
#if NRF_POWER_HAS_USBREG
	return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
#else
	return false;
#endif
}

bool transport_usb_attached(void)
{
	return usb_vbus_present() || atomic_get(&usb_attached) != 0 || atomic_get(&usb_configured) != 0;
}

bool transport_usb_configured(void)
{
	return atomic_get(&usb_configured) != 0;
}

static bool usb_ready(void)
{
	return atomic_get(&usb_configured) != 0 && atomic_get(&usb_suspended) == 0;
}

static void usb_input_queue_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&input_queue_lock);

	input_queue_head = 0;
	input_queue_count = 0;
	input_transfer_active = false;
	k_spin_unlock(&input_queue_lock, key);
}

static int usb_submit_next_input(void)
{
	struct usb_input_queue_entry *entry;
	k_spinlock_key_t key;
	int err;

	if(!usb_ready())
	{
		return -ENOTCONN;
	}

	key = k_spin_lock(&input_queue_lock);
	if(input_transfer_active || input_queue_count == 0)
	{
		k_spin_unlock(&input_queue_lock, key);
		return 0;
	}
	entry = &input_queue[input_queue_head];
	input_transfer_len = entry->len;
	memcpy(input_transfer_report, entry->data, entry->len);
	input_transfer_active = true;
	k_spin_unlock(&input_queue_lock, key);

	err = hid_int_ep_write(hid_dev, input_transfer_report, input_transfer_len, NULL);
	if(err)
	{
		key = k_spin_lock(&input_queue_lock);
		if(input_transfer_active)
		{
			input_transfer_active = false;
			if(input_queue_count != 0)
			{
				input_queue_head = (input_queue_head + 1U) % ARRAY_SIZE(input_queue);
				input_queue_count--;
			}
		}
		k_spin_unlock(&input_queue_lock, key);
	}
	return err;
}

static int usb_queue_input_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	struct usb_input_queue_entry *entry;
	k_spinlock_key_t key;
	uint8_t tail;

	if(!usb_ready())
	{
		return -ENOTCONN;
	}
	if(data == NULL || len + 1U > USB_INPUT_REPORT_MAX_SIZE)
	{
		return -EINVAL;
	}

	key = k_spin_lock(&input_queue_lock);
	if(input_queue_count == ARRAY_SIZE(input_queue))
	{
		k_spin_unlock(&input_queue_lock, key);
		return -ENOMEM;
	}
	tail = (input_queue_head + input_queue_count) % ARRAY_SIZE(input_queue);
	entry = &input_queue[tail];
	entry->len = len + 1U;
	entry->data[0] = report_id;
	memcpy(&entry->data[1], data, len);
	input_queue_count++;
	k_spin_unlock(&input_queue_lock, key);

	return usb_submit_next_input();
}

static void usb_cancel_unplug_poweroff(void)
{
	k_work_cancel_delayable(&usb_unplug_poweroff_work);
}

static void usb_schedule_unplug_poweroff(void)
{
	k_work_reschedule(&usb_unplug_poweroff_work, K_MSEC(USB_UNPLUG_POWEROFF_DELAY_MS));
}

static void usb_enter_mode(void)
{
	usb_cancel_unplug_poweroff();
	transport_enter_usb_mode();

#if CONFIG_IBEX_RGBW_LED
	if(transport_usb_configured())
	{
		rgbw_led_set((struct rgbw_color){ 0, 255, 0, 0 });
	}
#endif
}

static void usb_unplug_poweroff_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if(transport_usb_attached())
	{
		return;
	}

	LOG_INF("USB unplugged; powering off");
	power_off();
}

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	bool was_attached = transport_usb_attached();

	ARG_UNUSED(param);

	switch(status)
	{
		case USB_DC_CONNECTED:
			atomic_set(&usb_attached, 1);
			break;
		case USB_DC_CONFIGURED:
			atomic_set(&usb_attached, 1);
			atomic_set(&usb_configured, 1);
			atomic_clear(&usb_suspended);
			atomic_set(&usb_radio_off_mode, 1);
			usb_input_queue_reset();
			lizard_transport_reset();
			usb_enter_mode();
			break;
		case USB_DC_DISCONNECTED:
			atomic_clear(&usb_attached);
			atomic_clear(&usb_configured);
			atomic_clear(&usb_suspended);
			usb_input_queue_reset();
			if(was_attached &&
			   atomic_cas(&usb_radio_off_mode, 1, 0) &&
			   !transport_usb_radio_allowed())
			{
				usb_schedule_unplug_poweroff();
			}
			break;
		case USB_DC_RESET:
			atomic_clear(&usb_configured);
			atomic_clear(&usb_suspended);
			usb_input_queue_reset();
			break;
		case USB_DC_SUSPEND:
			atomic_set(&usb_suspended, 1);
			break;
		case USB_DC_RESUME:
			atomic_set(&usb_attached, 1);
			atomic_clear(&usb_suspended);
			break;
		default:
			break;
	}
}

static void prepare_feature_response(const uint8_t *request, size_t len)
{
	ssize_t inner_len;

	/*
	 * USB carries a report-ID byte before the inner response, so the
	 * inner buffer capacity is one byte less than the full report.
	 */
	inner_len = valve_feature_respond(VALVE_FEATURE_LINK_USB, request, len, &feature_response[1],
	                                  sizeof(feature_response) - 1);
	if(inner_len < 0)
	{
		memset(feature_response, 0, sizeof(feature_response));
		return;
	}
	feature_response[0] = VALVE_FEATURE_REPORT_ID;
}

static void handle_output_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	int err = haptics_handle_output_report(report_id, data, len);

	if(err && err != -ENOTSUP && err != -ENODEV && err != -EBUSY)
	{
		LOG_WRN("USB output report 0x%02x rejected: %d", report_id, err);
	}
}

static size_t usb_output_report_bytes(uint8_t report_id)
{
	switch(report_id)
	{
		case ID_OUT_REPORT_HAPTIC_RUMBLE:
			return HID_RUMBLE_OUTPUT_REPORT_BYTES;
		case ID_OUT_REPORT_HAPTIC_PULSE:
			return HID_HAPTIC_PULSE_OUTPUT_REPORT_BYTES;
		case ID_OUT_REPORT_HAPTIC_COMMAND:
			return HID_HAPTIC_COMMAND_REPORT_BYTES;
		case ID_OUT_REPORT_HAPTIC_LFO_TONE:
			return HID_HAPTIC_LFO_TONE_REPORT_BYTES;
		case ID_OUT_REPORT_HAPTIC_LOG_SWEEP:
			return HID_HAPTIC_LOG_SWEEP_REPORT_BYTES;
		case ID_OUT_REPORT_HAPTIC_SCRIPT:
			return HID_HAPTIC_SCRIPT_REPORT_BYTES;
		case 0x86:
			return 4U;
		case 0x87:
		case 0x88:
		case 0x89:
			return VALVE_FEATURE_REPORT_SIZE;
		default:
			return 0U;
	}
}

static int usb_get_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                          uint8_t **data)
{
	uint8_t type = setup->wValue >> 8;
	uint8_t id = setup->wValue;

	ARG_UNUSED(dev);
	if(type != HID_REPORT_TYPE_FEATURE || id != 0x01)
	{
		return -ENOTSUP;
	}

	*data = feature_response;
	/*
	 * Zephyr initializes len to zero for control-IN requests. The callback
	 * must advertise the available response length; the USB core limits it
	 * to the host's wLength.
	 */
	*len = sizeof(feature_response);
	return 0;
}

static int usb_set_report(const struct device *dev, struct usb_setup_packet *setup, int32_t *len,
                          uint8_t **data)
{
	uint8_t type = setup->wValue >> 8;
	uint8_t id = setup->wValue;

	ARG_UNUSED(dev);
	if(type == HID_REPORT_TYPE_FEATURE && id == VALVE_FEATURE_REPORT_ID)
	{
		prepare_feature_response(*data, *len);
		return 0;
	}

	if(type == HID_REPORT_TYPE_OUTPUT)
	{
		size_t report_len = *len > 0 ? (size_t)*len : 0;
		size_t full_report_len = usb_output_report_bytes(id);

		if(full_report_len != 0U && report_len == full_report_len && (*data)[0] == id)
		{
			handle_output_report(0, *data, report_len);
		}
		else
		{
			handle_output_report(id, *data, report_len);
		}
		return 0;
	}
	return -ENOTSUP;
}

static void usb_input_ready(const struct device *dev)
{
	k_spinlock_key_t key;
	bool submit_next;
	int err;

	ARG_UNUSED(dev);

	key = k_spin_lock(&input_queue_lock);
	if(input_transfer_active)
	{
		input_transfer_active = false;
		if(input_queue_count != 0)
		{
			input_queue_head = (input_queue_head + 1U) % ARRAY_SIZE(input_queue);
			input_queue_count--;
		}
	}
	submit_next = input_queue_count != 0;
	k_spin_unlock(&input_queue_lock, key);

	if(submit_next)
	{
		err = usb_submit_next_input();
		if(err && err != -ENOTCONN)
		{
			LOG_WRN("USB queued input write failed: %d", err);
		}
	}
}

static void usb_output_ready(const struct device *dev)
{
	uint32_t bytes_read = 0;
	int err = hid_int_ep_read(dev, output_report, sizeof(output_report), &bytes_read);

	if(err)
	{
		LOG_WRN("USB interrupt output read failed: %d", err);
		return;
	}
	if(bytes_read != 0)
	{
		handle_output_report(0, output_report, bytes_read);
	}
}

static const struct hid_ops hid_ops = {
	.get_report = usb_get_report,
	.set_report = usb_set_report,
	.int_in_ready = usb_input_ready,
	.int_out_ready = usb_output_ready,
};

int transport_usb_init(void)
{
	int err;

	if(atomic_get(&usb_initialized) != 0)
	{
		return 0;
	}

	hid_dev = device_get_binding("HID_0");
	if(hid_dev == NULL)
	{
		return -ENODEV;
	}

	usb_hid_register_device(hid_dev, valve_hid_report_map, valve_hid_report_map_size, &hid_ops);
	err = usb_hid_init(hid_dev);
	if(err)
	{
		return err;
	}
	if(usb_vbus_present())
	{
		atomic_set(&usb_attached, 1);
	}
	err = usb_enable(usb_status_cb);
	if(err)
	{
		return err;
	}

	atomic_set(&usb_initialized, 1);
	LOG_INF("USB HID enabled alongside CDC-ACM");
	return 0;
}

int transport_usb_send(const struct controller_report *report)
{
	uint8_t body[VALVE_INPUT_42_SIZE];

	if(!usb_ready())
	{
		return -ENOTCONN;
	}

	triton_state_report_pack_body(body, sizeof(body), input_sequence++, report,
	                              triton_state_report_timestamp_us());
	return usb_queue_input_report(ID_TRITON_CONTROLLER_STATE, body, sizeof(body));
}

int transport_usb_send_battery_status(const struct controller_battery_report *report)
{
	uint8_t body[VALVE_INPUT_43_SIZE] = { 0 };

	if(!report->valid)
	{
		return -EINVAL;
	}
	if(!usb_ready())
	{
		return -ENOTCONN;
	}
	body[0] = report->charge_state;
	body[1] = report->level_percent;
	sys_put_le16(report->battery_mv, &body[2]);
	sys_put_le16(report->system_mv, &body[4]);
	sys_put_le16(report->input_mv, &body[6]);
	sys_put_le16(report->current_ma, &body[8]);
	sys_put_le16(report->input_current_ma, &body[10]);
	sys_put_le16(report->temperature_c, &body[12]);

	return usb_queue_input_report(ID_TRITON_BATTERY_STATUS, body, sizeof(body));
}

int transport_usb_send_input_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	return usb_queue_input_report(report_id, data, len);
}
