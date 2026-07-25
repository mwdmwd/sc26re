/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

#include <hal/nrf_power.h>

#include "controller.h"
#include "haptics.h"
#include "lizard.h"
#include "power.h"
#include "sdl/controller_structs.h"
#include "triton_state_report.h"
#include "valve_feature.h"
#include "valve_hid_report_map.h"
#include "valve_identity.h"

#if CONFIG_IBEX_RGBW_LED
#include "rgbw_led.h"
#endif

LOG_MODULE_REGISTER(transport_usb);

#define VALVE_INPUT_42_SIZE 53
#define VALVE_INPUT_43_SIZE 14
#define USB_UNPLUG_POWEROFF_DELAY_MS 250
#define USB_INPUT_RETRY_DELAY_MS 1
#define USB_INPUT_REPORT_MAX_SIZE VALVE_FEATURE_REPORT_SIZE
#define USB_INPUT_QUEUE_DEPTH 32
#define USB_MAX_POWER_2MA 50

USBD_DEVICE_DEFINE(ibex_usbd, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_IBEX_USB_VID,
                   CONFIG_IBEX_USB_PID);
USBD_DESC_LANG_DEFINE(ibex_usb_lang);
USBD_DESC_MANUFACTURER_DEFINE(ibex_usb_manufacturer, CONFIG_IBEX_USB_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(ibex_usb_product, CONFIG_IBEX_USB_PRODUCT);

static uint8_t ibex_usb_serial_text[VALVE_IDENTITY_SERIAL_TEXT_SIZE + 1];
static struct usbd_desc_node ibex_usb_serial = {
	.str = {
		.utype = USBD_DUT_STRING_SERIAL_NUMBER,
		.ascii7 = true,
	},
	.ptr = ibex_usb_serial_text,
	.bLength = USB_STRING_DESCRIPTOR_LENGTH(VALVE_IDENTITY_SERIAL_PLACEHOLDER),
	.bDescriptorType = USB_DESC_STRING,
};

USBD_CONFIGURATION_DEFINE(ibex_usb_fs_config, USB_SCD_SELF_POWERED, USB_MAX_POWER_2MA, NULL);

struct usb_input_queue_entry
{
	/* Keep first so input_transfer.data inherits input_transfer's UDC alignment. */
	uint8_t data[USB_INPUT_REPORT_MAX_SIZE];
	uint8_t len;
};

static const struct device *hid_dev;
static uint8_t input_sequence;
static uint8_t feature_response[VALVE_FEATURE_REPORT_SIZE];
K_MSGQ_DEFINE(input_queue, sizeof(struct usb_input_queue_entry), USB_INPUT_QUEUE_DEPTH, 1);
static struct k_spinlock input_queue_lock;
static struct usb_input_queue_entry input_transfer __aligned(UDC_BUF_ALIGN);
static const uint8_t *input_transfer_report;
static bool input_transfer_active;
static bool input_transfer_dequeue_on_completion;
static atomic_t usb_initialized;
static atomic_t usb_attached;
static atomic_t usb_configured;
static atomic_t usb_suspended;
static atomic_t usb_radio_off_mode;
static atomic_t usb_unplug_poweroff_pending;

static int usb_submit_next_input(void);
static void usb_input_retry_work_handler(struct k_work *work);
static void usb_unplug_poweroff_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(usb_input_retry_work, usb_input_retry_work_handler);
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
	struct usb_input_queue_entry entry;
	k_spinlock_key_t key;

	(void)k_work_cancel_delayable(&usb_input_retry_work);
	key = k_spin_lock(&input_queue_lock);

	while(k_msgq_get(&input_queue, &entry, K_NO_WAIT) == 0)
	{
	}
	/* Keep an active buffer pinned, but its drained queue entry is retired. */
	input_transfer_dequeue_on_completion = false;
	k_spin_unlock(&input_queue_lock, key);
}

static int usb_submit_next_input(void)
{
	k_spinlock_key_t key;
	int err;

	if(!usb_ready())
	{
		return -ENOTCONN;
	}

	key = k_spin_lock(&input_queue_lock);
	if(input_transfer_active || k_msgq_peek(&input_queue, &input_transfer) != 0)
	{
		k_spin_unlock(&input_queue_lock, key);
		return 0;
	}
	input_transfer_report = input_transfer.data;
	input_transfer_active = true;
	input_transfer_dequeue_on_completion = true;
	k_spin_unlock(&input_queue_lock, key);

	err = hid_device_submit_report(hid_dev, input_transfer.len, input_transfer.data);
	if(err)
	{
		key = k_spin_lock(&input_queue_lock);
		input_transfer_report = NULL;
		input_transfer_active = false;
		input_transfer_dequeue_on_completion = false;
		k_spin_unlock(&input_queue_lock, key);
	}
	return err;
}

static void usb_input_retry_work_handler(struct k_work *work)
{
	int err;

	ARG_UNUSED(work);

	err = usb_submit_next_input();
	if(err && err != -ENOTCONN)
	{
		(void)k_work_reschedule(&usb_input_retry_work, K_MSEC(USB_INPUT_RETRY_DELAY_MS));
	}
}

static void usb_retry_input_later(int err)
{
	if(err == 0 || err == -ENOTCONN)
	{
		return;
	}

	LOG_WRN("USB queued input write failed: %d; retrying", err);
	(void)k_work_reschedule(&usb_input_retry_work, K_MSEC(USB_INPUT_RETRY_DELAY_MS));
}

static int usb_queue_input_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	struct usb_input_queue_entry entry = {
		.len = (uint8_t)(len + 1U),
		.data = { report_id },
	};
	k_spinlock_key_t key;
	int err;

	if(!usb_ready())
	{
		return -ENOTCONN;
	}
	if(data == NULL || len + 1U > USB_INPUT_REPORT_MAX_SIZE)
	{
		return -EINVAL;
	}
	memcpy(&entry.data[1], data, len);

	key = k_spin_lock(&input_queue_lock);
	err = k_msgq_put(&input_queue, &entry, K_NO_WAIT);
	k_spin_unlock(&input_queue_lock, key);
	if(err)
	{
		return err == -ENOMSG ? -ENOMEM : err;
	}

	err = usb_submit_next_input();
	usb_retry_input_later(err);
	/* The report is durable once it has entered input_queue. */
	return 0;
}

static void usb_cancel_unplug_poweroff(void)
{
	atomic_clear(&usb_unplug_poweroff_pending);
	k_work_cancel_delayable(&usb_unplug_poweroff_work);
}

static void usb_schedule_unplug_poweroff(void)
{
	atomic_set(&usb_unplug_poweroff_pending, 1);
	k_work_reschedule(&usb_unplug_poweroff_work, K_MSEC(USB_UNPLUG_POWEROFF_DELAY_MS));
}

static void usb_enter_mode(void)
{
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

	/*
	 * A VBUS_REMOVED message is authoritative. USB protocol messages such
	 * as RESUME can still be queued while the controller is shutting down,
	 * so neither their attached/configured state nor the raw Nordic VBUS
	 * register may veto this request. Only a new VBUS_READY message clears
	 * the pending flag before this work runs.
	 */
	if(!atomic_cas(&usb_unplug_poweroff_pending, 1, 0))
	{
		return;
	}

	LOG_INF("USB unplugged; powering off");
	power_off();
}

static void usb_msg_cb(struct usbd_context *const usbd_ctx, const struct usbd_msg *const msg)
{
	bool was_attached = transport_usb_attached();
	int err;

	switch(msg->type)
	{
		case USBD_MSG_VBUS_READY:
			usb_cancel_unplug_poweroff();
			atomic_set(&usb_attached, 1);
			err = usbd_enable(usbd_ctx);
			if(err)
			{
				LOG_ERR("Failed to enable USB device: %d", err);
			}
			break;
		case USBD_MSG_CONFIGURATION:
			if(msg->status != 0)
			{
				atomic_set(&usb_attached, 1);
				atomic_set(&usb_configured, 1);
				atomic_clear(&usb_suspended);
				atomic_set(&usb_radio_off_mode, 1);
				usb_input_queue_reset();
				lizard_transport_reset();
				usb_enter_mode();
			}
			else
			{
				atomic_clear(&usb_configured);
				usb_input_queue_reset();
			}
			break;
		case USBD_MSG_VBUS_REMOVED:
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
			err = usbd_disable(usbd_ctx);
			if(err)
			{
				LOG_ERR("Failed to disable USB device: %d", err);
			}
			break;
		case USBD_MSG_RESET:
			atomic_clear(&usb_configured);
			atomic_clear(&usb_suspended);
			usb_input_queue_reset();
			break;
		case USBD_MSG_SUSPEND:
			atomic_set(&usb_suspended, 1);
			break;
		case USBD_MSG_RESUME:
			atomic_clear(&usb_suspended);
			(void)k_work_reschedule(&usb_input_retry_work, K_NO_WAIT);
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

static int usb_get_report(const struct device *dev, uint8_t type, uint8_t id, uint16_t len,
                          uint8_t *const data)
{
	size_t response_len;

	ARG_UNUSED(dev);
	if(type != HID_REPORT_TYPE_FEATURE || id != 0x01)
	{
		return -ENOTSUP;
	}

	response_len = MIN((size_t)len, sizeof(feature_response));
	memcpy(data, feature_response, response_len);
	return (int)response_len;
}

static int usb_set_report(const struct device *dev, uint8_t type, uint8_t id, uint16_t len,
                          const uint8_t *const data)
{
	ARG_UNUSED(dev);
	if(type == HID_REPORT_TYPE_FEATURE && id == VALVE_FEATURE_REPORT_ID)
	{
		prepare_feature_response(data, len);
		return 0;
	}

	if(type == HID_REPORT_TYPE_OUTPUT)
	{
		size_t report_len = len;
		size_t full_report_len = usb_output_report_bytes(id);

		if(full_report_len != 0U && report_len == full_report_len && data[0] == id)
		{
			handle_output_report(0, data, report_len);
		}
		else
		{
			handle_output_report(id, data, report_len);
		}
		return 0;
	}
	return -ENOTSUP;
}

static void usb_input_ready(const struct device *dev, const uint8_t *const report)
{
	k_spinlock_key_t key;
	bool completion_matched;
	bool submit_next;
	int err;

	ARG_UNUSED(dev);

	key = k_spin_lock(&input_queue_lock);
	completion_matched = input_transfer_active && report == input_transfer_report;
	if(completion_matched)
	{
		input_transfer_report = NULL;
		input_transfer_active = false;
		if(input_transfer_dequeue_on_completion)
		{
			(void)k_msgq_get(&input_queue, &input_transfer, K_NO_WAIT);
		}
		input_transfer_dequeue_on_completion = false;
	}
	submit_next = completion_matched && k_msgq_num_used_get(&input_queue) != 0;
	k_spin_unlock(&input_queue_lock, key);

	if(submit_next)
	{
		err = usb_submit_next_input();
		usb_retry_input_later(err);
	}
}

static void usb_output_ready(const struct device *dev, uint16_t len, const uint8_t *const data)
{
	ARG_UNUSED(dev);
	if(len != 0)
	{
		handle_output_report(0, data, len);
	}
}

static const struct hid_device_ops hid_ops = {
	.get_report = usb_get_report,
	.set_report = usb_set_report,
	.input_report_done = usb_input_ready,
	.output_report = usb_output_ready,
};

static int usb_device_init(void)
{
	const char *serial = valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL);
	int err;

	memcpy(ibex_usb_serial_text, serial, VALVE_IDENTITY_SERIAL_TEXT_SIZE);
	ibex_usb_serial_text[VALVE_IDENTITY_SERIAL_TEXT_SIZE] = '\0';

	err = usbd_add_descriptor(&ibex_usbd, &ibex_usb_lang);
	if(err)
	{
		return err;
	}
	err = usbd_add_descriptor(&ibex_usbd, &ibex_usb_manufacturer);
	if(err)
	{
		return err;
	}
	err = usbd_add_descriptor(&ibex_usbd, &ibex_usb_product);
	if(err)
	{
		return err;
	}
	err = usbd_add_descriptor(&ibex_usbd, &ibex_usb_serial);
	if(err)
	{
		return err;
	}

	err = usbd_add_configuration(&ibex_usbd, USBD_SPEED_FS, &ibex_usb_fs_config);
	if(err)
	{
		return err;
	}
	err = usbd_register_all_classes(&ibex_usbd, USBD_SPEED_FS, 1, NULL);
	if(err)
	{
		return err;
	}
	err = usbd_device_set_code_triple(&ibex_usbd, USBD_SPEED_FS, USB_BCC_MISCELLANEOUS, 0x02, 0x01);
	if(err)
	{
		return err;
	}
	usbd_self_powered(&ibex_usbd, true);

	err = usbd_msg_register_cb(&ibex_usbd, usb_msg_cb);
	if(err)
	{
		return err;
	}
	return usbd_init(&ibex_usbd);
}

int transport_usb_init(void)
{
	int err;

	if(atomic_get(&usb_initialized) != 0)
	{
		return 0;
	}

	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	if(!device_is_ready(hid_dev))
	{
		return -ENODEV;
	}

	err = hid_device_register(hid_dev, valve_hid_report_map, valve_hid_report_map_size, &hid_ops);
	if(err)
	{
		return err;
	}
	err = usb_device_init();
	if(err)
	{
		return err;
	}
	if(usb_vbus_present())
	{
		atomic_set(&usb_attached, 1);
	}
	if(!usbd_can_detect_vbus(&ibex_usbd))
	{
		err = usbd_enable(&ibex_usbd);
		if(err)
		{
			return err;
		}
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
	sys_put_le16(report->temperature_mc, &body[12]);

	return usb_queue_input_report(ID_TRITON_BATTERY_STATUS, body, sizeof(body));
}

int transport_usb_send_input_report(uint8_t report_id, const uint8_t *data, size_t len)
{
	return usb_queue_input_report(report_id, data, len);
}
