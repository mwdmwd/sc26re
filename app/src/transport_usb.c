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

static const struct device *hid_dev;
static uint8_t input_sequence;
static uint8_t feature_response[VALVE_FEATURE_REPORT_SIZE];
static uint8_t input_report[1 + VALVE_INPUT_42_SIZE];
static uint8_t battery_report[1 + VALVE_INPUT_43_SIZE];
static atomic_t input_busy;
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
	radio_personality_cancel_pending_persist();
	(void)transport_allow_radio_with_usb(false);

#if CONFIG_IBEX_RGBW_LED
	if(transport_usb_configured())
	{
		rgbw_led_set(0, 255, 0, 0);
	}
#endif

	if(!transport_radio_debug_usb_allowed())
	{
		transport_ble_deactivate();
		transport_esb_deactivate();
	}
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
			usb_enter_mode();
			break;
		case USB_DC_DISCONNECTED:
			atomic_clear(&usb_attached);
			atomic_clear(&usb_configured);
			atomic_clear(&usb_suspended);
			if(was_attached && atomic_cas(&usb_radio_off_mode, 1, 0))
			{
				usb_schedule_unplug_poweroff();
			}
			break;
		case USB_DC_RESET:
			atomic_clear(&usb_configured);
			atomic_clear(&usb_suspended);
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

	/* Accept output reports for host compatibility; haptics follow later. */
	if(type == HID_REPORT_TYPE_OUTPUT)
	{
		return 0;
	}
	return -ENOTSUP;
}

static void usb_input_ready(const struct device *dev)
{
	ARG_UNUSED(dev);
	atomic_clear(&input_busy);
}

static const struct hid_ops hid_ops = {
	.get_report = usb_get_report,
	.set_report = usb_set_report,
	.int_in_ready = usb_input_ready,
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
	int err;

	if(!usb_ready())
	{
		return -ENOTCONN;
	}
	if(!atomic_cas(&input_busy, 0, 1))
	{
		return -EBUSY;
	}

	memset(input_report, 0, sizeof(input_report));
	input_report[0] = ID_TRITON_CONTROLLER_STATE;
	triton_state_report_pack_body(&input_report[1], VALVE_INPUT_42_SIZE, input_sequence++, report,
	                              triton_state_report_timestamp_us());

	err = hid_int_ep_write(hid_dev, input_report, sizeof(input_report), NULL);
	if(err)
	{
		atomic_clear(&input_busy);
	}
	return err;
}

int transport_usb_send_battery_status(const struct controller_battery_report *report)
{
	int err;

	if(!report->valid)
	{
		return -EINVAL;
	}
	if(!usb_ready())
	{
		return -ENOTCONN;
	}
	if(!atomic_cas(&input_busy, 0, 1))
	{
		return -EBUSY;
	}

	memset(battery_report, 0, sizeof(battery_report));
	battery_report[0] = ID_TRITON_BATTERY_STATUS;
	battery_report[1] = report->charge_state;
	battery_report[2] = report->level_percent;
	sys_put_le16(report->battery_mv, &battery_report[3]);
	sys_put_le16(report->system_mv, &battery_report[5]);
	sys_put_le16(report->input_mv, &battery_report[7]);
	sys_put_le16(report->current_ma, &battery_report[9]);
	sys_put_le16(report->input_current_ma, &battery_report[11]);
	sys_put_le16(report->temperature_c, &battery_report[13]);

	err = hid_int_ep_write(hid_dev, battery_report, sizeof(battery_report), NULL);
	if(err)
	{
		atomic_clear(&input_busy);
	}
	return err;
}
