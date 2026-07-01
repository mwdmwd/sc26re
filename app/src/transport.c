/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

#include "controller.h"

LOG_MODULE_REGISTER(transport);

static enum transport_usb_radio_mode usb_radio_mode;

__weak bool transport_usb_attached(void)
{
	return false;
}

__weak bool transport_usb_configured(void)
{
	return false;
}

__weak void transport_ble_deactivate(void)
{
}

__weak int transport_ble_clear_bonds(uint8_t id)
{
	ARG_UNUSED(id);

	return -ENOTSUP;
}

__weak void transport_esb_deactivate(void)
{
}

__weak bool transport_esb_connected(void)
{
	return false;
}

__weak uint8_t transport_esb_channel(void)
{
	return 0;
}

__weak uint32_t transport_esb_host_frames_received(void)
{
	return 0;
}

__weak uint32_t transport_esb_channel_maps_received(void)
{
	return 0;
}

__weak uint32_t transport_esb_awake_frames_received(void)
{
	return 0;
}

__weak uint32_t transport_esb_polls_received(void)
{
	return 0;
}

__weak uint32_t transport_esb_replies_sent(void)
{
	return 0;
}

__weak uint32_t transport_esb_backend_end_events(void)
{
	return 0;
}

__weak uint32_t transport_esb_backend_crc_ok_events(void)
{
	return 0;
}

__weak uint32_t transport_esb_backend_crc_bad_events(void)
{
	return 0;
}

__weak uint32_t transport_esb_backend_rx_dropped_events(void)
{
	return 0;
}

__weak bool transport_esb_bond_loaded(void)
{
	return false;
}

__weak int transport_esb_get_debug(struct valve_esb_backend_debug *debug)
{
	ARG_UNUSED(debug);

	return -ENOTSUP;
}

__weak int transport_ble_send_battery_status(const struct controller_battery_report *report)
{
	ARG_UNUSED(report);

	return -ENOTSUP;
}

__weak int transport_usb_send_battery_status(const struct controller_battery_report *report)
{
	ARG_UNUSED(report);

	return -ENOTSUP;
}

__weak int transport_esb_send_battery_status(const struct controller_battery_report *report)
{
	ARG_UNUSED(report);

	return -ENOTSUP;
}

static int transport_start_radio(void)
{
	int err;

	if(IS_ENABLED(CONFIG_IBEX_BLE) && radio_personality_get() == RADIO_PERSONALITY_BLE)
	{
		err = transport_ble_init();
		if(err)
		{
			return err;
		}
	}

	if(IS_ENABLED(CONFIG_IBEX_ESB) && radio_personality_get() == RADIO_PERSONALITY_ESB)
	{
		err = transport_esb_init();
		if(err)
		{
			return err;
		}
	}

	return 0;
}

bool transport_usb_radio_allowed(void)
{
	return usb_radio_mode != TRANSPORT_USB_RADIO_OFF;
}

static bool transport_usb_reports_suppressed(void)
{
	return usb_radio_mode == TRANSPORT_USB_RADIO_DIAGNOSTIC;
}

static bool transport_usb_radio_mode_valid(enum transport_usb_radio_mode mode)
{
	return mode == TRANSPORT_USB_RADIO_OFF ||
	       mode == TRANSPORT_USB_RADIO_VBUS_CHARGE ||
	       mode == TRANSPORT_USB_RADIO_DIAGNOSTIC;
}

const char *transport_usb_radio_mode_name(void)
{
	switch(usb_radio_mode)
	{
		case TRANSPORT_USB_RADIO_OFF:
			return "off";
		case TRANSPORT_USB_RADIO_VBUS_CHARGE:
			return "vbus-charge";
		case TRANSPORT_USB_RADIO_DIAGNOSTIC:
			return "diagnostic";
		default:
			return "unknown";
	}
}

int transport_set_usb_radio_mode(enum transport_usb_radio_mode mode)
{
	enum transport_usb_radio_mode old_mode;
	bool was_allowed;
	int err = 0;

	if(!transport_usb_radio_mode_valid(mode))
	{
		return -EINVAL;
	}
	if(mode == TRANSPORT_USB_RADIO_VBUS_CHARGE && transport_usb_configured())
	{
		mode = TRANSPORT_USB_RADIO_OFF;
	}

	old_mode = usb_radio_mode;
	was_allowed = transport_usb_radio_allowed();

	if(old_mode == mode)
	{
		return 0;
	}

	usb_radio_mode = mode;
	if(!transport_usb_attached())
	{
		return 0;
	}

	if(transport_usb_radio_allowed())
	{
		if(usb_radio_mode == TRANSPORT_USB_RADIO_DIAGNOSTIC)
		{
			LOG_WRN("USB radio diagnostic mode enabled; USB HID reports suppressed");
		}
		else if(usb_radio_mode == TRANSPORT_USB_RADIO_VBUS_CHARGE)
		{
			LOG_INF("allowing radio while charger VBUS is attached");
		}

		if(!was_allowed)
		{
			err = transport_start_radio();
			if(err)
			{
				usb_radio_mode = old_mode;
			}
		}
	}
	else
	{
		LOG_INF("USB radio mode disabled");
		if(was_allowed)
		{
			transport_ble_deactivate();
			transport_esb_deactivate();
		}
	}
	return err;
}

void transport_enter_usb_mode(void)
{
	radio_personality_cancel_pending_persist();

	if(usb_radio_mode == TRANSPORT_USB_RADIO_DIAGNOSTIC)
	{
		return;
	}

	if(usb_radio_mode == TRANSPORT_USB_RADIO_VBUS_CHARGE)
	{
		LOG_INF("USB configured; leaving charger VBUS radio mode");
		usb_radio_mode = TRANSPORT_USB_RADIO_OFF;
	}

	transport_ble_deactivate();
	transport_esb_deactivate();
}

int transport_init(void)
{
	int err;

	if(IS_ENABLED(CONFIG_IBEX_USB_HID))
	{
		err = transport_usb_init();
		if(err)
		{
			return err;
		}
		if(transport_usb_attached() && !transport_usb_radio_allowed())
		{
			LOG_INF("USB attached; leaving radio transports inactive");
			return 0;
		}
	}

	return transport_start_radio();
}

int transport_send(const struct controller_report *report)
{
	int ble_err = -ENOTSUP;
	int usb_err = -ENOTSUP;
	int esb_err = -ENOTSUP;

	if(IS_ENABLED(CONFIG_IBEX_USB_HID))
	{
		if(!transport_usb_reports_suppressed())
		{
			usb_err = transport_usb_send(report);
			if(transport_usb_configured())
			{
				return usb_err;
			}
		}
	}

	if(IS_ENABLED(CONFIG_IBEX_BLE) && radio_personality_get() == RADIO_PERSONALITY_BLE)
	{
		ble_err = transport_ble_send(report);
	}

	if(IS_ENABLED(CONFIG_IBEX_ESB) && radio_personality_get() == RADIO_PERSONALITY_ESB)
	{
		esb_err = transport_esb_send(report);
	}

	if(usb_err == 0 || ble_err == 0 || esb_err == 0)
	{
		return 0;
	}
	if(usb_err != -ENOTSUP && usb_err != -ENOTCONN && usb_err != -EAGAIN && usb_err != -EBUSY)
	{
		return usb_err;
	}
	if(ble_err != -ENOTSUP && ble_err != -ENOTCONN && ble_err != -EAGAIN && ble_err != -EBUSY)
	{
		return ble_err;
	}
	if(esb_err != -ENOTSUP && esb_err != -ENOTCONN)
	{
		return esb_err;
	}

	return -ENOTCONN;
}

int transport_send_battery_status(const struct controller_battery_report *report)
{
	int ble_err = -ENOTSUP;
	int usb_err = -ENOTSUP;
	int esb_err = -ENOTSUP;

	if(IS_ENABLED(CONFIG_IBEX_USB_HID))
	{
		if(!transport_usb_reports_suppressed())
		{
			usb_err = transport_usb_send_battery_status(report);
			if(transport_usb_configured())
			{
				return usb_err;
			}
		}
	}

	if(IS_ENABLED(CONFIG_IBEX_BLE) && radio_personality_get() == RADIO_PERSONALITY_BLE)
	{
		ble_err = transport_ble_send_battery_status(report);
	}

	if(IS_ENABLED(CONFIG_IBEX_ESB) && radio_personality_get() == RADIO_PERSONALITY_ESB)
	{
		esb_err = transport_esb_send_battery_status(report);
	}

	if(usb_err == 0 || ble_err == 0 || esb_err == 0)
	{
		return 0;
	}
	if(usb_err != -ENOTSUP && usb_err != -ENOTCONN && usb_err != -EAGAIN && usb_err != -EBUSY)
	{
		return usb_err;
	}
	if(ble_err != -ENOTSUP && ble_err != -ENOTCONN && ble_err != -EAGAIN && ble_err != -EBUSY)
	{
		return ble_err;
	}
	if(esb_err != -ENOTSUP && esb_err != -ENOTCONN)
	{
		return esb_err;
	}

	return -ENOTCONN;
}
