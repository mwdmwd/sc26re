/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>

#include <zephyr/logging/log.h>
#include <zephyr/toolchain.h>

#include "controller.h"

LOG_MODULE_REGISTER(transport);

static bool radio_debug_usb_allowed;

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

	if(IS_ENABLED(CONFIG_IBEX_ESB) && radio_personality_get() != RADIO_PERSONALITY_BLE)
	{
		err = transport_esb_init();
		if(err)
		{
			return err;
		}
	}

	return 0;
}

int transport_radio_debug_allow_usb(bool allow)
{
	int err = 0;

	radio_debug_usb_allowed = allow;
	if(!transport_usb_attached())
	{
		return 0;
	}

	if(allow)
	{
		LOG_WRN("diagnostic mode: allowing radio while USB is attached; "
		        "USB HID reports suppressed");
		err = transport_start_radio();
	}
	else
	{
		LOG_INF("USB radio diagnostic mode disabled");
		transport_ble_deactivate();
		transport_esb_deactivate();
	}
	return err;
}

bool transport_radio_debug_usb_allowed(void)
{
	return radio_debug_usb_allowed;
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
		if(transport_usb_attached() && !radio_debug_usb_allowed)
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
		if(!radio_debug_usb_allowed)
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

	if(IS_ENABLED(CONFIG_IBEX_ESB) && radio_personality_get() != RADIO_PERSONALITY_BLE)
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
		if(!radio_debug_usb_allowed)
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

	if(IS_ENABLED(CONFIG_IBEX_ESB) && radio_personality_get() != RADIO_PERSONALITY_BLE)
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
