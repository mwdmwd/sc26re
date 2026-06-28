/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#include "battery.h"
#include "controller.h"
#include "esb_backend.h"
#include "ibex_settings_registry.h"
#include "olympus.h"
#include "power.h"
#include "rgbw_led.h"
#include "valve_identity.h"

static int cmd_reboot(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return power_reboot_normal();
}

static int cmd_reboot_isp(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	return power_reboot_to_valve_isp();
}

static int cmd_power_off(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(shell);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	power_off();
	return 0;
}

static int cmd_power_wake(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t latches[2];

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	power_boot_gpio_latches(latches);
	shell_print(shell, "boot reset reason: 0x%08x", power_boot_reset_reason());
	shell_print(shell, "boot GPIO latches: p0=0x%08x p1=0x%08x", latches[0], latches[1]);
	return 0;
}

static int cmd_status(const struct shell *shell, size_t argc, char **argv)
{
	struct controller_battery_report battery;
	int battery_err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	battery_err = battery_get_status(&battery);
	shell_print(shell, "board: %s", CONFIG_BOARD_TARGET);
	shell_print(shell, "firmware: %s (build timestamp 0x%08x)", CONFIG_IBEX_FIRMWARE_IDENTITY,
	            CONFIG_IBEX_BUILD_TIMESTAMP);
	shell_print(shell, "radio personality: %s", radio_personality_name());
	shell_print(shell, "ESB: bond=%u connected=%u channel=%u E1=%u E4=%u E7=%u E3=%u replies=%u",
	            transport_esb_bond_loaded(), transport_esb_connected(), transport_esb_channel(),
	            transport_esb_host_frames_received(), transport_esb_channel_maps_received(),
	            transport_esb_awake_frames_received(), transport_esb_polls_received(),
	            transport_esb_replies_sent());
	shell_print(shell, "ESB radio: end=%u crc-ok=%u crc-bad=%u drop=%u",
	            transport_esb_backend_end_events(), transport_esb_backend_crc_ok_events(),
	            transport_esb_backend_crc_bad_events(), transport_esb_backend_rx_dropped_events());
	if(!battery_err)
	{
		shell_print(shell, "battery: %u%% %umV state=%s input=%umV current=%umA type=%u",
		            battery.level_percent, battery.battery_mv,
		            battery_charge_state_name(battery.charge_state), battery.input_mv,
		            battery.current_ma, battery.charger_type);
	}
	return 0;
}

static int cmd_battery_status(const struct shell *shell, size_t argc, char **argv)
{
	struct controller_battery_report battery;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = battery_get_status(&battery);
	if(err)
	{
		shell_error(shell, "battery status unavailable: %d", err);
		return err;
	}

	shell_print(shell, "valid: %u", battery.valid);
	shell_print(shell, "charge state: %u (%s)", battery.charge_state,
	            battery_charge_state_name(battery.charge_state));
	shell_print(shell, "level: %u%%", battery.level_percent);
	shell_print(shell, "battery voltage: %u mV", battery.battery_mv);
	shell_print(shell, "system voltage: %u mV", battery.system_mv);
	shell_print(shell, "input voltage: %u mV", battery.input_mv);
	shell_print(shell, "charge/discharge current: %u mA", battery.current_ma);
	shell_print(shell, "input current: %u mA", battery.input_current_ma);
	shell_print(shell, "temperature estimate: %u C", battery.temperature_c);
	shell_print(shell, "charger type: %u", battery.charger_type);
	return 0;
}

static int cmd_radio_status(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "radio personality: %s", radio_personality_name());
	shell_print(shell, "USB radio mode: %s", transport_usb_radio_mode_name());
	return 0;
}

#if CONFIG_IBEX_BLE
static int cmd_radio_ble(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "rebooting into BLE personality");
	radio_personality_reboot_into(RADIO_PERSONALITY_BLE);
	return 0;
}
#endif

static int cmd_radio_esb(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if(!IS_ENABLED(CONFIG_IBEX_ESB))
	{
		shell_error(shell, "ESB is not included in this build");
		return -ENOTSUP;
	}

	shell_print(shell, "rebooting into ESB personality");
	radio_personality_reboot_into(RADIO_PERSONALITY_ESB);
	return 0;
}

static int cmd_radio_esb_bond(const struct shell *shell, size_t argc, char **argv)
{
	unsigned long proteus_uuid;
	unsigned long ibex_uuid;
	const char *serial = NULL;
	int err = 0;

	enum
	{
		ESB_BOND_SERIAL_INPUT_SIZE = VALVE_IDENTITY_SERIAL_TEXT_SIZE,
	};

	if(!IS_ENABLED(CONFIG_IBEX_ESB))
	{
		shell_error(shell, "ESB is not included in this build");
		return -ENOTSUP;
	}

	proteus_uuid = shell_strtoul(argv[1], 16, &err);
	if(err || proteus_uuid > UINT32_MAX)
	{
		shell_error(shell, "invalid proteus UUID word");
		return -EINVAL;
	}
	err = 0;
	ibex_uuid = shell_strtoul(argv[2], 16, &err);
	if(err || ibex_uuid > UINT32_MAX)
	{
		shell_error(shell, "invalid ibex UUID word");
		return -EINVAL;
	}
	if(argc > 3)
	{
		if(strlen(argv[3]) > ESB_BOND_SERIAL_INPUT_SIZE)
		{
			shell_error(shell, "serial must be at most %u bytes", ESB_BOND_SERIAL_INPUT_SIZE);
			return -EINVAL;
		}
		serial = argv[3];
	}

	err = transport_esb_provision_bond((uint32_t)proteus_uuid, (uint32_t)ibex_uuid, serial);
	if(err)
	{
		shell_error(shell, "ESB bond provision failed: %d", err);
		return err;
	}

	shell_print(shell, "ESB bond provisioned: %08x/%08x serial=%s", (uint32_t)proteus_uuid,
	            (uint32_t)ibex_uuid,
	            serial ? serial : valve_identity_serial(VALVE_IDENTITY_UNIT_SERIAL));
	return 0;
}

static int cmd_radio_esb_regs(const struct shell *shell, size_t argc, char **argv)
{
	struct valve_esb_backend_debug debug;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if(!IS_ENABLED(CONFIG_IBEX_ESB))
	{
		shell_error(shell, "ESB is not included in this build");
		return -ENOTSUP;
	}

	err = transport_esb_get_debug(&debug);
	if(err)
	{
		shell_error(shell, "ESB register dump unavailable: %d", err);
		return err;
	}

	shell_print(shell, "ESB cfg: ch=%u base=%02x%02x%02x%02x prefix=%02x rx=%u tx=%u",
	            debug.channel, debug.base[0], debug.base[1], debug.base[2], debug.base[3],
	            debug.prefix, debug.rx_running, debug.tx_active);
	shell_print(shell, "RADIO: mode=0x%08x freq=0x%08x state=0x%08x shorts=0x%08x inten=0x%08x",
	            debug.mode, debug.frequency, debug.state, debug.shorts, debug.intenset);
	shell_print(shell, "RADIO pkt: pcnf0=0x%08x pcnf1=0x%08x ptr=0x%08x", debug.pcnf0, debug.pcnf1,
	            debug.packetptr);
	shell_print(shell, "RADIO addr: base0=0x%08x prefix0=0x%08x rxaddr=0x%08x txaddr=0x%08x",
	            debug.base0, debug.prefix0, debug.rxaddresses, debug.txaddress);
	shell_print(shell, "RADIO crc: cnf=0x%08x poly=0x%08x init=0x%08x status=0x%08x", debug.crccnf,
	            debug.crcpoly, debug.crcinit, debug.crcstatus);
	shell_print(shell, "RADIO events: ready=%u address=%u end=%u disabled=%u", debug.events_ready,
	            debug.events_address, debug.events_end, debug.events_disabled);
	return 0;
}

static int cmd_radio_usb_debug(const struct shell *shell, size_t argc, char **argv)
{
	bool allow;
	int err;

	if(strcmp(argv[1], "on") == 0)
	{
		allow = true;
	}
	else if(strcmp(argv[1], "off") == 0)
	{
		allow = false;
	}
	else
	{
		shell_error(shell, "usage: steamctl radio usb_debug <on|off>");
		return -EINVAL;
	}

	err = transport_set_usb_radio_mode(allow ? TRANSPORT_USB_RADIO_DIAGNOSTIC
	                                         : TRANSPORT_USB_RADIO_OFF);
	if(err)
	{
		shell_error(shell, "radio diagnostic mode failed: %d", err);
		return err;
	}

	shell_print(shell, "USB radio diagnostic mode: %s", allow ? "on" : "off");
	return 0;
}

#if CONFIG_IBEX_BLE
static int cmd_radio_ble_disconnect(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	transport_ble_deactivate();
	shell_print(shell, "BLE transport disconnected/deactivated");
	return 0;
}

static int cmd_radio_ble_unpair(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t id = TRANSPORT_BLE_ID_ALL;
	int err;

	if(argc > 1)
	{
		uint32_t parsed_id;

		if(strcmp(argv[1], "all") != 0)
		{
			err = 0;
			parsed_id = shell_strtoul(argv[1], 10, &err);
			if(err || parsed_id > UINT8_MAX)
			{
				shell_error(shell, "BLE identity must be 0..%u or all", UINT8_MAX);
				return -EINVAL;
			}
			id = (uint8_t)parsed_id;
		}
	}

	err = transport_ble_clear_bonds(id);
	if(err)
	{
		shell_error(shell, "BLE bond clear failed: %d", err);
		return err;
	}

	if(id == TRANSPORT_BLE_ID_ALL)
	{
		shell_print(shell, "BLE bonds cleared");
	}
	else
	{
		shell_print(shell, "BLE bonds cleared for identity %u", id);
	}
	return 0;
}
#endif

static int cmd_settings_get(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t id;
	int16_t current;
	int16_t default_value;
	int16_t min_value;
	int16_t max_value;
	const char *name;
	const char *source;
	const char *persist_path;
	int err = 0;

	id = shell_strtoul(argv[1], 10, &err);
	if(err ||
	   id >= IBEX_SETTING_COUNT ||
	   !ibex_setting_get((uint8_t)id, &current) ||
	   !ibex_setting_get_meta((uint8_t)id, IBEX_SETTING_DEFAULT, &default_value) ||
	   !ibex_setting_get_meta((uint8_t)id, IBEX_SETTING_MIN, &min_value) ||
	   !ibex_setting_get_meta((uint8_t)id, IBEX_SETTING_MAX, &max_value))
	{
		shell_error(shell, "setting id must be 0..%u", IBEX_SETTING_COUNT - 1);
		return -EINVAL;
	}

	name = ibex_setting_name((uint8_t)id);
	source = ibex_setting_name_source((uint8_t)id);
	persist_path = ibex_setting_persist_path((uint8_t)id);

	shell_print(shell, "%u %s [%s]: current=%d default=%d min=%d max=%d", id,
	            name ? name : "<unknown>", source ? source : "unknown", current, default_value,
	            min_value, max_value);
	if(persist_path)
	{
		shell_print(shell, "persist path: %s", persist_path);
	}
	return 0;
}

static int cmd_settings_set(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t id;
	int32_t value;
	int16_t clamped;
	int err = 0;

	id = shell_strtoul(argv[1], 10, &err);
	if(err || id >= IBEX_SETTING_COUNT)
	{
		shell_error(shell, "setting id must be 0..%u", IBEX_SETTING_COUNT - 1);
		return -EINVAL;
	}

	err = 0;
	value = shell_strtol(argv[2], 10, &err);
	if(err || value < INT16_MIN || value > INT16_MAX)
	{
		shell_error(shell, "setting value must fit s16");
		return -EINVAL;
	}

	err = ibex_setting_set((uint8_t)id, (int16_t)value);
	if(err)
	{
		return err;
	}
	(void)ibex_setting_get((uint8_t)id, &clamped);
	shell_print(shell, "%u = %d", id, clamped);
	return 0;
}

static int cmd_settings_defaults(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ibex_settings_registry_reset_defaults();
	shell_print(shell, "Ibex runtime settings reset to defaults");
	return 0;
}

#if CONFIG_IBEX_OLYMPUS
static void print_olympus_pad_debug(const struct shell *shell, const char *name,
                                    const struct olympus_pad_debug *pad)
{
	shell_print(shell,
	            "%s: touch=%u stick_touch=%u grip_touch=%u click=%u x=%d y=%d pressure=%u "
	            "raw_pressure=%d raw_click=%d raw_grip=%d grip=%d "
	            "peak=%d peak_x=%d peak_y=%d thr=%d/%d grip_thr=%d/%d noise=%d click_thr=%d",
	            name, pad->touched, pad->stick_touched, pad->grip_touched, pad->clicked, pad->x,
	            pad->y, pad->pressure, pad->raw_pressure, pad->raw_click, pad->raw_grip,
	            pad->grip_value, pad->peak, pad->peak_x, pad->peak_y, pad->touch_threshold,
	            pad->release_threshold, pad->grip_touch_threshold, pad->grip_release_threshold,
	            pad->noise_threshold, pad->pad_click_threshold);
}

static int cmd_olympus_status(const struct shell *shell, size_t argc, char **argv)
{
	struct olympus_debug_snapshot snapshot;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	olympus_get_debug_snapshot(&snapshot);
	shell_print(shell, "Olympus frames: %u", snapshot.frame_count);
	print_olympus_pad_debug(shell, "left", &snapshot.left);
	print_olympus_pad_debug(shell, "right", &snapshot.right);
	return 0;
}
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(sub_battery,
                               SHELL_CMD(status, NULL, "Show cached battery and charger status",
                                         cmd_battery_status),
                               SHELL_SUBCMD_SET_END);

#if CONFIG_IBEX_RGBW_LED
static int cmd_led_set(const struct shell *shell, size_t argc, char **argv)
{
	uint32_t r, g, b, w = 0;
	int err = 0;

	r = shell_strtoul(argv[1], 10, &err);
	if(err || r > 255)
	{
		shell_error(shell, "r must be 0..255");
		return -EINVAL;
	}

	err = 0;
	g = shell_strtoul(argv[2], 10, &err);
	if(err || g > 255)
	{
		shell_error(shell, "g must be 0..255");
		return -EINVAL;
	}

	err = 0;
	b = shell_strtoul(argv[3], 10, &err);
	if(err || b > 255)
	{
		shell_error(shell, "b must be 0..255");
		return -EINVAL;
	}

	if(argc > 4)
	{
		err = 0;
		w = shell_strtoul(argv[4], 10, &err);
		if(err || w > 255)
		{
			shell_error(shell, "w must be 0..255");
			return -EINVAL;
		}
	}

	rgbw_led_set((struct rgbw_color){ r, g, b, w });
	shell_print(shell, "LED: r=%u g=%u b=%u w=%u", r, g, b, w);
	return 0;
}

static int cmd_led_off(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	rgbw_led_off();
	shell_print(shell, "LED off");
	return 0;
}

static int cmd_led_get(const struct shell *shell, size_t argc, char **argv)
{
	struct rgbw_color color = rgbw_led_get();

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "LED: r=%u g=%u b=%u w=%u", color.r, color.g, color.b, color.w);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_led, SHELL_CMD_ARG(set, NULL, "Set LED color <r> <g> <b> [w] (0..255)", cmd_led_set, 4, 1),
    SHELL_CMD(off, NULL, "Turn LED off", cmd_led_off),
    SHELL_CMD(get, NULL, "Show current LED color", cmd_led_get), SHELL_SUBCMD_SET_END);
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_power, SHELL_CMD(reboot, NULL, "Reboot normally", cmd_reboot),
    SHELL_CMD(off, NULL, "Power off", cmd_power_off),
    SHELL_CMD(wake, NULL, "Show reset reason and GPIO latches captured at boot", cmd_power_wake),
    SHELL_CMD(reboot_isp, NULL, "Reboot into Valve bootloader ISP mode", cmd_reboot_isp),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_radio,
#if CONFIG_IBEX_BLE
    SHELL_CMD(ble, NULL, "Reboot into BLE personality", cmd_radio_ble),
    SHELL_CMD(ble_disconnect, NULL, "Disconnect/deactivate BLE without rebooting",
              cmd_radio_ble_disconnect),
    SHELL_CMD_ARG(ble_unpair, NULL, "Clear BLE bonds [identity]", cmd_radio_ble_unpair, 1, 1),
#endif
    SHELL_CMD(esb, NULL, "Reboot into ESB personality", cmd_radio_esb),
    SHELL_CMD_ARG(esb_bond, NULL, "Provision ESB bond <proteus_uuid> <ibex_uuid> [serial]",
                  cmd_radio_esb_bond, 3, 1),
    SHELL_CMD(esb_regs, NULL, "Dump ESB radio registers", cmd_radio_esb_regs),
    SHELL_CMD(status, NULL, "Show active radio personality", cmd_radio_status),
    SHELL_CMD_ARG(usb_debug, NULL, "Allow radio and suppress USB HID reports <on|off>",
                  cmd_radio_usb_debug, 2, 0),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_settings, SHELL_CMD_ARG(get, NULL, "Get Ibex runtime setting <id>", cmd_settings_get, 2, 0),
    SHELL_CMD_ARG(set, NULL, "Set Ibex runtime setting <id> <value>", cmd_settings_set, 3, 0),
    SHELL_CMD(defaults, NULL, "Reset Ibex runtime settings to defaults", cmd_settings_defaults),
    SHELL_SUBCMD_SET_END);

#if CONFIG_IBEX_OLYMPUS
SHELL_STATIC_SUBCMD_SET_CREATE(sub_olympus,
                               SHELL_CMD(status, NULL, "Show last Olympus raw decode snapshot",
                                         cmd_olympus_status),
                               SHELL_SUBCMD_SET_END);
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_steamctl, SHELL_CMD(battery, &sub_battery, "Battery and charger commands", NULL),
#if CONFIG_IBEX_RGBW_LED
    SHELL_CMD(led, &sub_led, "RGBW LED commands", NULL),
#endif
#if CONFIG_IBEX_OLYMPUS
    SHELL_CMD(olympus, &sub_olympus, "Olympus touchpad debug commands", NULL),
#endif
    SHELL_CMD(power, &sub_power, "Power and reboot commands", NULL),
    SHELL_CMD(radio, &sub_radio, "Radio personality commands", NULL),
    SHELL_CMD(settings, &sub_settings, "Ibex runtime settings registry", NULL),
    SHELL_CMD(status, NULL, "Show firmware status", cmd_status), SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(steamctl, &sub_steamctl, "SC26re commands", NULL);
