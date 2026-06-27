/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct valve_esb_backend_debug;

enum controller_button
{
	CONTROLLER_BUTTON_A = 0,
	CONTROLLER_BUTTON_B = 1,
	CONTROLLER_BUTTON_X = 2,
	CONTROLLER_BUTTON_Y = 3,
	CONTROLLER_BUTTON_QAM = 4,
	CONTROLLER_BUTTON_RIGHT_STICK = 5,
	CONTROLLER_BUTTON_VIEW = 6,
	CONTROLLER_BUTTON_RIGHT_PADDLE1 = 7,
	CONTROLLER_BUTTON_RIGHT_PADDLE2 = 8,
	CONTROLLER_BUTTON_RIGHT_SHOULDER = 9,
	CONTROLLER_BUTTON_DPAD_DOWN = 10,
	CONTROLLER_BUTTON_DPAD_RIGHT = 11,
	CONTROLLER_BUTTON_DPAD_LEFT = 12,
	CONTROLLER_BUTTON_DPAD_UP = 13,
	CONTROLLER_BUTTON_MENU = 14,
	CONTROLLER_BUTTON_LEFT_STICK = 15,
	CONTROLLER_BUTTON_STEAM = 16,
	CONTROLLER_BUTTON_LEFT_PADDLE1 = 17,
	CONTROLLER_BUTTON_LEFT_PADDLE2 = 18,
	CONTROLLER_BUTTON_LEFT_SHOULDER = 19,
	CONTROLLER_BUTTON_RIGHT_STICK_TOUCH = 20,
	CONTROLLER_BUTTON_RIGHT_TOUCHPAD_TOUCH = 21,
	CONTROLLER_BUTTON_RIGHT_TOUCHPAD_CLICK = 22,
	CONTROLLER_BUTTON_RIGHT_TRIGGER_CLICK = 23,
	CONTROLLER_BUTTON_LEFT_STICK_TOUCH = 24,
	CONTROLLER_BUTTON_LEFT_TOUCHPAD_TOUCH = 25,
	CONTROLLER_BUTTON_LEFT_TOUCHPAD_CLICK = 26,
	CONTROLLER_BUTTON_LEFT_TRIGGER_CLICK = 27,
	CONTROLLER_BUTTON_RIGHT_GRIP_TOUCH = 28,
	CONTROLLER_BUTTON_LEFT_GRIP_TOUCH = 29,
};

struct controller_report
{
	uint32_t buttons;
	int16_t trigger_left;
	int16_t trigger_right;
	int16_t stick_left_x;
	int16_t stick_left_y;
	int16_t stick_right_x;
	int16_t stick_right_y;
	int16_t touchpad_left_x;
	int16_t touchpad_left_y;
	uint16_t touchpad_left_pressure;
	int16_t touchpad_right_x;
	int16_t touchpad_right_y;
	uint16_t touchpad_right_pressure;
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
} __packed;

enum controller_charge_state
{
	CONTROLLER_CHARGE_STATE_RESET = 0,
	CONTROLLER_CHARGE_STATE_DISCHARGING = 1,
	CONTROLLER_CHARGE_STATE_CHARGING = 2,
	CONTROLLER_CHARGE_STATE_SOURCE_VALIDATE = 3,
	CONTROLLER_CHARGE_STATE_CHARGING_DONE = 4,
};

struct controller_battery_report
{
	uint8_t charge_state;
	uint8_t level_percent;
	uint16_t battery_mv;
	uint16_t system_mv;
	uint16_t input_mv;
	uint16_t current_ma;
	uint16_t input_current_ma;
	uint16_t temperature_c;
	uint8_t charger_type;
	bool charge_complete;
	bool valid;
} __packed;

enum radio_personality
{
	RADIO_PERSONALITY_BLE,
	RADIO_PERSONALITY_ESB,
};

int hardware_init(void);
void hardware_read_report(struct controller_report *report);
void hardware_wait_for_change(void);
void hardware_signal_input_changed(void);
uint32_t hardware_read_buttons(void);
bool hardware_pairing_chord_pressed(void);
void hardware_haptic_pulse(void);
int hardware_haptic_tone(uint32_t frequency_hz, uint32_t duration_ms);

int transport_init(void);
int transport_send(const struct controller_report *report);
int transport_send_battery_status(const struct controller_battery_report *report);

enum transport_usb_radio_mode
{
	TRANSPORT_USB_RADIO_OFF,
	TRANSPORT_USB_RADIO_VBUS_CHARGE,
	TRANSPORT_USB_RADIO_DIAGNOSTIC,
};

#define TRANSPORT_BLE_ID_ALL UINT8_MAX
int transport_ble_init(void);
int transport_ble_send(const struct controller_report *report);
int transport_ble_send_battery_status(const struct controller_battery_report *report);
void transport_ble_deactivate(void);
int transport_ble_clear_bonds(uint8_t id);

bool transport_usb_attached(void);
bool transport_usb_configured(void);
int transport_set_usb_radio_mode(enum transport_usb_radio_mode mode);
bool transport_usb_radio_allowed(void);
const char *transport_usb_radio_mode_name(void);
void transport_enter_usb_mode(void);
int transport_usb_init(void);
int transport_usb_send(const struct controller_report *report);
int transport_usb_send_battery_status(const struct controller_battery_report *report);

int transport_esb_init(void);
int transport_esb_send(const struct controller_report *report);
int transport_esb_send_battery_status(const struct controller_battery_report *report);
void transport_esb_deactivate(void);
bool transport_esb_connected(void);
uint8_t transport_esb_channel(void);
uint32_t transport_esb_host_frames_received(void);
uint32_t transport_esb_channel_maps_received(void);
uint32_t transport_esb_awake_frames_received(void);
uint32_t transport_esb_polls_received(void);
uint32_t transport_esb_replies_sent(void);
uint32_t transport_esb_backend_end_events(void);
uint32_t transport_esb_backend_crc_ok_events(void);
uint32_t transport_esb_backend_crc_bad_events(void);
uint32_t transport_esb_backend_rx_dropped_events(void);
bool transport_esb_bond_loaded(void);
int transport_esb_provision_bond(uint32_t proteus_uuid, uint32_t ibex_uuid, const char *serial);
int transport_esb_get_debug(struct valve_esb_backend_debug *debug);

void radio_personality_init(void);
enum radio_personality radio_personality_get(void);
const char *radio_personality_name(void);
void radio_personality_reboot_into(enum radio_personality personality);
void radio_personality_reboot_into_after(enum radio_personality personality, uint32_t delay_ms);
void radio_personality_remember(enum radio_personality personality);
void radio_personality_persist_after(enum radio_personality personality, uint32_t delay_ms);
void radio_personality_cancel_pending_persist(void);
