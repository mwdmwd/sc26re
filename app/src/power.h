/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdint.h>

int power_reboot_normal(void);
int power_reboot_to_valve_isp(void);
void power_capture_boot_state(void);
uint32_t power_boot_reset_reason(void);
void power_boot_gpio_latches(uint32_t latches[2]);
void power_off(void);
void power_off_after_buttons_released(uint32_t release_mask);
