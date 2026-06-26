/* SPDX-License-Identifier: AGPL-3.0-or-later */

#pragma once

enum charge_mode_result
{
	CHARGE_MODE_SKIPPED,
	CHARGE_MODE_USB_CONFIGURED,
	CHARGE_MODE_POWER_ON_RADIO,
};

enum charge_mode_result charge_mode_run_if_needed(void);
