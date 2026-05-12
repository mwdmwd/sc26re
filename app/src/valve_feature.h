/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define VALVE_FEATURE_REPORT_ID 0x01
#define VALVE_FEATURE_REPORT_SIZE 64
enum valve_feature_link
{
	VALVE_FEATURE_LINK_BLE,
	VALVE_FEATURE_LINK_USB,
	VALVE_FEATURE_LINK_ESB,
};

ssize_t valve_feature_respond(enum valve_feature_link link, const uint8_t *request,
                              size_t request_len, uint8_t *response, size_t response_capacity);
