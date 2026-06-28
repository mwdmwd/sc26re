/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdint.h>

#include <zephyr/sys/util.h>

#define OLYMPUS_GLOBAL_BASE 0x51000000u
#define OLYMPUS_MEASUREMENT_BASE 0x51100000u
#define OLYMPUS_GROUP_BASE 0x51200000u
#define OLYMPUS_NOISE_BASE 0x51500000u
#define OLYMPUS_UNK516_BASE 0x51600000u
#define OLYMPUS_RESULT_MAP_BASE 0x51700000u
#define OLYMPUS_COMPENSATION_BASE 0x51800000u
#define OLYMPUS_UNK519_BASE 0x51900000u
#define OLYMPUS_RESULT_ROUTE_BASE 0x51a00000u
#define OLYMPUS_RESULT_ROUTE_LIMIT_BASE 0x51b00000u
#define OLYMPUS_RECORD_STRIDE 0x1000u

#define OLYMPUS_GROUP_REQUEST_CALIBRATION BIT(6)
#define OLYMPUS_GROUP_ENABLE_CALIBRATION BIT(7)

#define OLYMPUS_LE16(value) \
	{ (uint8_t)((uint16_t)(value) & 0xffu), (uint8_t)(((uint16_t)(value) >> 8) & 0xffu) }

struct olympus_config_block
{
	uint32_t base;
	uint16_t count;
	uint16_t record_size;
	const uint8_t *data;
};

/*
 * Cirque Olympus CustomMeas profile recovered from the original
 * controller firmware image and observed register writes.
 *
 * The record addresses, sizes, and values are recovered device data.
 * Semantic labels for the global, group, measurement, and noise records
 * follow Cirque's public CustomMeas terminology. Unknown records and
 * fields are kept as unk_*.
 */
struct olympus_global_config
{
	uint8_t enable;
	uint8_t frame_period_ms_le[2];
	uint8_t persist;
	uint8_t restore;
	uint8_t power_on_enable;
	uint8_t low_power_mode;
};

struct olympus_group_config
{
	uint8_t mode;
	uint8_t calibration_flags;
	uint8_t frames_between_compensations_le[2];
	uint8_t negative_threshold_le[2];
	uint8_t speed_threshold_le[2];
	uint8_t activity_threshold_le[2];
	uint8_t activity_timeout_le[2];
	uint8_t unk_0c[14];
};

struct olympus_measurement_config
{
	uint8_t control;
	uint8_t electrode_states[24];
	uint8_t gain;
	uint8_t global_offset;
	uint8_t channel_offset_multiplier;
	uint8_t channel_offsets[8];
	uint8_t toggle_frequency;
	uint8_t aperture_length;
	uint8_t waveform;
	uint8_t adc_channel_mask_le[2];
};

struct olympus_noise_config
{
	uint8_t alternate_frequency_index;
	uint8_t noise_change_threshold_le[2];
	uint8_t max_adjust_per_frame_le[2];
	uint8_t offset_adjust_filter_weight;
	uint8_t clean_count_threshold;
	uint8_t unk_07[2];
};

struct olympus_unk516_config
{
	uint8_t unk_00[7];
};

struct olympus_channel_reference
{
	uint8_t measurement;
	uint8_t reading;
};

struct olympus_result_map
{
	uint8_t flags;
	uint8_t x_count;
	uint8_t y_count;
	uint8_t reserved;
	uint8_t x_scale_le[2];
	uint8_t y_scale_le[2];
	struct olympus_channel_reference channels[16];
	uint8_t unk_28[32];
	uint8_t primary_scale_q8_8_le[16][2];
	uint8_t secondary_scale_q8_8_le[16][2];
};

struct olympus_compensation_slot
{
	uint8_t unk[6];
};

struct olympus_unk519_config
{
	uint8_t value0_le[2];
	uint8_t value1_le[2];
};

struct olympus_result_route
{
	uint8_t route_type;
	uint8_t measurement;
	uint8_t first_result;
	uint8_t last_result;
	uint8_t threshold_le[2];
};

struct olympus_u16_config
{
	uint8_t value_le[2];
};

static const struct olympus_global_config olympus_active_config = {
	.enable = 1,
	.frame_period_ms_le = OLYMPUS_LE16(4),
	.persist = 0,
	.restore = 0,
	.power_on_enable = 1,
	.low_power_mode = 1,
};

static const struct olympus_group_config olympus_group_configs[] = {
	{
	    .mode = 0x00,
	    .calibration_flags = 0x81,
	    .frames_between_compensations_le = OLYMPUS_LE16(500),
	    .negative_threshold_le = OLYMPUS_LE16(-75),
	    .speed_threshold_le = OLYMPUS_LE16(300),
	    .activity_threshold_le = OLYMPUS_LE16(500),
	    .activity_timeout_le = OLYMPUS_LE16(150),
	    .unk_0c = { 0x01, 0x00, 0x08, 0x00, 0x80, 0x00, 0x20, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
	                0x00 },
	},
	{
	    .mode = 0x00,
	    .calibration_flags = 0x81,
	    .frames_between_compensations_le = OLYMPUS_LE16(500),
	    .negative_threshold_le = OLYMPUS_LE16(-75),
	    .speed_threshold_le = OLYMPUS_LE16(300),
	    .activity_threshold_le = OLYMPUS_LE16(600),
	    .activity_timeout_le = OLYMPUS_LE16(150),
	    .unk_0c = { 0x01, 0x00, 0x08, 0x00, 0x80, 0x00, 0x20, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	                0x00 },
	},
	{
	    .mode = 0x01,
	    .calibration_flags = 0x81,
	    .frames_between_compensations_le = OLYMPUS_LE16(48),
	    .negative_threshold_le = OLYMPUS_LE16(100),
	    .speed_threshold_le = OLYMPUS_LE16(200),
	    .activity_threshold_le = OLYMPUS_LE16(300),
	    .activity_timeout_le = OLYMPUS_LE16(0),
	    .unk_0c = { 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x0a,
	                0x00 },
	},
	{
	    .mode = 0x01,
	    .calibration_flags = 0x81,
	    .frames_between_compensations_le = OLYMPUS_LE16(500),
	    .negative_threshold_le = OLYMPUS_LE16(-75),
	    .speed_threshold_le = OLYMPUS_LE16(300),
	    .activity_threshold_le = OLYMPUS_LE16(600),
	    .activity_timeout_le = OLYMPUS_LE16(150),
	    .unk_0c = { 0x01, 0x00, 0x08, 0x00, 0x80, 0x00, 0x20, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	                0x00 },
	},
	{
	    .mode = 0x00,
	    .calibration_flags = 0x81,
	    .frames_between_compensations_le = OLYMPUS_LE16(750),
	    .negative_threshold_le = OLYMPUS_LE16(-32767),
	    .speed_threshold_le = OLYMPUS_LE16(300),
	    .activity_threshold_le = OLYMPUS_LE16(600),
	    .activity_timeout_le = OLYMPUS_LE16(150),
	    .unk_0c = { 0x01, 0x00, 0x08, 0x00, 0x80, 0x00, 0x20, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
	                0x00 },
	},
};

static const struct olympus_measurement_config olympus_measurement_configs[] = {
	{
	    .control = 0x80,
	    .electrode_states = { 0x33, 0x33, 0x33, 0x33, 0x77, 0x77, 0x77, 0x77 },
	    .gain = 0x09,
	    .global_offset = 0x2d,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x12,
	    .aperture_length = 0x04,
	    .adc_channel_mask_le = { 0x00, 0xff },
	},
	{
	    .control = 0x80,
	    .electrode_states = { 0x77, 0x77, 0x77, 0x77, 0x33, 0x33, 0x33, 0x33 },
	    .gain = 0x09,
	    .global_offset = 0x2d,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x12,
	    .aperture_length = 0x04,
	    .adc_channel_mask_le = { 0xff, 0x00 },
	},
	{
	    .control = 0x80,
	    .electrode_states = { [8] = 0x33, 0x33, 0x33, 0x33, 0x77, 0x77, 0x77, 0x77 },
	    .gain = 0x09,
	    .global_offset = 0x2d,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x12,
	    .aperture_length = 0x04,
	    .adc_channel_mask_le = { 0x00, 0xff },
	},
	{
	    .control = 0x80,
	    .electrode_states = { [8] = 0x77, 0x77, 0x77, 0x77, 0x33, 0x33, 0x33, 0x33 },
	    .gain = 0x09,
	    .global_offset = 0x2d,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x12,
	    .aperture_length = 0x04,
	    .adc_channel_mask_le = { 0xff, 0x00 },
	},
	{
	    .control = 0x81,
	    .electrode_states = { [16] = 0x34, 0x49, 0x00, 0x00, 0x34, 0x49 },
	    .gain = 0x0a,
	    .global_offset = 0xa3,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x12,
	    .aperture_length = 0x09,
	    .adc_channel_mask_le = { 0x04, 0x04 },
	},
	{
	    .control = 0x82,
	    .electrode_states = { [18] = 0x74, 0x47 },
	    .gain = 0x08,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x11,
	    .aperture_length = 0x0a,
	    .adc_channel_mask_le = { 0x60, 0x00 },
	},
	{
	    .control = 0x83,
	    .electrode_states = { [22] = 0x74, 0x47 },
	    .gain = 0x04,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x11,
	    .aperture_length = 0x27,
	    .adc_channel_mask_le = { 0x00, 0x60 },
	},
	{
	    .control = 0x84,
	    .electrode_states = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
	                          0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
	                          0x00, 0x00, 0x70, 0x07, 0x00, 0x00, 0x70, 0x07 },
	    .gain = 0x04,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x12,
	    .aperture_length = 0x27,
	},
	{
	    .control = 0x84,
	    .electrode_states = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
	                          0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77,
	                          0x00, 0x00, 0x70, 0x07, 0x00, 0x00, 0x70, 0x07 },
	    .gain = 0x04,
	    .channel_offset_multiplier = 0x01,
	    .toggle_frequency = 0x11,
	    .aperture_length = 0x27,
	},
};

static const struct olympus_noise_config olympus_noise_configs[] = {
	{
	    .alternate_frequency_index = 0x11,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
	{
	    .alternate_frequency_index = 0x11,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
	{
	    .alternate_frequency_index = 0x11,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
	{
	    .alternate_frequency_index = 0x11,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
	{
	    .alternate_frequency_index = 0x11,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
	{
	    .alternate_frequency_index = 0x12,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	    .unk_07 = { 0xee, 0x16 },
	},
	{
	    .alternate_frequency_index = 0x12,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(200),
	    .offset_adjust_filter_weight = 0x02,
	    .clean_count_threshold = 0x0a,
	    .unk_07 = { 0x99, 0x19 },
	},
	{
	    .alternate_frequency_index = 0xff,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
	{
	    .alternate_frequency_index = 0xff,
	    .noise_change_threshold_le = OLYMPUS_LE16(500),
	    .max_adjust_per_frame_le = OLYMPUS_LE16(20),
	    .offset_adjust_filter_weight = 0x19,
	    .clean_count_threshold = 0x0a,
	},
};

static const struct olympus_unk516_config olympus_unk516_configs[] = {
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
	{ { 0xff, 0x0a, 0x32, 0x00, 0x2c, 0x01, 0x1f } },
};

static const struct olympus_result_map olympus_result_maps[] = {
	{
		.x_count = 8,
		.y_count = 8,
		.x_scale_le = OLYMPUS_LE16(0x05dc),
		.y_scale_le = OLYMPUS_LE16(0x05dc),
		.channels = {
			{0x00, 0x00},
			{0x00, 0x01},
			{0x00, 0x02},
			{0x00, 0x03},
			{0x00, 0x04},
			{0x00, 0x05},
			{0x00, 0x06},
			{0x00, 0x07},
			{0x01, 0x08},
			{0x01, 0x09},
			{0x01, 0x0a},
			{0x01, 0x0b},
			{0x01, 0x0c},
			{0x01, 0x0d},
			{0x01, 0x0e},
			{0x01, 0x0f},
		},
		.primary_scale_q8_8_le = {
			OLYMPUS_LE16(317), OLYMPUS_LE16(296), OLYMPUS_LE16(298), OLYMPUS_LE16(297),
			OLYMPUS_LE16(295), OLYMPUS_LE16(305), OLYMPUS_LE16(321), OLYMPUS_LE16(338),
			OLYMPUS_LE16(322), OLYMPUS_LE16(314), OLYMPUS_LE16(297), OLYMPUS_LE16(295),
			OLYMPUS_LE16(297), OLYMPUS_LE16(303), OLYMPUS_LE16(305), OLYMPUS_LE16(305),
		},
		.secondary_scale_q8_8_le = {
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
		},
	},
	{
		.x_count = 8,
		.y_count = 8,
		.x_scale_le = OLYMPUS_LE16(0x05dc),
		.y_scale_le = OLYMPUS_LE16(0x05dc),
		.channels = {
			{0x00, 0x08},
			{0x00, 0x09},
			{0x00, 0x0a},
			{0x00, 0x0b},
			{0x00, 0x0c},
			{0x00, 0x0d},
			{0x00, 0x0e},
			{0x00, 0x0f},
			{0x01, 0x07},
			{0x01, 0x06},
			{0x01, 0x05},
			{0x01, 0x04},
			{0x01, 0x03},
			{0x01, 0x02},
			{0x01, 0x01},
			{0x01, 0x00},
		},
		.primary_scale_q8_8_le = {
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
		},
		.secondary_scale_q8_8_le = {
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
			OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256), OLYMPUS_LE16(256),
		},
	},
};

static const struct olympus_compensation_slot olympus_compensation_slots[16];

static const struct olympus_unk519_config olympus_unk519_configs[] = {
	{
	    .value0_le = OLYMPUS_LE16(500),
	    .value1_le = OLYMPUS_LE16(20),
	},
};

static const struct olympus_result_route olympus_result_routes[20] = {
	{ .route_type = 0x03,
	  .measurement = 0,
	  .first_result = 8,
	  .last_result = 15,
	  .threshold_le = OLYMPUS_LE16(200) },
	{ .route_type = 0x03,
	  .measurement = 1,
	  .first_result = 0,
	  .last_result = 7,
	  .threshold_le = OLYMPUS_LE16(200) },
	{ .route_type = 0x03,
	  .measurement = 2,
	  .first_result = 8,
	  .last_result = 15,
	  .threshold_le = OLYMPUS_LE16(300) },
	{ .route_type = 0x03,
	  .measurement = 3,
	  .first_result = 0,
	  .last_result = 7,
	  .threshold_le = OLYMPUS_LE16(300) },
	{ .route_type = 0x01,
	  .measurement = 4,
	  .first_result = 2,
	  .last_result = 2,
	  .threshold_le = OLYMPUS_LE16(300) },
	{ .route_type = 0x01,
	  .measurement = 4,
	  .first_result = 10,
	  .last_result = 10,
	  .threshold_le = OLYMPUS_LE16(300) },
	{ .route_type = 0x01,
	  .measurement = 5,
	  .first_result = 5,
	  .last_result = 6,
	  .threshold_le = OLYMPUS_LE16(300) },
	{ .route_type = 0x01,
	  .measurement = 6,
	  .first_result = 13,
	  .last_result = 14,
	  .threshold_le = OLYMPUS_LE16(300) },
	{ .route_type = 0x05,
	  .measurement = 0,
	  .first_result = 8,
	  .last_result = 15,
	  .threshold_le = OLYMPUS_LE16(200) },
	{ .route_type = 0x05,
	  .measurement = 1,
	  .first_result = 0,
	  .last_result = 7,
	  .threshold_le = OLYMPUS_LE16(200) },
	/* Remaining route entries are zero. */
};

static const struct olympus_u16_config olympus_result_route_limits[] = {
	{
	    .value_le = OLYMPUS_LE16(20),
	},
};

#define OLYMPUS_CONFIG_BLOCK(base_, records_) \
	{ \
		.base = (base_), \
		.count = ARRAY_SIZE(records_), \
		.record_size = sizeof((records_)[0]), \
		.data = (const uint8_t *)(records_), \
	}

static const struct olympus_config_block olympus_config_blocks[] = {
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_GROUP_BASE, olympus_group_configs),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_MEASUREMENT_BASE, olympus_measurement_configs),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_NOISE_BASE, olympus_noise_configs),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_UNK516_BASE, olympus_unk516_configs),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_RESULT_MAP_BASE, olympus_result_maps),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_COMPENSATION_BASE, olympus_compensation_slots),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_UNK519_BASE, olympus_unk519_configs),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_RESULT_ROUTE_BASE, olympus_result_routes),
	OLYMPUS_CONFIG_BLOCK(OLYMPUS_RESULT_ROUTE_LIMIT_BASE, olympus_result_route_limits),
};

#undef OLYMPUS_CONFIG_BLOCK
#undef OLYMPUS_LE16
