/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "ibex_settings_registry.h"
#include "olympus.h"
#include "olympus_config.h"

LOG_MODULE_REGISTER(olympus);

#define OLYMPUS_NODE DT_NODELABEL(olympus)
#define OLYMPUS_SENSOR_PAYLOAD_SIZE 76
#define OLYMPUS_MAX_PAYLOAD_SIZE 200
#define OLYMPUS_ELECTRODE_BASELINE 40
#define OLYMPUS_TOUCH_THRESHOLD 1200
#define OLYMPUS_RELEASE_THRESHOLD 1000
#define OLYMPUS_NOISE_THRESHOLD 250
#define OLYMPUS_THRESHOLD_SCALE_DIVISOR 5
#define OLYMPUS_CLICK_THRESHOLD 500
#define OLYMPUS_CLICK_RELEASE_THRESHOLD 400
#define OLYMPUS_PAD_CLICK_PRESSURE 500
#define OLYMPUS_PAD_CLICK_PRESSURE_SCALE 10
#define OLYMPUS_PRESSURE_SCALE 32
#define OLYMPUS_PRESSURE_MAX INT16_MAX
#define OLYMPUS_DESCRIPTOR_ADDRESS 0x20
#define OLYMPUS_DESCRIPTOR_SIZE 30
#define OLYMPUS_VENDOR_ID 0x0488
#define OLYMPUS_PRODUCT_ID_A 0xd0c1
#define OLYMPUS_PRODUCT_ID_B 0x1015

static const struct i2c_dt_spec olympus_i2c = I2C_DT_SPEC_GET(OLYMPUS_NODE);
static const struct gpio_dt_spec olympus_irq = GPIO_DT_SPEC_GET(OLYMPUS_NODE, int_gpios);
static const struct gpio_dt_spec olympus_reset = GPIO_DT_SPEC_GET(OLYMPUS_NODE, reset_gpios);

static struct gpio_callback olympus_irq_callback;
static struct k_sem olympus_irq_sem;
static struct k_mutex olympus_report_mutex;
static struct controller_report olympus_report;
static struct olympus_debug_snapshot olympus_debug;
static uint32_t olympus_frame_count;
static struct k_thread olympus_thread;
static K_THREAD_STACK_DEFINE(olympus_thread_stack, 2048);

struct olympus_pad_state
{
	bool touched;
	bool stick_touched;
	bool pad_clicked;
	bool click_filter_initialized;
	uint8_t click_sample_index;
	int16_t click_samples[2];
	float filtered_x;
	float filtered_y;
	int32_t filtered_touch_pressure;
};

struct olympus_axis_calibration
{
	float x_offset;
	float x_scale;
	float y_offset;
	float y_scale;
};

static struct olympus_pad_state left_pad;
static struct olympus_pad_state right_pad;

static const struct olympus_axis_calibration left_axis_cal = {
	.x_offset = -0.01f,
	.x_scale = 0.90f,
	.y_offset = 0.05f,
	.y_scale = 0.91f,
};

static const struct olympus_axis_calibration right_axis_cal = {
	.x_offset = 0.11f,
	.x_scale = 0.88f,
	.y_offset = 0.05f,
	.y_scale = 0.91f,
};

static int16_t registry_setting_or_default(uint8_t id, int16_t fallback)
{
	int16_t value;

	return ibex_setting_get(id, &value) ? value : fallback;
}

static int32_t scaled_setting_or_default(uint8_t id, int16_t fallback)
{
	int32_t scaled = MAX((int32_t)registry_setting_or_default(id, fallback), 0) /
	                 OLYMPUS_THRESHOLD_SCALE_DIVISOR;

	return MAX(scaled, fallback);
}

static int32_t pad_click_threshold(bool right)
{
	uint8_t id = right ? IBEX_SETTING_RIGHT_TRACKPAD_CLICK_PRESSURE
	                   : IBEX_SETTING_LEFT_TRACKPAD_CLICK_PRESSURE;

	return MAX(MAX((int32_t)registry_setting_or_default(id, OLYMPUS_PAD_CLICK_PRESSURE), 0) *
	               OLYMPUS_PAD_CLICK_PRESSURE_SCALE,
	           OLYMPUS_PAD_CLICK_PRESSURE);
}

BUILD_ASSERT(sizeof(struct olympus_global_config) == 7);
BUILD_ASSERT(sizeof(struct olympus_group_config) == 26);
BUILD_ASSERT(sizeof(struct olympus_measurement_config) == 41);
BUILD_ASSERT(sizeof(struct olympus_noise_config) == 9);
BUILD_ASSERT(sizeof(struct olympus_unk516_config) == 7);
BUILD_ASSERT(sizeof(struct olympus_channel_reference) == 2);
BUILD_ASSERT(sizeof(struct olympus_result_map) == 136);
BUILD_ASSERT(sizeof(struct olympus_compensation_slot) == 6);
BUILD_ASSERT(sizeof(struct olympus_unk519_config) == 4);
BUILD_ASSERT(sizeof(struct olympus_result_route) == 6);
BUILD_ASSERT(sizeof(struct olympus_u16_config) == 2);
BUILD_ASSERT(sizeof(olympus_group_configs) == 5 * 26);
BUILD_ASSERT(sizeof(olympus_measurement_configs) == 9 * 41);
BUILD_ASSERT(sizeof(olympus_noise_configs) == 9 * 9);
BUILD_ASSERT(sizeof(olympus_unk516_configs) == 9 * 7);
BUILD_ASSERT(sizeof(olympus_result_maps) == 2 * 136);
BUILD_ASSERT(sizeof(olympus_compensation_slots) == 16 * 6);
BUILD_ASSERT(sizeof(olympus_unk519_configs) == 1 * 4);
BUILD_ASSERT(sizeof(olympus_result_routes) == 20 * 6);
BUILD_ASSERT(sizeof(olympus_result_route_limits) == 1 * 2);

static uint8_t checksum(const uint8_t *data, size_t size)
{
	uint8_t sum = 0;

	for(size_t i = 0; i < size; ++i)
	{
		sum += data[i];
	}
	return sum;
}

static int olympus_write_register(uint32_t address, const uint8_t *data, uint8_t size)
{
	uint8_t packet[9 + OLYMPUS_MAX_PAYLOAD_SIZE] = {
		[1] = 0x09,
	};

	if(size > OLYMPUS_MAX_PAYLOAD_SIZE)
	{
		return -E2BIG;
	}

	sys_put_le32(address, &packet[2]);
	packet[6] = size;
	memcpy(&packet[8], data, size);
	packet[8 + size] = checksum(packet, 8 + size);

	/* The factory driver leaves roughly two milliseconds between commands. */
	k_sleep(K_MSEC(2));
	return i2c_write_dt(&olympus_i2c, packet, size + 9);
}

static int olympus_read_register(uint32_t address, uint8_t *data, uint8_t size)
{
	uint8_t request[8] = {
		[0] = 0x01,
		[1] = 0x09,
	};
	uint8_t response[3 + OLYMPUS_MAX_PAYLOAD_SIZE];
	int err;

	if(size > OLYMPUS_MAX_PAYLOAD_SIZE)
	{
		return -E2BIG;
	}

	sys_put_le32(address, &request[2]);
	request[6] = size;
	k_sleep(K_MSEC(2));

	err = i2c_write_read_dt(&olympus_i2c, request, sizeof(request), response, size + 3);
	if(err)
	{
		return err;
	}
	if(sys_get_le16(response) != size + 3 || checksum(response, size + 2) != response[size + 2])
	{
		return -EBADMSG;
	}

	memcpy(data, &response[2], size);
	return 0;
}

static int olympus_apply_factory_config(void)
{
	struct olympus_global_config activation;
	struct olympus_group_config group_config;
	int err;

	/*
	 * The normal factory path disables an already-running device before
	 * replacing its configuration.
	 */
	err = olympus_read_register(OLYMPUS_GLOBAL_BASE, (uint8_t *)&activation, sizeof(activation));
	if(err)
	{
		return err;
	}
	activation.enable = 0;
	err = olympus_write_register(OLYMPUS_GLOBAL_BASE, (const uint8_t *)&activation,
	                             sizeof(activation));
	if(err)
	{
		return err;
	}

	for(size_t block_index = 0; block_index < ARRAY_SIZE(olympus_config_blocks); ++block_index)
	{
		const struct olympus_config_block *block = &olympus_config_blocks[block_index];

		for(uint16_t record = 0; record < block->count; ++record)
		{
			err = olympus_write_register(block->base + (uint32_t)record * OLYMPUS_RECORD_STRIDE,
			                             &block->data[record * block->record_size],
			                             block->record_size);

			if(err)
			{
				LOG_ERR("config block %u record %u failed: %d", (unsigned int)block_index, record,
				        err);
				return err;
			}
		}
	}

	/*
	 * Valve's normal-mode post-pass reads each group config back and sets
	 * the request-calibration flag before starting measurements.
	 */
	for(uint32_t record = 0; record < ARRAY_SIZE(olympus_group_configs); ++record)
	{
		uint32_t address = OLYMPUS_GROUP_BASE + record * OLYMPUS_RECORD_STRIDE;

		err = olympus_read_register(address, (uint8_t *)&group_config, sizeof(group_config));
		if(err)
		{
			return err;
		}
		group_config.calibration_flags |= OLYMPUS_GROUP_REQUEST_CALIBRATION;
		err = olympus_write_register(address, (const uint8_t *)&group_config, sizeof(group_config));
		if(err)
		{
			return err;
		}
	}

	return olympus_write_register(OLYMPUS_GLOBAL_BASE, (const uint8_t *)&olympus_active_config,
	                              sizeof(olympus_active_config));
}

static int olympus_check_descriptor(void)
{
	uint8_t address[2];
	uint8_t descriptor[OLYMPUS_DESCRIPTOR_SIZE];
	uint16_t product;
	int err;

	sys_put_le16(OLYMPUS_DESCRIPTOR_ADDRESS, address);
	err = i2c_write_read_dt(&olympus_i2c, address, sizeof(address), descriptor, sizeof(descriptor));
	if(err)
	{
		return err;
	}

	product = sys_get_le16(&descriptor[22]);
	if(sys_get_le16(descriptor) != OLYMPUS_DESCRIPTOR_SIZE ||
	   sys_get_le16(&descriptor[20]) != OLYMPUS_VENDOR_ID ||
	   (product != OLYMPUS_PRODUCT_ID_A && product != OLYMPUS_PRODUCT_ID_B))
	{
		LOG_ERR("unexpected Olympus descriptor: len=%u vendor=%04x product=%04x",
		        sys_get_le16(descriptor), sys_get_le16(&descriptor[20]), product);
		return -ENODEV;
	}

	LOG_INF("Olympus descriptor: vendor=%04x product=%04x", OLYMPUS_VENDOR_ID, product);
	return 0;
}

static float electrode_centroid(const int16_t *samples, bool reverse, int32_t *peak)
{
	int32_t weighted = 0;
	int32_t total = 0;

	*peak = 0;
	for(int i = 0; i < 8; ++i)
	{
		int index = reverse ? 7 - i : i;
		int32_t value = MAX((int32_t)samples[index] - OLYMPUS_ELECTRODE_BASELINE, 0);

		weighted += i * value;
		total += value;
		*peak = MAX(*peak, value);
	}

	if(total == 0)
	{
		return 0.5f;
	}

	return ((float)weighted / (float)total) / 7.0f;
}

static float expand_edge_position(float normalized)
{
	float distance = fabsf(normalized - 0.5f);

	if(distance > 0.35f)
	{
		float adjustment = (distance - 0.35f) * 7.0f * (distance - 0.35f);

		normalized += normalized > 0.5f ? adjustment : -adjustment;
	}

	return normalized;
}

static float lerp_clamped(float x, float x0, float x1, float y0, float y1)
{
	if(x <= x0)
	{
		return y0;
	}
	if(x >= x1)
	{
		return y1;
	}

	return y0 + ((x - x0) * (y1 - y0)) / (x1 - x0);
}

static int32_t scale_touch_pressure_at_edge(int32_t peak, float normalized)
{
	float distance = fabsf(normalized - 0.5f);
	float scale;

	if(distance <= 0.35f)
	{
		return peak;
	}

	scale = lerp_clamped(distance, 0.35f, 0.5f, 1.0f, 3.0f);
	return (int32_t)((float)peak * scale);
}

static void apply_axis_calibration(bool right, float *normalized_x, float *normalized_y)
{
	const struct olympus_axis_calibration *cal = right ? &right_axis_cal : &left_axis_cal;

	*normalized_x = (*normalized_x - cal->x_offset) / cal->x_scale;
	*normalized_y = (*normalized_y - cal->y_offset) / cal->y_scale;
}

static int16_t normalized_to_axis(float normalized)
{
	return CLAMP((int32_t)((normalized - 0.5f) * 65532.0f), INT16_MIN, INT16_MAX);
}

static uint16_t scale_pad_pressure(int32_t raw)
{
	return CLAMP(raw * OLYMPUS_PRESSURE_SCALE, 0, OLYMPUS_PRESSURE_MAX);
}

static void decode_pad(struct olympus_pad_state *state, const int16_t *x_samples,
                       const int16_t *y_samples, int16_t raw_pressure, int16_t raw_click,
                       bool right, int16_t *x, int16_t *y, uint16_t *pressure, bool *touched,
                       bool *stick_touched, bool *clicked, struct olympus_pad_debug *debug)
{
	int32_t peak_x;
	int32_t peak_y;
	int32_t peak;
	int32_t pressure_value;
	int32_t touch_pressure;
	int32_t click_value;
	int32_t noise_threshold;
	int32_t touch_threshold;
	int32_t release_threshold;
	int32_t pressure_click_threshold;
	float normalized_x;
	float normalized_y;

	noise_threshold =
	    scaled_setting_or_default(IBEX_SETTING_TRACKPAD_NOISE_THRESHOLD, OLYMPUS_NOISE_THRESHOLD);
	touch_threshold =
	    scaled_setting_or_default(IBEX_SETTING_TIMP_TOUCH_THRESHOLD_ON, OLYMPUS_TOUCH_THRESHOLD);
	release_threshold =
	    scaled_setting_or_default(IBEX_SETTING_TIMP_TOUCH_THRESHOLD_OFF, OLYMPUS_RELEASE_THRESHOLD);
	pressure_click_threshold = pad_click_threshold(right);

	normalized_x = expand_edge_position(electrode_centroid(x_samples, true, &peak_x));
	normalized_y = expand_edge_position(electrode_centroid(y_samples, true, &peak_y));
	apply_axis_calibration(right, &normalized_x, &normalized_y);
	peak = MAX(peak_x, peak_y);
	touch_pressure = MAX(scale_touch_pressure_at_edge(peak_x, normalized_x),
	                     scale_touch_pressure_at_edge(peak_y, normalized_y));

	pressure_value = raw_pressure > 0 ? raw_pressure : 0;

	if(!state->touched && touch_threshold > 3 * peak)
	{
		normalized_x = 0.5f;
		normalized_y = 0.5f;
	}
	else if(state->touched)
	{
		int32_t filtered = (7 * state->filtered_touch_pressure + touch_pressure) / 8;
		float blend = 1.0f;

		state->filtered_touch_pressure = filtered;
		if(touch_pressure > release_threshold &&
		   release_threshold < filtered - 200 &&
		   touch_pressure < filtered - 200)
		{
			blend = lerp_clamped((float)touch_pressure, (float)release_threshold,
			                     (float)(filtered - 200), 0.2f, 1.0f);
		}

		normalized_x = state->filtered_x + (normalized_x - state->filtered_x) * blend;
		normalized_y = state->filtered_y + (normalized_y - state->filtered_y) * blend;
		state->filtered_x = normalized_x;
		state->filtered_y = normalized_y;
		state->touched = touch_pressure > release_threshold;
	}
	else
	{
		state->filtered_touch_pressure = touch_pressure;
		state->filtered_x = normalized_x;
		state->filtered_y = normalized_y;
		state->touched = touch_pressure > touch_threshold;
	}

	if(!state->touched)
	{
		normalized_x = 0.5f;
		normalized_y = 0.5f;
		state->filtered_x = normalized_x;
		state->filtered_y = normalized_y;
	}

	*x = normalized_to_axis(normalized_x);
	*y = -normalized_to_axis(normalized_y);

	if(!state->click_filter_initialized)
	{
		state->click_samples[0] = raw_click;
		state->click_samples[1] = raw_click;
		state->click_filter_initialized = true;
	}
	else
	{
		state->click_samples[state->click_sample_index] = raw_click;
		state->click_sample_index ^= 1;
	}
	click_value = ((int32_t)state->click_samples[0] + state->click_samples[1]) / 2;

	if(state->stick_touched)
	{
		state->stick_touched =
		    click_value >= registry_setting_or_default(IBEX_SETTING_OLYMPUS_CLICK_RELEASE,
		                                               OLYMPUS_CLICK_RELEASE_THRESHOLD);
	}
	else
	{
		state->stick_touched =
		    click_value >
		    registry_setting_or_default(IBEX_SETTING_OLYMPUS_CLICK_PRESS, OLYMPUS_CLICK_THRESHOLD);
	}

	if(state->pad_clicked)
	{
		state->pad_clicked = pressure_value >= (pressure_click_threshold * 3) / 4;
	}
	else
	{
		state->pad_clicked = pressure_value >= pressure_click_threshold;
	}

	*pressure = scale_pad_pressure(pressure_value);
	*touched = state->touched;
	*stick_touched = state->stick_touched;
	*clicked = state->pad_clicked;

	if(!state->touched)
	{
		*x = 0;
		*y = 0;
		*pressure = 0;
	}

	*debug = (struct olympus_pad_debug){
		.x = *x,
		.y = *y,
		.pressure = *pressure,
		.raw_pressure = raw_pressure,
		.raw_click = raw_click,
		.peak_x = peak_x,
		.peak_y = peak_y,
		.peak = touch_pressure,
		.noise_threshold = noise_threshold,
		.touch_threshold = touch_threshold,
		.release_threshold = release_threshold,
		.pad_click_threshold = pressure_click_threshold,
		.stick_touched = *stick_touched,
		.touched = *touched,
		.clicked = *clicked,
	};
}

static void olympus_process_sensor_payload(const uint8_t *payload)
{
	int16_t values[OLYMPUS_SENSOR_PAYLOAD_SIZE / sizeof(int16_t)];
	struct controller_report next = { 0 };
	struct olympus_debug_snapshot debug = { 0 };
	int16_t left_x;
	int16_t left_y;
	int16_t right_x;
	int16_t right_y;
	uint16_t left_pressure;
	uint16_t right_pressure;
	bool left_touched, left_stick_touched, left_clicked;
	bool right_touched, right_stick_touched, right_clicked;
	int16_t click_suppress_mask;

	for(size_t i = 0; i < ARRAY_SIZE(values); ++i)
	{
		values[i] = sys_get_le16(&payload[i * sizeof(int16_t)]);
	}

	decode_pad(&left_pad, &values[0], &values[8], values[33], values[37], false, &left_x, &left_y,
	           &left_pressure, &left_touched, &left_stick_touched, &left_clicked, &debug.left);
	decode_pad(&right_pad, &values[16], &values[24], values[32], values[36], true, &right_x,
	           &right_y, &right_pressure, &right_touched, &right_stick_touched, &right_clicked,
	           &debug.right);
	click_suppress_mask = registry_setting_or_default(IBEX_SETTING_OLYMPUS_CLICK_SUPPRESS_MASK, 3);

	if(left_clicked && (click_suppress_mask & BIT(0)))
	{
		left_x = 0;
		left_y = 0;
		left_pressure = 0;
		left_touched = false;
		left_stick_touched = false;
		debug.left.x = left_x;
		debug.left.y = left_y;
		debug.left.pressure = left_pressure;
		debug.left.touched = left_touched;
		debug.left.stick_touched = left_stick_touched;
	}
	if(right_clicked && (click_suppress_mask & BIT(1)))
	{
		right_x = 0;
		right_y = 0;
		right_pressure = 0;
		right_touched = false;
		right_stick_touched = false;
		debug.right.x = right_x;
		debug.right.y = right_y;
		debug.right.pressure = right_pressure;
		debug.right.touched = right_touched;
		debug.right.stick_touched = right_stick_touched;
	}

	next.touchpad_left_x = left_x;
	next.touchpad_left_y = left_y;
	next.touchpad_left_pressure = left_pressure;
	next.touchpad_right_x = right_x;
	next.touchpad_right_y = right_y;
	next.touchpad_right_pressure = right_pressure;

	if(left_touched)
	{
		next.buttons |= BIT(CONTROLLER_BUTTON_LEFT_TOUCHPAD_TOUCH);
	}
	if(left_clicked)
	{
		next.buttons |= BIT(CONTROLLER_BUTTON_LEFT_TOUCHPAD_CLICK);
	}
	if(left_stick_touched)
	{
		next.buttons |= BIT(CONTROLLER_BUTTON_LEFT_STICK_TOUCH);
	}
	if(right_touched)
	{
		next.buttons |= BIT(CONTROLLER_BUTTON_RIGHT_TOUCHPAD_TOUCH);
	}
	if(right_clicked)
	{
		next.buttons |= BIT(CONTROLLER_BUTTON_RIGHT_TOUCHPAD_CLICK);
	}
	if(right_stick_touched)
	{
		next.buttons |= BIT(CONTROLLER_BUTTON_RIGHT_STICK_TOUCH);
	}

	k_mutex_lock(&olympus_report_mutex, K_FOREVER);
	olympus_report = next;
	debug.frame_count = ++olympus_frame_count;
	olympus_debug = debug;
	k_mutex_unlock(&olympus_report_mutex);
	hardware_signal_input_changed();
}

static int olympus_read_frame(void)
{
	uint8_t header[5];
	uint8_t frame[5 + OLYMPUS_MAX_PAYLOAD_SIZE];
	uint16_t payload_size;
	int err;

	err = i2c_read_dt(&olympus_i2c, header, sizeof(header));
	if(err)
	{
		return err;
	}

	payload_size = sys_get_le16(&header[3]);
	if(header[2] != 0x0e || payload_size == 0xa5a5 || payload_size > OLYMPUS_MAX_PAYLOAD_SIZE)
	{
		return -EBADMSG;
	}

	err = i2c_read_dt(&olympus_i2c, frame, payload_size + 5);
	if(err)
	{
		return err;
	}

	if(payload_size == OLYMPUS_SENSOR_PAYLOAD_SIZE)
	{
		olympus_process_sensor_payload(&frame[5]);
	}
	return 0;
}

static void olympus_irq_changed(const struct device *port, struct gpio_callback *callback,
                                gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(callback);
	ARG_UNUSED(pins);
	k_sem_give(&olympus_irq_sem);
}

static void olympus_thread_entry(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

	for(;;)
	{
		(void)k_sem_take(&olympus_irq_sem, K_MSEC(20));
		while(gpio_pin_get_dt(&olympus_irq) > 0)
		{
			int err = olympus_read_frame();

			if(err)
			{
				LOG_DBG("frame read failed: %d", err);
				break;
			}
		}
	}
}

int olympus_init(void)
{
	int err;

	if(!i2c_is_ready_dt(&olympus_i2c) ||
	   !gpio_is_ready_dt(&olympus_irq) ||
	   !gpio_is_ready_dt(&olympus_reset))
	{
		return -ENODEV;
	}

	k_sem_init(&olympus_irq_sem, 0, 1);
	k_mutex_init(&olympus_report_mutex);

	err = gpio_pin_configure_dt(&olympus_reset, GPIO_OUTPUT_INACTIVE);
	if(err)
	{
		return err;
	}
	err = gpio_pin_configure_dt(&olympus_irq, GPIO_INPUT);
	if(err)
	{
		return err;
	}

	gpio_pin_set_dt(&olympus_reset, 0);
	k_sleep(K_MSEC(2));
	gpio_pin_set_dt(&olympus_reset, 1);
	k_sleep(K_MSEC(12));

	err = olympus_check_descriptor();
	if(err)
	{
		return err;
	}

	err = olympus_apply_factory_config();
	if(err)
	{
		return err;
	}

	gpio_init_callback(&olympus_irq_callback, olympus_irq_changed, BIT(olympus_irq.pin));
	err = gpio_add_callback(olympus_irq.port, &olympus_irq_callback);
	if(err)
	{
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&olympus_irq, GPIO_INT_EDGE_TO_ACTIVE);
	if(err)
	{
		return err;
	}

	k_thread_create(&olympus_thread, olympus_thread_stack,
	                K_THREAD_STACK_SIZEOF(olympus_thread_stack), olympus_thread_entry, NULL, NULL,
	                NULL, 9, 0, K_NO_WAIT);
	k_thread_name_set(&olympus_thread, "olympus");
	LOG_INF("Olympus configured at I2C address 0x%02x", olympus_i2c.addr);
	return 0;
}

void olympus_read_report(struct controller_report *report)
{
	k_mutex_lock(&olympus_report_mutex, K_FOREVER);
	report->touchpad_left_x = olympus_report.touchpad_left_x;
	report->touchpad_left_y = olympus_report.touchpad_left_y;
	report->touchpad_left_pressure = olympus_report.touchpad_left_pressure;
	report->touchpad_right_x = olympus_report.touchpad_right_x;
	report->touchpad_right_y = olympus_report.touchpad_right_y;
	report->touchpad_right_pressure = olympus_report.touchpad_right_pressure;
	report->buttons |= olympus_report.buttons;
	k_mutex_unlock(&olympus_report_mutex);
}

void olympus_get_debug_snapshot(struct olympus_debug_snapshot *snapshot)
{
	k_mutex_lock(&olympus_report_mutex, K_FOREVER);
	*snapshot = olympus_debug;
	k_mutex_unlock(&olympus_report_mutex);
}
