/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/ztest.h>

#include "controller.h"
#include "ibex_settings_registry.h"
#include "imu.h"
#include "lsm6dsv16x_emul.h"

#define REG_FIFO_CTRL4 0x0a
#define REG_CTRL10 0x19

static const struct gpio_dt_spec interrupt = GPIO_DT_SPEC_GET(DT_ALIAS(accel0), int1_gpios);
static const uint8_t gyro_sample[7] = { 0x08, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00 };

static void read_sample(void)
{
	lsm6dsv16x_emul_set_fifo(gyro_sample);
	zassert_ok(gpio_emul_input_set_dt(&interrupt, 1));
	zassert_ok(gpio_emul_input_set_dt(&interrupt, 0));
	k_msleep(20);
}

ZTEST(imu_threshold, test_live_threshold_change_preserves_bias_and_fifo)
{
	struct controller_report report = { 0 };
	const uint8_t thresholds[] = { 50, 0, 200 };

	ibex_settings_registry_init();
	zassert_ok(ibex_setting_set(IBEX_SETTING_IMU_MODE, 1));
	zassert_ok(imu_init());
	k_msleep(20);

	/* Model learned SFLP bias that differs from the initial zero calibration. */
	lsm6dsv16x_emul_page_set(0x6e, 0x55);
	lsm6dsv16x_emul_page_set(0x6f, 0x35);
	lsm6dsv16x_emul_clear_write_counts();

	for(size_t i = 0; i < ARRAY_SIZE(thresholds); ++i)
	{
		zassert_ok(ibex_setting_set(IBEX_SETTING_IMU_GYRO_THRESHOLD, thresholds[i]));
		/* Exercise multiple completions, including any pending cancellation/restart. */
		read_sample();
		read_sample();
		read_sample();
		zassert_equal(0x55, lsm6dsv16x_emul_page_get(0x6e));
		zassert_equal(0x35, lsm6dsv16x_emul_page_get(0x6f));
		zassert_equal(0, lsm6dsv16x_emul_write_count(REG_FIFO_CTRL4));
		zassert_equal(0, lsm6dsv16x_emul_write_count(REG_CTRL10));
	}
	zassert_ok(imu_read_report(&report));
	zassert_not_equal(0, report.gyro_x);
	zassert_not_equal(0, report.gyro_y);
}

ZTEST_SUITE(imu_threshold, NULL, NULL, NULL, NULL, NULL);
