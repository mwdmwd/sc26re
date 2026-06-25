/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "battery.h"

#if CONFIG_IBEX_BATTERY

#if defined(CONFIG_BT_BAS)
#include <zephyr/bluetooth/services/bas.h>
#endif

LOG_MODULE_REGISTER(battery);

#define MP2733_NODE DT_NODELABEL(mp2733)

#define MP2733_REG_CONVERTER_CTRL_A 0x03
#define MP2733_REG_CONVERTER_CTRL_B 0x08
#define MP2733_REG_STATUS_BASE 0x0c
#define MP2733_ADC_ENABLE BIT(6)
#define MP2733_VIN_STAT_SHIFT 5
#define MP2733_VIN_STAT_MASK 0x07
enum mp2733_chg_stat
{
	MP2733_NOT_CHARGING = 0,
	MP2733_TRICKLE = 1,
	MP2733_CONSTANT_CURRENT = 2,
	MP2733_DONE = 3,
};
#define MP2733_CHG_STAT_SHIFT 3
#define MP2733_CHG_STAT_MASK 0x03

#define BATTERY_POLL_PERIOD_MS 30000
#define BATTERY_INITIAL_POLL_DELAY_MS 500
#define BATTERY_ADC_SETTLE_MS 164
#define BATTERY_EMPTY_MV 3400
/* "full": LED turns off when charging, "SoC100": scale for charge level */
#define BATTERY_FULL_MV 4120
#define BATTERY_SOC100_MV 4200

static const struct i2c_dt_spec mp2733 = I2C_DT_SPEC_GET(MP2733_NODE);
static struct controller_battery_report cached_report;
static struct k_mutex battery_lock;
static K_THREAD_STACK_DEFINE(battery_stack, 1024);
static struct k_thread battery_thread;

static int mp2733_update_reg(uint8_t reg, uint8_t set_mask, uint8_t clear_mask)
{
	uint8_t value;
	int err;

	err = i2c_reg_read_byte_dt(&mp2733, reg, &value);
	if(err)
	{
		return err;
	}

	value = (value | set_mask) & ~clear_mask;
	return i2c_reg_write_byte_dt(&mp2733, reg, value);
}

static uint8_t mp2733_level_from_voltage(uint16_t mv)
{
	if(mv <= BATTERY_EMPTY_MV)
	{
		return 0;
	}
	if(mv >= BATTERY_SOC100_MV)
	{
		return 100;
	}
	return (uint8_t)(((uint32_t)(mv - BATTERY_EMPTY_MV) * 100U) /
	                 (BATTERY_SOC100_MV - BATTERY_EMPTY_MV));
}

static uint8_t mp2733_charge_state(enum mp2733_chg_stat chg_stat, uint8_t vin_stat)
{
	switch(chg_stat)
	{
		case MP2733_TRICKLE:
		case MP2733_CONSTANT_CURRENT:
			return CONTROLLER_CHARGE_STATE_CHARGING;
		case MP2733_DONE:
			return CONTROLLER_CHARGE_STATE_CHARGING_DONE;
		case MP2733_NOT_CHARGING:
		default:
			return vin_stat ? CONTROLLER_CHARGE_STATE_SOURCE_VALIDATE
			                : CONTROLLER_CHARGE_STATE_DISCHARGING;
	}
}

static uint16_t mp2733_current_ma(uint8_t raw)
{
	return (uint16_t)(((uint32_t)17500U * raw) / 1000U);
}

static bool mp2733_charge_complete(enum mp2733_chg_stat chg_stat, uint8_t vin_stat,
                                   uint16_t battery_mv)
{
	if(vin_stat == 0U)
	{
		return false;
	}

	return chg_stat == MP2733_DONE || battery_mv >= BATTERY_FULL_MV;
}

static int battery_poll_once(struct controller_battery_report *report)
{
	uint8_t raw[8];
	enum mp2733_chg_stat chg_stat;
	uint8_t vin_stat;
	int err;

	if(!i2c_is_ready_dt(&mp2733))
	{
		return -ENODEV;
	}

	err = mp2733_update_reg(MP2733_REG_CONVERTER_CTRL_B, MP2733_ADC_ENABLE, 0);
	err |= mp2733_update_reg(MP2733_REG_CONVERTER_CTRL_A, MP2733_ADC_ENABLE, 0);
	k_msleep(BATTERY_ADC_SETTLE_MS);
	err |= mp2733_update_reg(MP2733_REG_CONVERTER_CTRL_A, 0, MP2733_ADC_ENABLE);
	err |= mp2733_update_reg(MP2733_REG_CONVERTER_CTRL_B, 0, MP2733_ADC_ENABLE);
	if(err)
	{
		return err;
	}

	err = i2c_burst_read_dt(&mp2733, MP2733_REG_STATUS_BASE, raw, sizeof(raw));
	if(err)
	{
		return err;
	}

	vin_stat = (raw[0] >> MP2733_VIN_STAT_SHIFT) & MP2733_VIN_STAT_MASK;
	chg_stat = (raw[0] >> MP2733_CHG_STAT_SHIFT) & MP2733_CHG_STAT_MASK;
	memset(report, 0, sizeof(*report));
	report->charge_state = mp2733_charge_state(chg_stat, vin_stat);
	report->battery_mv = 20U * raw[2];
	report->system_mv = 20U * raw[3];
	report->input_mv = 60U * raw[5];
	report->current_ma = mp2733_current_ma(raw[6]);
	report->input_current_ma = ((uint32_t)13300U * raw[7]) / 1000U;
	report->level_percent = mp2733_level_from_voltage(report->battery_mv);
	report->charger_type = vin_stat;
	report->charge_complete = mp2733_charge_complete(chg_stat, vin_stat, report->battery_mv);
	report->valid = true;
	return 0;
}

static void battery_publish(const struct controller_battery_report *report)
{
#if defined(CONFIG_BT_BAS)
	(void)bt_bas_set_battery_level(report->level_percent);
#endif
	(void)transport_send_battery_status(report);
}

int battery_read_fresh_status(struct controller_battery_report *report)
{
	int err;

	if(report == NULL)
	{
		return -EINVAL;
	}

	k_mutex_lock(&battery_lock, K_FOREVER);
	err = battery_poll_once(report);
	if(!err)
	{
		cached_report = *report;
	}
	k_mutex_unlock(&battery_lock);
	if(err)
	{
		return err;
	}

	return 0;
}

static void battery_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_msleep(BATTERY_INITIAL_POLL_DELAY_MS);

	for(;;)
	{
		struct controller_battery_report report;
		int err = battery_read_fresh_status(&report);

		if(err)
		{
			LOG_WRN("MP2733 battery poll failed: %d", err);
		}
		else
		{
			battery_publish(&report);
			LOG_INF("%u%% %umV state=%s input=%umV ichg=%umA type=%u", report.level_percent,
			        report.battery_mv, battery_charge_state_name(report.charge_state),
			        report.input_mv, report.current_ma, report.charger_type);
		}

		k_msleep(BATTERY_POLL_PERIOD_MS);
	}
}

int battery_init(void)
{
	k_mutex_init(&battery_lock);

	if(!i2c_is_ready_dt(&mp2733))
	{
		LOG_WRN("MP2733 I2C bus is not ready");
		return -ENODEV;
	}

	k_thread_create(&battery_thread, battery_stack, K_THREAD_STACK_SIZEOF(battery_stack),
	                battery_thread_entry, NULL, NULL, NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);
	k_thread_name_set(&battery_thread, "battery");
	return 0;
}

int battery_get_status(struct controller_battery_report *report)
{
	if(report == NULL)
	{
		return -EINVAL;
	}

	k_mutex_lock(&battery_lock, K_FOREVER);
	*report = cached_report;
	k_mutex_unlock(&battery_lock);

	return report->valid ? 0 : -EAGAIN;
}

const char *battery_charge_state_name(uint8_t charge_state)
{
	switch(charge_state)
	{
		case CONTROLLER_CHARGE_STATE_RESET:
			return "reset";
		case CONTROLLER_CHARGE_STATE_DISCHARGING:
			return "discharging";
		case CONTROLLER_CHARGE_STATE_CHARGING:
			return "charging";
		case CONTROLLER_CHARGE_STATE_SOURCE_VALIDATE:
			return "source-validate";
		case CONTROLLER_CHARGE_STATE_CHARGING_DONE:
			return "done";
		default:
			return "unknown";
	}
}

#else

int battery_init(void)
{
	return -ENODEV;
}

int battery_read_fresh_status(struct controller_battery_report *report)
{
	if(report == NULL)
	{
		return -EINVAL;
	}

	memset(report, 0, sizeof(*report));
	return -ENODEV;
}

int battery_get_status(struct controller_battery_report *report)
{
	if(report == NULL)
	{
		return -EINVAL;
	}

	memset(report, 0, sizeof(*report));
	return -ENODEV;
}

const char *battery_charge_state_name(uint8_t charge_state)
{
	ARG_UNUSED(charge_state);

	return "unavailable";
}

#endif
