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
#define MP2733_REG_CHARGE_CURRENT 0x05
#define MP2733_REG_PRECHARGE_TERMINATION 0x06
#define MP2733_REG_CHARGE_VOLTAGE 0x07
#define MP2733_REG_TIMER_CONFIGURATION 0x08
#define MP2733_REG_STATUS_BASE 0x0c
#define MP2733_ADC_ENABLE BIT(6)
#define MP2733_VBATT_REG_OFFSET_MV 3400
#define MP2733_VBATT_REG_STEP_MV 10
#define MP2733_VBATT_REG_MAX_MV 4670
/* IIN_LIM=200 mA, ICC=1720mA, IPRE=230mA, ITERM=120mA,
 * watchdog off, termination and safety timer on.
 * IIN_LIM can be raised to 500 mA depending on the power source, NYI */
#define MP2733_CHARGE_CURRENT 0xa3
#define MP2733_PRECHARGE_TERMINATION 0x20
#define MP2733_TIMER_CONFIGURATION 0x85
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
#define BATTERY_FULL_MV 4120
#define MP2733_SOC_SCALE 1000000LL

struct mp2733_soc_segment
{
	uint16_t threshold_mv;
	int32_t slope;
	int32_t intercept;
};

/* clang-format on
 * ^ this directive suppresses formatting for this comment only
 *
 * Generated with:
 * python3 -c 'import pathlib,struct;d=pathlib.Path("IBEX_FW_69FE17FF.fw.payload.bin").read_bytes();b=0x8000;[print(n+"\n"+"\n".join(f"{{ {round(v*1000)}, {round(m*1000000)}, {round(c*1000000)} }},"for v,m,c in struct.iter_unpack("<fff",d[a-b:a-b+l*12])))for n,a,l in(("charge",0x5b6b4,16),("discharge",0x5b774,13))]'
 *
 * the multiplier must match the MP2733_SOC_SCALE constant above
 */
static const struct mp2733_soc_segment mp2733_charge_curve[] = {
	{ 3545, 0, 0 },
	{ 3766, 125873, -473989 },
	{ 3804, 3610806, -13729670 },
	{ 3822, 1864111, -7053337 },
	{ 3865, 3249592, -12408002 },
	{ 3907, 4376704, -16811445 },
	{ 3928, 2723910, -10320033 },
	{ 3966, 2140409, -8005860 },
	{ 3981, 1197510, -4252494 },
	{ 4054, 1441182, -5240413 },
	{ 4107, 1255132, -4476263 },
	{ 4209, 1073036, -3709823 },
	{ 4325, 1932818, -7428214 },
	{ 4340, 4824315, -19977425 },
	{ 4345, 7921407, -33434048 },
	{ 4347, 386755, -681853 },
};

static const struct mp2733_soc_segment mp2733_discharge_curve[] = {
	{ 3666, 0, 0 },
	{ 3684, 1067181, -3912718 },
	{ 3690, 5094007, -18748272 },
	{ 3693, 7839416, -28878401 },
	{ 3739, 1713558, -6256995 },
	{ 3783, 2943625, -10855810 },
	{ 3811, 3711445, -13760778 },
	{ 3848, 2618301, -9595303 },
	{ 3863, 2244718, -8157812 },
	{ 3919, 1155642, -3950404 },
	{ 4091, 1259872, -4358917 },
	{ 4287, 1039723, -3458326 },
	{ 4295, 112280, 517724 },
};

#define MP2733_CHARGE_CURVE_BASE_PERCENT 75
#define MP2733_DISCHARGE_CURVE_BASE_PERCENT 83

static const struct i2c_dt_spec mp2733 = I2C_DT_SPEC_GET(MP2733_NODE);
static struct controller_battery_report cached_report;
static struct k_mutex battery_lock;
static K_THREAD_STACK_DEFINE(battery_stack, 1024);
static struct k_thread battery_thread;

static bool mp2733_charge_complete(enum mp2733_chg_stat chg_stat, uint8_t vin_stat,
                                   uint16_t battery_mv);

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

static int mp2733_write_reg_verify(uint8_t reg, uint8_t value)
{
	uint8_t actual;
	int err;

	err = i2c_reg_write_byte_dt(&mp2733, reg, value);
	if(err)
	{
		return err;
	}

	err = i2c_reg_read_byte_dt(&mp2733, reg, &actual);
	if(err)
	{
		return err;
	}

	return actual == value ? 0 : -EIO;
}

static int mp2733_vbat_reg_value(uint16_t mv, uint8_t *value)
{
	if(mv < MP2733_VBATT_REG_OFFSET_MV || mv > MP2733_VBATT_REG_MAX_MV)
	{
		return -EINVAL;
	}

	*value = (uint8_t)(((mv - MP2733_VBATT_REG_OFFSET_MV) / MP2733_VBATT_REG_STEP_MV) << 1);
	return 0;
}

static int mp2733_configure_charger(uint16_t full_mv)
{
	uint8_t value;
	int err;

	err = mp2733_vbat_reg_value(full_mv, &value);
	if(err)
	{
		return err;
	}

	err = mp2733_write_reg_verify(MP2733_REG_CHARGE_VOLTAGE, value);
	if(err)
	{
		return err;
	}

	err = mp2733_write_reg_verify(MP2733_REG_CHARGE_CURRENT, MP2733_CHARGE_CURRENT);
	if(err)
	{
		return err;
	}

	err = mp2733_write_reg_verify(MP2733_REG_PRECHARGE_TERMINATION, MP2733_PRECHARGE_TERMINATION);
	if(err)
	{
		return err;
	}

	return mp2733_write_reg_verify(MP2733_REG_TIMER_CONFIGURATION, MP2733_TIMER_CONFIGURATION);
}

static int64_t div_round_closest_s64(int64_t numerator, int64_t denominator)
{
	if(numerator < 0)
	{
		return (numerator - denominator / 2) / denominator;
	}
	return (numerator + denominator / 2) / denominator;
}

static const struct mp2733_soc_segment *
mp2733_select_lower_bound_segment(const struct mp2733_soc_segment *curve, size_t curve_count,
                                  uint16_t mv)
{
	const struct mp2733_soc_segment *segment = &curve[0];

	for(size_t i = 1; i < curve_count; ++i)
	{
		if(mv < curve[i].threshold_mv)
		{
			break;
		}

		segment = &curve[i];
	}

	return segment;
}

static const struct mp2733_soc_segment *
mp2733_select_upper_bound_segment(const struct mp2733_soc_segment *curve, size_t curve_count,
                                  uint16_t mv)
{
	for(size_t i = 0; i < curve_count; ++i)
	{
		if(mv <= curve[i].threshold_mv)
		{
			return &curve[i];
		}
	}

	return &curve[curve_count - 1];
}

static uint8_t mp2733_percent_from_segment(const struct mp2733_soc_segment *segment,
                                           uint8_t base_percent, uint16_t mv)
{
	int64_t millipercent = 100 * 1000;
	int64_t line;

	line = (int64_t)segment->slope * mv + (int64_t)segment->intercept * 1000LL;
	millipercent = div_round_closest_s64(line * (200U - base_percent), MP2733_SOC_SCALE);

	if(millipercent <= 0)
	{
		return 0;
	}
	if(millipercent >= 100 * 1000)
	{
		return 100;
	}
	return (uint8_t)((millipercent + 500) / 1000);
}

static uint8_t mp2733_level_from_voltage(uint16_t mv, enum mp2733_chg_stat chg_stat,
                                         uint8_t vin_stat)
{
	const struct mp2733_soc_segment *segment;

	if(mp2733_charge_complete(chg_stat, vin_stat, mv))
	{
		return 100;
	}

	/* OFW charge thresholds act as lower bounds; discharge thresholds act as upper bounds. */
	if(chg_stat == MP2733_TRICKLE || chg_stat == MP2733_CONSTANT_CURRENT)
	{
		segment = mp2733_select_lower_bound_segment(mp2733_charge_curve,
		                                            ARRAY_SIZE(mp2733_charge_curve), mv);
		return mp2733_percent_from_segment(segment, MP2733_CHARGE_CURVE_BASE_PERCENT, mv);
	}
	segment = mp2733_select_upper_bound_segment(mp2733_discharge_curve,
	                                            ARRAY_SIZE(mp2733_discharge_curve), mv);
	return mp2733_percent_from_segment(segment, MP2733_DISCHARGE_CURVE_BASE_PERCENT, mv);
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

	err = mp2733_update_reg(MP2733_REG_TIMER_CONFIGURATION, MP2733_ADC_ENABLE, 0);
	err |= mp2733_update_reg(MP2733_REG_CONVERTER_CTRL_A, MP2733_ADC_ENABLE, 0);
	k_msleep(BATTERY_ADC_SETTLE_MS);
	err |= mp2733_update_reg(MP2733_REG_CONVERTER_CTRL_A, 0, MP2733_ADC_ENABLE);
	err |= mp2733_update_reg(MP2733_REG_TIMER_CONFIGURATION, 0, MP2733_ADC_ENABLE);
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
	report->level_percent = mp2733_level_from_voltage(report->battery_mv, chg_stat, vin_stat);
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
	int err;

	k_mutex_init(&battery_lock);

	if(!i2c_is_ready_dt(&mp2733))
	{
		LOG_WRN("MP2733 I2C bus is not ready");
		return -ENODEV;
	}

	err = mp2733_configure_charger(BATTERY_FULL_MV);
	if(err)
	{
		LOG_WRN("MP2733 charger setup failed: %d", err);
		return err;
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
