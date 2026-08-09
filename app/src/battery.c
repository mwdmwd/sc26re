/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include <errno.h>
#include <limits.h>
#include <string.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "analog.h"
#include "battery.h"
#include "calibration.h"

#if CONFIG_IBEX_BATTERY

LOG_MODULE_REGISTER(battery);

#define MP2733_NODE DT_NODELABEL(mp2733)

#define MP2733_REG_INPUT_CURRENT_LIMIT 0x00
#define MP2733_REG_INPUT_VOLTAGE_LIMIT 0x01
#define MP2733_REG_THERMAL_CONFIGURATION 0x02
#define MP2733_REG_ADC_OTG_CONFIGURATION 0x03
#define MP2733_REG_CHARGE_CONTROL 0x04
#define MP2733_REG_CHARGE_VOLTAGE 0x07
#define MP2733_REG_TIMER_CONFIGURATION 0x08
#define MP2733_REG_BATFET_CONFIGURATION 0x0a
#define MP2733_REG_STATUS_BASE 0x0c
#define MP2733_REG_INPUT_LIMIT_STATUS 0x14
#define MP2733_REG_DPM_MASK 0x15
#define MP2733_INITIAL_CONFIG_REGISTER_COUNT 11
#define MP2733_STATUS_REGISTER_COUNT 12
#define MP2733_REG_RESET BIT(7)
#define MP2733_TSM_DELAY BIT(7)
#define MP2733_BATFET_DISABLE BIT(5)
#define MP2733_ADC_START BIT(7)
#define MP2733_ADC_RATE BIT(6)
#define MP2733_ADC_CONTROL_MASK (MP2733_ADC_START | MP2733_ADC_RATE)
#define MP2733_EN_ADC_DSG BIT(6)
#define MP2733_DPM_INTERRUPT_MASK (BIT(6) | BIT(5))
#define MP2733_VBATT_REG_OFFSET_MV 3400
#define MP2733_VBATT_REG_STEP_MV 10
#define MP2733_VBATT_REG_MAX_MV 4670
#define MP2733_VIN_REG_OFFSET_MV 3700
#define MP2733_VIN_REG_STEP_MV 100
#define MP2733_VIN_REG_MAX_MV 15200
#define MP2733_VIN_REG_MASK GENMASK(6, 0)
#define MP2733_INPUT_LIMIT_NO_SOURCE 0x40
#define MP2733_INPUT_LIMIT_SOURCE_PRESENT 0x48
#define MP2733_CHARGE_CONTROL_DISABLED 0x43
#define MP2733_CHARGE_CONTROL_ENABLED 0x53
#define MP2733_CHARGE_VOLTAGE_HOLD 0x90
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
#define BATTERY_POLL_PERIOD_MS 2000
#define BATTERY_LOG_PERIOD_MS 30000
#define BATTERY_INITIAL_POLL_DELAY_MS 500
#define BATTERY_CACHE_MAX_AGE_MS (3 * BATTERY_POLL_PERIOD_MS)
#define BATTERY_ADC_CONVERSION_MS 164
#define BATTERY_CHARGE_STATE_SETTLE_MS 500
#define BATTERY_METER_WINDOW_SIZE 40
#define BATTERY_HOLD_MV 4120
#define BATTERY_CHARGE_MV 4160
#define MP2733_VIN_SWEEP_START_MV 4600
#define MP2733_VIN_SWEEP_STOP_MV 4300
#define MP2733_VIN_SWEEP_STEP_MV 100
#define MP2733_VIN_SWEEP_STEP_MS 10000
#define MP2733_VIN_SWEEP_RETRY_MS 120000
#define MP2733_SOC_SCALE 1000000LL

BUILD_ASSERT(BATTERY_HOLD_MV >= MP2733_VBATT_REG_OFFSET_MV &&
             BATTERY_HOLD_MV <= MP2733_VBATT_REG_MAX_MV &&
             (BATTERY_HOLD_MV - MP2733_VBATT_REG_OFFSET_MV) % MP2733_VBATT_REG_STEP_MV == 0U);
BUILD_ASSERT(BATTERY_CHARGE_MV >= MP2733_VBATT_REG_OFFSET_MV &&
             BATTERY_CHARGE_MV <= MP2733_VBATT_REG_MAX_MV &&
             (BATTERY_CHARGE_MV - MP2733_VBATT_REG_OFFSET_MV) % MP2733_VBATT_REG_STEP_MV == 0U);

static const uint8_t mp2733_initial_config[MP2733_INITIAL_CONFIG_REGISTER_COUNT] = {
	0x42, /* REG00: input limit enabled, 200 mA initial limit */
	0x09, /* REG01: 4.6 V input-voltage limit */
	0xdc, /* REG02: JEITA/NTC charging, 120 C thermal regulation */
	0x10, /* REG03: ADC one-shot, 5 V / 500 mA OTG */
	0x43, /* REG04: charger disabled during setup, 3.15 V minimum system voltage */
	0xa3, /* REG05: 1.72 A charge-current ceiling */
	0x20, /* REG06: 230 mA precharge, 120 mA termination */
	0x90, /* REG07: conservative 4.12 V target */
	0x85, /* REG08: termination and 12-hour safety timer, watchdog off */
	0x00, /* REG09: bandgap enabled */
	0x42, /* REG0A: stock switching/BATFET configuration */
};

enum mp2733_fg_state
{
	MP2733_FG_SETTLING,
	MP2733_FG_VBAT_SWEEP,
	MP2733_FG_CHARGING,
	MP2733_FG_FULL,
};

struct mp2733_fg_policy
{
	enum mp2733_fg_state state;
	int64_t deadline_ms;
	uint16_t next_vin_min_mv;
	uint16_t requested_vin_min_mv;
	uint16_t vbat_target_mv;
	uint8_t source_present;
	uint8_t level_percent;
	bool charge_active_valid;
	bool charge_active;
	bool level_valid;
};

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
 *
 * The same tables are byte-identical at 0x6468c/0x6474c in 6A4D85E3.
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
static int64_t cached_report_at_ms;
static struct mp2733_fg_policy fg_policy;
static uint16_t battery_meter_samples_mv[BATTERY_METER_WINDOW_SIZE];
static uint16_t battery_meter_batch_mv[BATTERY_METER_WINDOW_SIZE];
static uint32_t battery_meter_sample_count;
static bool battery_powering_off;
K_MUTEX_DEFINE(battery_poll_lock);
K_MUTEX_DEFINE(battery_cache_lock);
static K_THREAD_STACK_DEFINE(battery_stack, 1536);
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

static int mp2733_set_vbat_target(uint16_t mv)
{
	uint8_t value = (uint8_t)(((mv - MP2733_VBATT_REG_OFFSET_MV) / MP2733_VBATT_REG_STEP_MV) << 1);
	int err;

	if(fg_policy.vbat_target_mv == mv)
	{
		return 0;
	}

	err = mp2733_write_reg_verify(MP2733_REG_CHARGE_VOLTAGE, value);
	if(!err)
	{
		fg_policy.vbat_target_mv = mv;
	}
	return err;
}

static int mp2733_set_vin_min(uint16_t mv)
{
	uint8_t value;
	int err;

	if(mv < MP2733_VIN_REG_OFFSET_MV ||
	   mv > MP2733_VIN_REG_MAX_MV ||
	   (mv - MP2733_VIN_REG_OFFSET_MV) % MP2733_VIN_REG_STEP_MV != 0U)
	{
		return -EINVAL;
	}
	value = (mv - MP2733_VIN_REG_OFFSET_MV) / MP2733_VIN_REG_STEP_MV;
	err = mp2733_write_reg_verify(MP2733_REG_INPUT_VOLTAGE_LIMIT, value);
	if(!err)
	{
		fg_policy.requested_vin_min_mv = mv;
	}
	return err;
}

static int mp2733_set_source_limit(bool source_present)
{
	uint8_t encoded =
	    source_present ? MP2733_INPUT_LIMIT_SOURCE_PRESENT : MP2733_INPUT_LIMIT_NO_SOURCE;
	int err;

	if(fg_policy.source_present == (uint8_t)source_present)
	{
		return 0;
	}

	err = mp2733_write_reg_verify(MP2733_REG_INPUT_CURRENT_LIMIT, encoded);
	if(!err)
	{
		fg_policy.source_present = source_present;
	}
	return err;
}

static int mp2733_write_initial_config(void)
{
	uint8_t transaction[1 + ARRAY_SIZE(mp2733_initial_config)];

	/*
	 * Keep the register address and payload in one I2C message. Zephyr's
	 * i2c_burst_write() represents them as two adjacent write messages, which
	 * the nRF TWIM RTIO driver cannot concatenate without an unexpected bus
	 * transition (so it fails with -ECANCELED).
	 */
	transaction[0] = MP2733_REG_INPUT_CURRENT_LIMIT;
	memcpy(&transaction[1], mp2733_initial_config, sizeof(mp2733_initial_config));
	return i2c_write_dt(&mp2733, transaction, sizeof(transaction));
}

static int mp2733_disable_dpm_interrupts(void)
{
	uint8_t actual;
	int err;

	err = i2c_reg_write_byte_dt(&mp2733, MP2733_REG_DPM_MASK, 0x00);
	if(err)
	{
		return err;
	}
	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_DPM_MASK, &actual);
	if(err)
	{
		return err;
	}

	return (actual & MP2733_DPM_INTERRUPT_MASK) == 0U ? 0 : -EIO;
}

static void mp2733_emergency_disable(void)
{
	int disable_err;
	int target_err;

	disable_err =
	    i2c_reg_write_byte_dt(&mp2733, MP2733_REG_CHARGE_CONTROL, MP2733_CHARGE_CONTROL_DISABLED);
	target_err =
	    i2c_reg_write_byte_dt(&mp2733, MP2733_REG_CHARGE_VOLTAGE, MP2733_CHARGE_VOLTAGE_HOLD);
	if(disable_err || target_err)
	{
		LOG_ERR("MP2733 emergency disable failed: charge=%d target=%d", disable_err, target_err);
	}
}

static int mp2733_configure_charger(void)
{
	uint8_t actual[ARRAY_SIZE(mp2733_initial_config)];
	int err;

	err = i2c_reg_write_byte_dt(&mp2733, MP2733_REG_INPUT_VOLTAGE_LIMIT, MP2733_REG_RESET);
	if(err)
	{
		LOG_ERR("MP2733 register reset failed: %d", err);
		goto fail_safe;
	}

	err = mp2733_write_reg_verify(MP2733_REG_CHARGE_CONTROL, MP2733_CHARGE_CONTROL_DISABLED);
	if(err)
	{
		LOG_ERR("MP2733 pre-configuration disable failed: %d", err);
		goto fail_safe;
	}

	err = mp2733_write_initial_config();
	if(err)
	{
		LOG_ERR("MP2733 initial register write failed: %d", err);
		goto fail_safe;
	}

	err = i2c_burst_read_dt(&mp2733, MP2733_REG_INPUT_CURRENT_LIMIT, actual, sizeof(actual));
	if(err)
	{
		LOG_ERR("MP2733 initial register readback failed: %d", err);
		goto fail_safe;
	}
	for(size_t i = 0; i < ARRAY_SIZE(mp2733_initial_config); ++i)
	{
		if(actual[i] != mp2733_initial_config[i])
		{
			LOG_ERR("MP2733 REG%02x readback mismatch: wrote %02x, read %02x",
			        (unsigned int)(MP2733_REG_INPUT_CURRENT_LIMIT + i), mp2733_initial_config[i],
			        actual[i]);
			err = -EIO;
			goto fail_safe;
		}
	}

	err = mp2733_disable_dpm_interrupts();
	if(err)
	{
		LOG_ERR("MP2733 DPM interrupt-mask setup failed: %d", err);
		goto fail_safe;
	}

	err = mp2733_write_reg_verify(MP2733_REG_CHARGE_CONTROL, MP2733_CHARGE_CONTROL_ENABLED);
	if(err)
	{
		LOG_ERR("MP2733 final charger enable failed: %d", err);
		goto fail_safe;
	}

	memset(&fg_policy, 0, sizeof(fg_policy));
	fg_policy.state = MP2733_FG_SETTLING;
	fg_policy.next_vin_min_mv = MP2733_VIN_SWEEP_START_MV;
	fg_policy.requested_vin_min_mv = MP2733_VIN_SWEEP_START_MV;
	fg_policy.vbat_target_mv = BATTERY_HOLD_MV;
	fg_policy.source_present = UINT8_MAX;
	battery_meter_sample_count = 0;
	return 0;

fail_safe:
	mp2733_emergency_disable();
	return err;
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
	int64_t millipercent;
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

static bool mp2733_charging_active(enum mp2733_chg_stat chg_stat)
{
	return chg_stat == MP2733_TRICKLE || chg_stat == MP2733_CONSTANT_CURRENT;
}

static uint8_t mp2733_level_from_voltage(uint16_t mv, enum mp2733_chg_stat chg_stat,
                                         bool charge_complete)
{
	const struct mp2733_soc_segment *segment;

	if(charge_complete)
	{
		return 100;
	}

	if(mp2733_charging_active(chg_stat))
	{
		segment = mp2733_select_upper_bound_segment(mp2733_charge_curve,
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

static uint16_t mp2733_current_ma(enum mp2733_chg_stat chg_stat, uint8_t raw)
{
	/* Both paths are recovered from Valve's MP2733 ADC worker. */
	if(mp2733_charging_active(chg_stat))
	{
		return (uint16_t)(((uint32_t)17500U * raw) / 1000U);
	}
	if(raw == 0U)
	{
		return 0;
	}
	return (uint16_t)(((uint32_t)70000U * raw + 200000U) / 1000U);
}

static int32_t mp2733_ntc_temperature_mc(uint8_t raw)
{
	float x = raw * 0.00392f;
	float temperature_mc =
	    ((((476.62f * x - 1667.7f) * x + 1909.6f) * x - 999.36f) * x + 225.51f) * 1000.0f;
	/* Polynomial recovered from Valve's MP2733 ADC worker. */
	return (int32_t)(temperature_mc + (temperature_mc >= 0.0f ? 0.5f : -0.5f));
}

static uint16_t mp2733_source_rating_ma(uint8_t vin_stat)
{
	static const uint16_t source_rating_ma[] = {
		0, 500, 1000, 2100, 2400, 500, 1500, 1800,
	};

	return source_rating_ma[vin_stat];
}

static int mp2733_sample_registers(uint8_t raw[MP2733_STATUS_REGISTER_COUNT])
{
	bool adc_dsg_enabled = false;
	uint8_t adc_config;
	int cleanup_err;
	int err;

	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_ADC_OTG_CONFIGURATION, &adc_config);
	if(err)
	{
		return err;
	}
	adc_config &= ~MP2733_ADC_CONTROL_MASK;

	err = mp2733_update_reg(MP2733_REG_TIMER_CONFIGURATION, MP2733_EN_ADC_DSG, 0);
	if(err)
	{
		return err;
	}
	adc_dsg_enabled = true;

	err = i2c_reg_write_byte_dt(&mp2733, MP2733_REG_ADC_OTG_CONFIGURATION,
	                            adc_config | MP2733_ADC_RATE);
	if(!err)
	{
		/* Valve's 6A4D85E3 worker allows 164 ms for a complete ADC scan. */
		k_msleep(BATTERY_ADC_CONVERSION_MS);
	}

	/* Explicitly clear ADC_START as well as ADC_RATE; do not replay a read-high bit. */
	cleanup_err = i2c_reg_write_byte_dt(&mp2733, MP2733_REG_ADC_OTG_CONFIGURATION, adc_config);
	if(!err)
	{
		err = cleanup_err;
	}
	if(adc_dsg_enabled)
	{
		cleanup_err = mp2733_update_reg(MP2733_REG_TIMER_CONFIGURATION, 0, MP2733_EN_ADC_DSG);
		if(!err)
		{
			err = cleanup_err;
		}
	}
	if(err)
	{
		return err;
	}

	return i2c_burst_read_dt(&mp2733, MP2733_REG_STATUS_BASE, raw, MP2733_STATUS_REGISTER_COUNT);
}

static int battery_meter_average_mv(uint16_t *average_mv)
{
	size_t sample_count = battery_meter_sample_count < BATTERY_METER_WINDOW_SIZE
	                          ? BATTERY_METER_WINDOW_SIZE - battery_meter_sample_count
	                          : 1U;
	int16_t offset_mv = calibration_battery_voltage_offset_mv();
	uint32_t sum = 0;
	size_t window_count;
	int err;

	err = analog_battery_voltage_samples_mv(battery_meter_batch_mv, sample_count);
	if(err)
	{
		return err;
	}

	for(size_t i = 0; i < sample_count; ++i)
	{
		int32_t adjusted_mv = battery_meter_batch_mv[i] + offset_mv;

		battery_meter_samples_mv[battery_meter_sample_count % BATTERY_METER_WINDOW_SIZE] =
		    CLAMP(adjusted_mv, 0, UINT16_MAX);
		battery_meter_sample_count++;
	}

	window_count = MIN(battery_meter_sample_count, (uint32_t)BATTERY_METER_WINDOW_SIZE);
	for(size_t i = 0; i < window_count; ++i)
	{
		sum += battery_meter_samples_mv[i];
	}
	*average_mv = sum / window_count;
	return 0;
}

static int mp2733_enter_fg_state(enum mp2733_fg_state state)
{
	int err = 0;

	switch(state)
	{
		case MP2733_FG_SETTLING:
		case MP2733_FG_FULL:
			err = mp2733_set_vbat_target(BATTERY_HOLD_MV);
			break;
		case MP2733_FG_VBAT_SWEEP:
			err = mp2733_set_vbat_target(BATTERY_CHARGE_MV);
			if(!err)
			{
				err = mp2733_set_vin_min(MP2733_VIN_SWEEP_START_MV);
			}
			if(!err)
			{
				fg_policy.next_vin_min_mv = MP2733_VIN_SWEEP_START_MV - MP2733_VIN_SWEEP_STEP_MV;
				fg_policy.deadline_ms = k_uptime_get() + MP2733_VIN_SWEEP_STEP_MS;
			}
			break;
		case MP2733_FG_CHARGING:
			break;
	}
	if(!err)
	{
		fg_policy.state = state;
	}
	return err;
}

static int mp2733_run_fg_policy(enum mp2733_chg_stat chg_stat, uint8_t vin_stat,
                                uint16_t battery_mv)
{
	int64_t now = k_uptime_get();
	int err;

	err = mp2733_set_source_limit(vin_stat != 0U);
	if(err)
	{
		return err;
	}

	if(vin_stat == 0U)
	{
		fg_policy.deadline_ms = 0;
		return mp2733_enter_fg_state(MP2733_FG_SETTLING);
	}

	switch(fg_policy.state)
	{
		case MP2733_FG_SETTLING:
			if(battery_mv >= BATTERY_HOLD_MV)
			{
				return mp2733_enter_fg_state(MP2733_FG_FULL);
			}
			if(now >= fg_policy.deadline_ms)
			{
				return mp2733_enter_fg_state(MP2733_FG_VBAT_SWEEP);
			}
			break;

		case MP2733_FG_VBAT_SWEEP:
			if(now >= fg_policy.deadline_ms)
			{
				if(chg_stat == MP2733_NOT_CHARGING)
				{
					err = mp2733_set_vin_min(fg_policy.next_vin_min_mv);
					if(err)
					{
						return err;
					}
					fg_policy.next_vin_min_mv -= MP2733_VIN_SWEEP_STEP_MV;
				}
				fg_policy.deadline_ms = now + MP2733_VIN_SWEEP_STEP_MS;
			}
			if(mp2733_charging_active(chg_stat))
			{
				return mp2733_enter_fg_state(MP2733_FG_CHARGING);
			}
			if(chg_stat == MP2733_DONE)
			{
				return mp2733_enter_fg_state(MP2733_FG_FULL);
			}
			if(fg_policy.next_vin_min_mv <= MP2733_VIN_SWEEP_STOP_MV)
			{
				err = mp2733_enter_fg_state(MP2733_FG_SETTLING);
				if(!err)
				{
					fg_policy.deadline_ms = now + MP2733_VIN_SWEEP_RETRY_MS;
				}
				return err;
			}
			break;

		case MP2733_FG_CHARGING:
			if(chg_stat == MP2733_DONE)
			{
				return mp2733_enter_fg_state(MP2733_FG_FULL);
			}
			if(!mp2733_charging_active(chg_stat))
			{
				return mp2733_enter_fg_state(MP2733_FG_SETTLING);
			}
			break;

		case MP2733_FG_FULL:
			/* FULL is latched for the attachment session, including automatic top-up. */
			break;
	}

	return 0;
}

static int battery_poll_once(struct controller_battery_report *report)
{
	uint8_t raw[MP2733_STATUS_REGISTER_COUNT];
	uint8_t vin_min_reg;
	enum mp2733_chg_stat chg_stat;
	bool charge_active;
	uint16_t battery_mv;
	uint8_t calculated_level;
	uint8_t vin_stat;
	int err;

	if(!i2c_is_ready_dt(&mp2733))
	{
		return -ENODEV;
	}

	err = mp2733_sample_registers(raw);
	if(err)
	{
		return err;
	}

	vin_stat = (raw[0] >> MP2733_VIN_STAT_SHIFT) & MP2733_VIN_STAT_MASK;
	chg_stat = (raw[0] >> MP2733_CHG_STAT_SHIFT) & MP2733_CHG_STAT_MASK;
	charge_active = mp2733_charging_active(chg_stat);
	if(!fg_policy.charge_active_valid || charge_active != fg_policy.charge_active)
	{
		battery_meter_sample_count = 0;
		if((fg_policy.charge_active_valid && charge_active != fg_policy.charge_active) ||
		   charge_active)
		{
			k_msleep(BATTERY_CHARGE_STATE_SETTLE_MS);
		}
		fg_policy.charge_active = charge_active;
		fg_policy.charge_active_valid = true;
	}

	err = battery_meter_average_mv(&battery_mv);
	if(err)
	{
		return err;
	}
	err = mp2733_run_fg_policy(chg_stat, vin_stat, battery_mv);
	if(err)
	{
		return err;
	}
	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_INPUT_LIMIT_STATUS, &raw[8]);
	if(err)
	{
		return err;
	}
	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_INPUT_VOLTAGE_LIMIT, &vin_min_reg);
	if(err)
	{
		return err;
	}

	memset(report, 0, sizeof(*report));
	report->charge_state = mp2733_charge_state(chg_stat, vin_stat);
	report->battery_mv = battery_mv;
	report->charger_battery_mv = 20U * raw[2];
	report->system_mv = 20U * raw[3];
	report->input_mv = 60U * raw[5];
	report->current_ma = mp2733_current_ma(chg_stat, raw[6]);
	report->input_current_ma = ((uint32_t)13300U * raw[7]) / 1000U;
	report->temperature_estimate_mc = mp2733_ntc_temperature_mc(raw[4]);
	report->temperature_mc = CLAMP(report->temperature_estimate_mc, 0, UINT16_MAX);
	report->charger_type = vin_stat;
	report->status_flags = raw[0];
	report->ntc_raw = raw[4];
	report->current_raw = raw[6];
	report->fuel_gauge_state = fg_policy.state;
	report->source_rating_ma = mp2733_source_rating_ma(vin_stat);
	report->requested_input_limit_ma = vin_stat ? 500U : 100U;
	report->effective_input_limit_ma = 100U + 50U * (raw[8] & 0x3fU);
	report->requested_vbat_target_mv = fg_policy.vbat_target_mv;
	report->requested_vin_min_mv = fg_policy.requested_vin_min_mv;
	report->observed_vin_min_mv =
	    MP2733_VIN_REG_OFFSET_MV + MP2733_VIN_REG_STEP_MV * (vin_min_reg & MP2733_VIN_REG_MASK);
	report->fault_status = raw[1];
	report->power_status = raw[8];
	report->reg17_status = raw[11];
	report->charge_complete = fg_policy.state == MP2733_FG_FULL;
	calculated_level =
	    mp2733_level_from_voltage(report->battery_mv, chg_stat, report->charge_complete);
	if(!fg_policy.level_valid ||
	   report->charge_complete ||
	   (charge_active && calculated_level > fg_policy.level_percent) ||
	   (!charge_active && calculated_level < fg_policy.level_percent))
	{
		fg_policy.level_percent = calculated_level;
		fg_policy.level_valid = true;
	}
	report->level_percent = fg_policy.level_percent;
	report->valid = true;
	return 0;
}

static int battery_read_fresh_status(struct controller_battery_report *report)
{
	int err;

	if(report == NULL)
	{
		return -EINVAL;
	}

	k_mutex_lock(&battery_poll_lock, K_FOREVER);
	if(battery_powering_off)
	{
		k_mutex_unlock(&battery_poll_lock);
		return -ESHUTDOWN;
	}
	err = battery_poll_once(report);
	if(!err)
	{
		k_mutex_lock(&battery_cache_lock, K_FOREVER);
		cached_report = *report;
		cached_report_at_ms = k_uptime_get();
		k_mutex_unlock(&battery_cache_lock);
	}
	k_mutex_unlock(&battery_poll_lock);
	if(err)
	{
		return err;
	}

	return 0;
}

static void battery_thread_entry(void *p1, void *p2, void *p3)
{
	struct controller_battery_report last_logged = { 0 };
	int64_t last_log_ms = 0;
	bool have_last_log = false;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_msleep(BATTERY_INITIAL_POLL_DELAY_MS);
	for(;;)
	{
		struct controller_battery_report report;
		int err;

		err = battery_read_fresh_status(&report);

		if(err)
		{
			LOG_WRN_RATELIMIT_RATE(BATTERY_LOG_PERIOD_MS, "MP2733 battery poll failed: %d", err);
		}
		else
		{
			int64_t now = k_uptime_get();
			bool status_changed = !have_last_log ||
			                      report.charge_state != last_logged.charge_state ||
			                      report.charger_type != last_logged.charger_type ||
			                      report.fault_status != last_logged.fault_status ||
			                      report.power_status != last_logged.power_status ||
			                      report.reg17_status != last_logged.reg17_status;

			(void)transport_send_battery_status(&report);
			if(status_changed || now - last_log_ms >= BATTERY_LOG_PERIOD_MS)
			{
				LOG_INF("%u%% %umV (mp=%umV) state=%s input=%umV ichg=%umA iin=%umA "
				        "type=%u limit=%u/%umA temp=%dmc flags=%02x/%02x/%02x",
				        report.level_percent, report.battery_mv, report.charger_battery_mv,
				        battery_charge_state_name(report.charge_state), report.input_mv,
				        report.current_ma, report.input_current_ma, report.charger_type,
				        report.requested_input_limit_ma, report.effective_input_limit_ma,
				        report.temperature_estimate_mc, report.fault_status, report.power_status,
				        report.reg17_status);
				last_logged = report;
				last_log_ms = now;
				have_last_log = true;
			}
		}

		k_msleep(BATTERY_POLL_PERIOD_MS);
	}
}

int battery_init(void)
{
	int err;

	if(!i2c_is_ready_dt(&mp2733))
	{
		LOG_WRN("MP2733 I2C bus is not ready");
		return -ENODEV;
	}

	err = mp2733_configure_charger();
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

int battery_prepare_poweroff(void)
{
	uint8_t status;
	uint8_t thermal_config;
	uint8_t batfet_config;
	uint8_t writes[3][2];
	struct i2c_msg messages[3];
	int err;

	if(!i2c_is_ready_dt(&mp2733))
	{
		return -ENODEV;
	}

	k_mutex_lock(&battery_poll_lock, K_FOREVER);
	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_STATUS_BASE, &status);
	if(err)
	{
		goto out;
	}
	if(((status >> MP2733_VIN_STAT_SHIFT) & MP2733_VIN_STAT_MASK) != 0U)
	{
		err = -EAGAIN;
		goto out;
	}

	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_THERMAL_CONFIGURATION, &thermal_config);
	if(err)
	{
		goto out;
	}
	err = i2c_reg_read_byte_dt(&mp2733, MP2733_REG_BATFET_CONFIGURATION, &batfet_config);
	if(err)
	{
		goto out;
	}

	/*
	 * OFW clears tSM_DLY, asserts BATTFET_DIS, then restores REG02 in one
	 * chained transfer. The middle write disconnects the battery from VSYS;
	 * keeping the sequence atomic prevents an unrelated charger access from
	 * being interleaved while the system rail is disappearing.
	 */
	writes[0][0] = MP2733_REG_THERMAL_CONFIGURATION;
	writes[0][1] = thermal_config & ~MP2733_TSM_DELAY;
	writes[1][0] = MP2733_REG_BATFET_CONFIGURATION;
	writes[1][1] = batfet_config | MP2733_BATFET_DISABLE;
	writes[2][0] = MP2733_REG_THERMAL_CONFIGURATION;
	writes[2][1] = thermal_config;

	messages[0] = (struct i2c_msg){
		.buf = writes[0],
		.len = sizeof(writes[0]),
		.flags = I2C_MSG_WRITE,
	};
	messages[1] = (struct i2c_msg){
		.buf = writes[1],
		.len = sizeof(writes[1]),
		.flags = I2C_MSG_WRITE | I2C_MSG_RESTART,
	};
	messages[2] = (struct i2c_msg){
		.buf = writes[2],
		.len = sizeof(writes[2]),
		.flags = I2C_MSG_WRITE | I2C_MSG_RESTART | I2C_MSG_STOP,
	};

	/* No charger worker may touch the PMIC after the shipping transaction. */
	battery_powering_off = true;
	err = i2c_transfer_dt(&mp2733, messages, ARRAY_SIZE(messages));
	if(err)
	{
		battery_powering_off = false;
	}

out:
	k_mutex_unlock(&battery_poll_lock);
	return err;
}

int battery_get_status(struct controller_battery_report *report)
{
	int64_t age_ms;
	int64_t sampled_at_ms;

	if(report == NULL)
	{
		return -EINVAL;
	}

	k_mutex_lock(&battery_cache_lock, K_FOREVER);
	*report = cached_report;
	sampled_at_ms = cached_report_at_ms;
	k_mutex_unlock(&battery_cache_lock);

	if(!report->valid)
	{
		return -EAGAIN;
	}

	age_ms = k_uptime_get() - sampled_at_ms;
	report->sample_age_ms = age_ms > UINT32_MAX ? UINT32_MAX : (uint32_t)age_ms;

	return age_ms > BATTERY_CACHE_MAX_AGE_MS ? -ESTALE : 0;
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

const char *battery_fuel_gauge_state_name(uint8_t state)
{
	switch(state)
	{
		case MP2733_FG_SETTLING:
			return "settling";
		case MP2733_FG_VBAT_SWEEP:
			return "vbat-sweep";
		case MP2733_FG_CHARGING:
			return "charging";
		case MP2733_FG_FULL:
			return "full-latched";
		default:
			return "unknown";
	}
}

#else

int battery_init(void)
{
	return -ENODEV;
}

int battery_prepare_poweroff(void)
{
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

const char *battery_fuel_gauge_state_name(uint8_t state)
{
	ARG_UNUSED(state);

	return "unavailable";
}

#endif
