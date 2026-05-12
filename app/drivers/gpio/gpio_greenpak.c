/*
 * Input-only GPIO behavior for Ibex's SLG4L48185 GreenPAK.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#define DT_DRV_COMPAT valve_slg4l48185_gpio

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(gpio_greenpak, CONFIG_GPIO_LOG_LEVEL);

struct gpio_greenpak_config
{
	struct gpio_driver_config common;
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec interrupt;
	uint8_t input_selector;
	uint16_t poll_interval_ms;
};

struct gpio_greenpak_data
{
	struct gpio_driver_data common;
	const struct device *dev;
	struct k_mutex lock;
	struct k_work_delayable work;
	struct gpio_callback host_callback;
	sys_slist_t callbacks;
	gpio_port_value_t input_state;
	gpio_port_pins_t interrupt_enabled;
	gpio_port_pins_t pending;
	bool read_failed;
};

static int gpio_greenpak_read_inputs(const struct device *dev, gpio_port_value_t *value)
{
	const struct gpio_greenpak_config *config = dev->config;
	struct gpio_greenpak_data *data = dev->data;
	uint8_t raw[2];
	int err;

	err = i2c_write_read_dt(&config->i2c, &config->input_selector, sizeof(config->input_selector),
	                        raw, sizeof(raw));
	if(err)
	{
		if(!data->read_failed)
		{
			LOG_WRN("%s: input-state read failed: %d; treating all inputs as released", dev->name,
			        err);
			data->read_failed = true;
		}
		return err;
	}

	if(data->read_failed)
	{
		LOG_INF("%s: input-state reads resumed", dev->name);
		data->read_failed = false;
	}
	*value = sys_get_le16(raw) & config->common.port_pin_mask;
	return 0;
}

static void gpio_greenpak_work_handler(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);
	struct gpio_greenpak_data *data = CONTAINER_OF(delayable, struct gpio_greenpak_data, work);
	const struct gpio_greenpak_config *config = data->dev->config;
	gpio_port_value_t state;
	gpio_port_pins_t changed = 0;

	if(gpio_greenpak_read_inputs(data->dev, &state) == 0)
	{
		k_mutex_lock(&data->lock, K_FOREVER);
		changed = (state ^ data->input_state) & data->interrupt_enabled;
		data->input_state = state;
		data->pending |= changed;
		k_mutex_unlock(&data->lock);

		if(changed != 0U)
		{
			gpio_fire_callbacks(&data->callbacks, data->dev, changed);
		}
	}

	k_work_reschedule(&data->work, K_MSEC(config->poll_interval_ms));
}

static void gpio_greenpak_host_interrupt(const struct device *port, struct gpio_callback *callback,
                                         gpio_port_pins_t pins)
{
	struct gpio_greenpak_data *data =
	    CONTAINER_OF(callback, struct gpio_greenpak_data, host_callback);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);
	k_work_reschedule(&data->work, K_NO_WAIT);
}

static int gpio_greenpak_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct gpio_greenpak_config *config = dev->config;

	if((config->common.port_pin_mask & BIT(pin)) == 0U)
	{
		return -EINVAL;
	}

	if((flags & GPIO_INPUT) == 0U ||
	   (flags & GPIO_OUTPUT) != 0U ||
	   (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN | GPIO_SINGLE_ENDED)) != 0U)
	{
		return -ENOTSUP;
	}

	return 0;
}

static int gpio_greenpak_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct gpio_greenpak_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	*value = data->input_state;
	k_mutex_unlock(&data->lock);
	return 0;
}

static int gpio_greenpak_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
                                                 enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_greenpak_config *config = dev->config;
	struct gpio_greenpak_data *data = dev->data;

	if((config->common.port_pin_mask & BIT(pin)) == 0U)
	{
		return -EINVAL;
	}
	if(mode != GPIO_INT_MODE_DISABLED && (mode != GPIO_INT_MODE_EDGE || trig != GPIO_INT_TRIG_BOTH))
	{
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if(mode == GPIO_INT_MODE_DISABLED)
	{
		data->interrupt_enabled &= ~BIT(pin);
	}
	else
	{
		data->interrupt_enabled |= BIT(pin);
	}
	k_mutex_unlock(&data->lock);
	return 0;
}

static int gpio_greenpak_manage_callback(const struct device *dev, struct gpio_callback *callback,
                                         bool set)
{
	struct gpio_greenpak_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static uint32_t gpio_greenpak_get_pending_int(const struct device *dev)
{
	struct gpio_greenpak_data *data = dev->data;
	gpio_port_pins_t pending;

	k_mutex_lock(&data->lock, K_FOREVER);
	pending = data->pending;
	data->pending = 0;
	k_mutex_unlock(&data->lock);
	return pending;
}

static int gpio_greenpak_init(const struct device *dev)
{
	const struct gpio_greenpak_config *config = dev->config;
	struct gpio_greenpak_data *data = dev->data;
	int err;

	if(!i2c_is_ready_dt(&config->i2c))
	{
		return -ENODEV;
	}

	data->dev = dev;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->work, gpio_greenpak_work_handler);

	/* GreenPAK inputs are active-low so this is the safe unavailable state. */
	data->input_state = config->common.port_pin_mask;
	err = gpio_greenpak_read_inputs(dev, &data->input_state);
	if(err)
	{
		LOG_WRN("%s: continuing without initial input state", dev->name);
	}

	if(config->interrupt.port != NULL)
	{
		if(!gpio_is_ready_dt(&config->interrupt))
		{
			return -ENODEV;
		}
		err = gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT);
		if(err)
		{
			return err;
		}
		gpio_init_callback(&data->host_callback, gpio_greenpak_host_interrupt,
		                   BIT(config->interrupt.pin));
		err = gpio_add_callback(config->interrupt.port, &data->host_callback);
		if(err)
		{
			return err;
		}
		err = gpio_pin_interrupt_configure_dt(&config->interrupt, GPIO_INT_EDGE_BOTH);
		if(err)
		{
			return err;
		}
	}

	k_work_reschedule(&data->work, K_MSEC(config->poll_interval_ms));
	return 0;
}

static const struct gpio_driver_api gpio_greenpak_api = {
	.pin_configure = gpio_greenpak_pin_configure,
	.port_get_raw = gpio_greenpak_port_get_raw,
	.pin_interrupt_configure = gpio_greenpak_pin_interrupt_configure,
	.manage_callback = gpio_greenpak_manage_callback,
	.get_pending_int = gpio_greenpak_get_pending_int,
};

#define GPIO_GREENPAK_DEFINE(inst) \
	static const struct gpio_greenpak_config gpio_greenpak_config_##inst = { \
		.common = { \
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst), \
		}, \
		.i2c = I2C_DT_SPEC_INST_GET(inst), \
		.interrupt = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}), \
		.input_selector = DT_INST_PROP(inst, input_selector), \
		.poll_interval_ms = DT_INST_PROP(inst, poll_interval_ms), \
	}; \
	static struct gpio_greenpak_data gpio_greenpak_data_##inst; \
	DEVICE_DT_INST_DEFINE(inst, gpio_greenpak_init, NULL, \
			      &gpio_greenpak_data_##inst, &gpio_greenpak_config_##inst, \
			      POST_KERNEL, 80, &gpio_greenpak_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_GREENPAK_DEFINE)
