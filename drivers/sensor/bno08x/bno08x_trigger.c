#define DT_DRV_COMPAT ceva_bno08x

#include <device.h>
#include <drivers/i2c.h>
#include <sys/__assert.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "bno08x.h"

LOG_MODULE_DECLARE(BNO08x, CONFIG_SENSOR_LOG_LEVEL);

static inline void setup_irq(
    struct bno08x_data *drv_data,
	uint32_t irq_pin,
    bool enable)
{
	unsigned int flags = true
		? GPIO_INT_EDGE_TO_ACTIVE
		: GPIO_INT_DISABLE;

	gpio_pin_interrupt_configure(drv_data->gpio, irq_pin, flags);
}

static inline void handle_irq(
    struct bno08x_data *drv_data,
    uint32_t irq_pin)
{
	// setup_irq(drv_data, irq_pin, false);
	k_work_submit(&drv_data->work);
}

int bno08x_trigger_set(
    const struct device *dev,
    const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
	const struct bno08x_config *config = dev->config;
	struct bno08x_data *drv_data = dev->data;

	__ASSERT_NO_MSG(trig->type == SENSOR_TRIG_DATA_READY);

	/* If irq_gpio is not configured in DT just return error */
	if (!drv_data->gpio) {
		LOG_ERR("triggers not supported");
		return -ENOTSUP;
	}

	setup_irq(drv_data, config->irq_pin, false);

	drv_data->data_ready_handler = handler;
	if (handler == NULL) {
		return 0;
	}

	drv_data->data_ready_trigger = *trig;

	setup_irq(drv_data, config->irq_pin, true);
	if (gpio_pin_get(drv_data->gpio, config->irq_pin) > 0)
    {
		handle_irq(drv_data, config->irq_pin);
	}

	return 0;
}

static void bno08x_gpio_callback(
    const struct device *dev,
    struct gpio_callback *cb,
    uint32_t pins)
{
	struct bno08x_data *drv_data =
		CONTAINER_OF(cb, struct bno08x_data, gpio_cb);
	const struct bno08x_config *config = drv_data->dev->config;

	handle_irq(drv_data, config->irq_pin);
}

static void bno08x_thread_cb(const struct device *dev)
{
	const struct bno08x_config *config = dev->config;
	struct bno08x_data *drv_data = dev->data;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(
            dev, &drv_data->data_ready_trigger);
	}

	// setup_irq(drv_data, config->irq_pin, true);
}

static void bno08x_work_cb(struct k_work *work)
{
	struct bno08x_data *drv_data =
		CONTAINER_OF(work, struct bno08x_data, work);

	bno08x_thread_cb(drv_data->dev);
}

int bno08x_init_interrupt(const struct device *dev)
{
	const struct bno08x_config *config = dev->config;
	struct bno08x_data *drv_data = dev->data;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(config->irq_dev_name);
	if (drv_data->gpio == NULL)
    {
		LOG_INF("Cannot get pointer for irq_dev_name");
		return -EIO;
	}

	gpio_pin_configure(
        drv_data->gpio, config->irq_pin, GPIO_INPUT | config->irq_flags);

	gpio_init_callback(
        &drv_data->gpio_cb, bno08x_gpio_callback, BIT(config->irq_pin));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0)
    {
		LOG_ERR("Could not set gpio callback.");
		return -EIO;
	}

	drv_data->dev = dev;
	drv_data->work.handler = bno08x_work_cb;

	setup_irq(drv_data, config->irq_pin, true);

	return 0;
}
