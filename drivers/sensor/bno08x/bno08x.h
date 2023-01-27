#ifndef ZEPHYR_DRIVERS_SENSOR_BNO08x_BNO08x_H__
#define ZEPHYR_DRIVERS_SENSOR_BNO08x_BNO08x_H__

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <stdbool.h>
#include <zephyr/drivers/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

union bno08x_bus_cfg
{
	struct i2c_dt_spec i2c;
};

struct bno08x_config
{
	int (*bus_init)(const struct device *dev);
	const union bno08x_bus_cfg bus_cfg;
    char * irq_dev_name;
    uint32_t irq_pin;
    int irq_flags;
};

struct bno08x_transfer_function
{
	int (*read_data)(
        const struct device *dev,
        uint8_t reg_addr,
		uint8_t *value,
        uint8_t len);
	int (*write_data)(
        const struct device *dev,
        uint8_t reg_addr,
		uint8_t *value,
        uint8_t len);
	int (*read_reg)(
        const struct device *dev,
        uint8_t reg_addr,
        uint8_t *value);
	int (*update_reg)(
        const struct device *dev,
        uint8_t reg_addr,
		uint8_t mask,
        uint8_t value);
};

struct bno08x_data
{
    uint32_t irqn, an, gn, mn, qn;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float qi, qj, qk, qr, qacc;

    sh2_Hal_t sh2_hal;
    int sh2_reset_occurred;
    sh2_SensorValue_t *sh2_sensor_value;
	const struct bno08x_transfer_function *hw_tf;

    const struct device * dev;
    const struct device * gpio;
    struct gpio_callback gpio_cb;

    struct sensor_trigger data_ready_trigger;
    sensor_trigger_handler_t data_ready_handler;

    struct k_work work;
};

int bno08x_i2c_init(const struct device *dev);

int bno08x_trigger_set(
    const struct device *dev,
    const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler);
int bno08x_init_interrupt(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif
