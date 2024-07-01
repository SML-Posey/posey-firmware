#ifndef ZEPHYR_DRIVERS_SENSOR_CEVA_BNO08X_H_
#define ZEPHYR_DRIVERS_SENSOR_CEVA_BNO08X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_hal.h"

struct ceva_bno08x_bus_api {
        int (*read)(const void* context, uint8_t* buffer, uint16_t length);
        int (*write)(const void* context, uint8_t* buffer, uint16_t length);
        int (*init)(const void* context);
        void (*release)(const void* context);
};

struct ceva_bno08x_bus {
        const void* context;
        const struct ceva_bno08x_bus_api* api;
};

struct ceva_bno08x_sh2_hal {
        struct sh2_Hal_s sh2_hal;
        const struct device* dev;
};

struct ceva_bno08x_config {
        const struct ceva_bno08x_bus* bus;
        const struct ceva_bno08x_sh2_hal* hal;
        const struct gpio_dt_spec reset_gpio;
        const struct gpio_dt_spec ps0_gpio;
        const struct gpio_dt_spec ps1_gpio;
        const struct gpio_dt_spec int_gpio;

        void (*int_gpio_callback)(
            const struct device* dev, struct gpio_callback* cb, uint32_t pins);
};

struct ceva_bno08x_data {
        struct k_mutex lock;

        sh2_SensorValue_t* sh2_sensor_value;

        struct gpio_callback gpio_callback;
        struct k_work callback_work;
        const struct device* dev;

        void (*user_event_handler)(
            const struct device* dev, sh2_AsyncEvent_t* event);
        void (*user_sensor_handler)(
            const struct device* dev, const sh2_SensorValue_t* sensor_value);
};

void ceva_bno08x_hardware_reset(const struct device* dev);
void ceva_bno08x_work_handler(struct ceva_bno08x_data* data);
bool ceva_bno08x_int_active(const struct device* dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_CEVA_BNO08X_H_ */
