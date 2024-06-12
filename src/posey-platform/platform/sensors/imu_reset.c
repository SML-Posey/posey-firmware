#include "imu_reset.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu_reset);

// static const struct gpio_dt_spec gpio_imu_reset =
// DT_GPIO_LABEL(DT_ALIAS(gpioimureset0), gpios);
static const struct gpio_dt_spec gpio_imu_reset =
    GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), imureset_gpios);

void imu_reset() {
    LOG_WRN("Will try to reset the IMU");
    if (!device_is_ready(gpio_imu_reset.port))
        LOG_ERR("Could not get IMU nRESET GPIO device");

    gpio_pin_configure_dt(&gpio_imu_reset, GPIO_OUTPUT_INACTIVE);
    k_sleep(K_MSEC(10));
    gpio_pin_configure_dt(&gpio_imu_reset, GPIO_OUTPUT_ACTIVE);
    k_sleep(K_MSEC(10));
    gpio_pin_configure_dt(&gpio_imu_reset, GPIO_OUTPUT_INACTIVE);
    k_sleep(K_MSEC(10));
}