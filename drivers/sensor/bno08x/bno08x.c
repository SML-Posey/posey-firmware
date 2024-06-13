#define DT_DRV_COMPAT ceva_bno08x

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>

#include "bno08x.h"
#include "posey-platform/platform/sensors/imu_reset.h"

// LOG_MODULE_REGISTER(BNO08x, LOG_LEVEL_DBG);
LOG_MODULE_REGISTER(BNO08x);

static int bno08x_attr_set(
    const struct device* dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value* val) {
    switch (chan) {
        default:
            LOG_WRN("attr_set() not supported on this channel.");
            return -ENOTSUP;
    }

    return 0;
}

static int bno08x_sample_fetch_accel(const struct device* dev) {
    struct bno08x_data* data = dev->data;

    data->ax = 11;
    data->ay = 12;
    data->az = 13;
    data->an++;

    return 0;
}

static int bno08x_sample_fetch_gyro(const struct device* dev) {
    struct bno08x_data* data = dev->data;

    data->gx = 21;
    data->gy = 22;
    data->gz = 23;

    return 0;
}

static int bno08x_sample_fetch_magn(const struct device* dev) {
    struct bno08x_data* data = dev->data;

    data->mx = 31;
    data->my = 32;
    data->mz = 33;

    return 0;
}

static int bno08x_sample_fetch(
    const struct device* dev, enum sensor_channel chan) {
    switch (chan) {
        case SENSOR_CHAN_ACCEL_XYZ:
            bno08x_sample_fetch_accel(dev);
            break;
        case SENSOR_CHAN_GYRO_XYZ:
            bno08x_sample_fetch_gyro(dev);
            break;
        case SENSOR_CHAN_MAGN_XYZ:
            bno08x_sample_fetch_magn(dev);
            break;
        case SENSOR_CHAN_ALL:
            bno08x_sample_fetch_accel(dev);
            bno08x_sample_fetch_gyro(dev);
            bno08x_sample_fetch_magn(dev);
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static inline void bno08x_accel_convert(struct sensor_value* val, double dval) {
    sensor_value_from_double(val, dval);
}

static inline int bno08x_accel_get_channel(
    enum sensor_channel chan,
    struct sensor_value* val,
    struct bno08x_data* data) {
    switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
            bno08x_accel_convert(val, data->ax);
            break;
        case SENSOR_CHAN_ACCEL_Y:
            bno08x_accel_convert(val, data->ay);
            break;
        case SENSOR_CHAN_ACCEL_Z:
            bno08x_accel_convert(val, data->az);
            break;
        case SENSOR_CHAN_ACCEL_XYZ:
            bno08x_accel_convert(val, data->ax);
            bno08x_accel_convert(val + 1, data->ay);
            bno08x_accel_convert(val + 2, data->az);
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static int bno08x_accel_channel_get(
    enum sensor_channel chan,
    struct sensor_value* val,
    struct bno08x_data* data) {
    return bno08x_accel_get_channel(chan, val, data);
}

static inline void bno08x_gyro_convert(struct sensor_value* val, double dval) {
    sensor_value_from_double(val, dval);
}

static inline int bno08x_gyro_get_channel(
    enum sensor_channel chan,
    struct sensor_value* val,
    struct bno08x_data* data) {
    switch (chan) {
        case SENSOR_CHAN_GYRO_X:
            bno08x_gyro_convert(val, data->gx);
            break;
        case SENSOR_CHAN_GYRO_Y:
            bno08x_gyro_convert(val, data->gy);
            break;
        case SENSOR_CHAN_GYRO_Z:
            bno08x_gyro_convert(val, data->gz);
            break;
        case SENSOR_CHAN_GYRO_XYZ:
            bno08x_gyro_convert(val, data->gx);
            bno08x_gyro_convert(val + 1, data->gy);
            bno08x_gyro_convert(val + 2, data->gz);
            break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

static int bno08x_gyro_channel_get(
    enum sensor_channel chan,
    struct sensor_value* val,
    struct bno08x_data* data) {
    return bno08x_gyro_get_channel(chan, val, data);
}

static inline void bno08x_magn_convert(struct sensor_value* val, double dval) {
    sensor_value_from_double(val, dval);
}

static inline int bno08x_magn_get_channel(
    enum sensor_channel chan,
    struct sensor_value* val,
    struct bno08x_data* data) {
    switch (chan) {
        case SENSOR_CHAN_MAGN_X:
            bno08x_magn_convert(val, data->mx);
            break;
        case SENSOR_CHAN_MAGN_Y:
            bno08x_magn_convert(val, data->my);
            break;
        case SENSOR_CHAN_MAGN_Z:
            bno08x_magn_convert(val, data->mz);
            break;
        case SENSOR_CHAN_MAGN_XYZ:
            bno08x_magn_convert(val, data->mx);
            bno08x_magn_convert(val + 1, data->my);
            bno08x_magn_convert(val + 2, data->mz);
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}

static int bno08x_magn_channel_get(
    enum sensor_channel chan,
    struct sensor_value* val,
    struct bno08x_data* data) {
    return bno08x_magn_get_channel(chan, val, data);
}

static int bno08x_channel_get(
    const struct device* dev,
    enum sensor_channel chan,
    struct sensor_value* val) {
    struct bno08x_data* data = dev->data;

    switch (chan) {
        case SENSOR_CHAN_ACCEL_X:
        case SENSOR_CHAN_ACCEL_Y:
        case SENSOR_CHAN_ACCEL_Z:
        case SENSOR_CHAN_ACCEL_XYZ:
            bno08x_accel_channel_get(chan, val, data);
            break;

        case SENSOR_CHAN_GYRO_X:
        case SENSOR_CHAN_GYRO_Y:
        case SENSOR_CHAN_GYRO_Z:
        case SENSOR_CHAN_GYRO_XYZ:
            bno08x_gyro_channel_get(chan, val, data);
            break;

        case SENSOR_CHAN_MAGN_X:
        case SENSOR_CHAN_MAGN_Y:
        case SENSOR_CHAN_MAGN_Z:
        case SENSOR_CHAN_MAGN_XYZ:
            bno08x_magn_channel_get(chan, val, data);
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api bno08x_driver_api = {
    .attr_set = bno08x_attr_set,
    .trigger_set = bno08x_trigger_set,
    .sample_fetch = bno08x_sample_fetch,
    .channel_get = bno08x_channel_get,
};

static void hal_hardwareReset(void) {
    LOG_DBG("BNO08x Hardware reset");
    imu_reset();
}

static uint32_t hal_getTimeUs(sh2_Hal_t* self) {
    uint32_t t = k_cyc_to_us_floor32(sys_clock_tick_get());
    return t;
}

static void hal_callback(void* cookie, sh2_AsyncEvent_t* pEvent) {
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        LOG_INF("IMU reset!");
        if (cookie != NULL) {
            struct bno08x_data* data = ((struct device*)cookie)->data;
            data->sh2_reset_occurred = 1;
        }
    }
}

static int hal_enable_report(sh2_SensorId_t sensorId, uint32_t interval_us) {
    static sh2_SensorConfig_t config;

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    config.reportInterval_us = interval_us;
    int status = sh2_setSensorConfig(sensorId, &config);

    return status;
}

// Handle sensor events.
static void hal_sensorHandler(void* cookie, sh2_SensorEvent_t* event) {
    LOG_DBG("Sensor event!");
    if (cookie == NULL)
        return;
    struct bno08x_data* data = ((struct device*)cookie)->data;
    if (data == NULL)
        return;
    if (data->sh2_sensor_value == NULL)
        return;

    int rc = sh2_decodeSensorEvent(data->sh2_sensor_value, event);
    if (rc != SH2_OK) {
        LOG_DBG("BNO08x - Error decoding sensor event; rc=%x", rc);
        data->sh2_sensor_value->timestamp = 0;
    }
}

static void bno08x_trigger_handler(
    const struct device* dev, const struct sensor_trigger* trig) {
    if (dev == NULL)
        return;
    struct bno08x_data* data = dev->data;
    if (data == NULL)
        return;

    data->irqn += 1;

    sh2_SensorValue_t value;
    data->sh2_sensor_value = &value;

    while (true) {
        value.timestamp = 0;
        sh2_service();
        if (value.timestamp == 0)
            return;

        if (value.sensorId == SH2_ROTATION_VECTOR) {
            data->qi = value.un.rotationVector.i;
            data->qj = value.un.rotationVector.j;
            data->qk = value.un.rotationVector.k;
            data->qr = value.un.rotationVector.real;
            data->qacc = value.un.rotationVector.accuracy;
            ++data->qn;
            LOG_DBG(
                "rot < %.2f , %.2f , %.2f, %.2f > acc: %.2f", data->qi,
                data->qj, data->qk, data->qr, data->qacc);
        } else if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
            data->qi = value.un.gameRotationVector.i;
            data->qj = value.un.gameRotationVector.j;
            data->qk = value.un.gameRotationVector.k;
            data->qr = value.un.gameRotationVector.real;
            ++data->qn;
            LOG_DBG(
                "rot < %.2f , %.2f , %.2f, %.2f > acc: %.2f", data->qi,
                data->qj, data->qk, data->qr, data->qacc);
        } else if (value.sensorId == SH2_ACCELEROMETER) {
            data->ax = value.un.accelerometer.x;
            data->ay = value.un.accelerometer.y;
            data->az = value.un.accelerometer.z;
            ++data->an;
            LOG_DBG(
                "accel < %.2f , %.2f , %.2f >", data->ax, data->ay, data->az);
        } else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            data->gx = value.un.gyroscope.x;
            data->gy = value.un.gyroscope.y;
            data->gz = value.un.gyroscope.z;
            ++data->gn;
            LOG_DBG(
                "gyro < %.2f , %.2f , %.2f >", data->gx, data->gy, data->gz);
        } else if (value.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
            data->mx = value.un.magneticField.x;
            data->my = value.un.magneticField.y;
            data->mz = value.un.magneticField.z;
            ++data->mn;
            LOG_DBG("mag < %.2f , %.2f , %.2f >", data->mx, data->my, data->mz);
        } else {
            LOG_WRN("Unhandled message: %x", value.sensorId);
        }
    }

    data->sh2_sensor_value = NULL;
}

static int bno08x_init_chip(const struct device* dev) {
    struct bno08x_data* data = dev->data;

    data->irqn = 0;
    data->an = 0;
    data->ax = 0;
    data->ay = 0;
    data->az = 0;

    data->gn = 0;
    data->gx = 0;
    data->gy = 0;
    data->gz = 0;

    data->mn = 0;
    data->mx = 0;
    data->my = 0;
    data->mz = 0;

    data->qn = 0;
    data->qi = 0;
    data->qj = 0;
    data->qk = 0;
    data->qr = 0;
    data->qacc = 0;

    data->sh2_sensor_value = NULL;
    data->hw_tf = NULL;
    data->dev = dev;
    data->gpio = NULL;

    return 0;
}

static int bno08x_init_sh2(const struct device* dev) {
    LOG_INF("Initializing BNO08x");
    struct bno08x_data* data = dev->data;

    data->sh2_hal.getTimeUs = hal_getTimeUs;
    hal_hardwareReset();

    // Wait for BNO08x to boot.
    LOG_INF("BNO08x wait delay...");
    k_sleep(K_MSEC(200));

    // Open SH2 interface (also registers non-sensor event handler.)
    int status = sh2_open(&data->sh2_hal, hal_callback, NULL);
    if (status != SH2_OK) {
        LOG_ERR("sh2_open failed: %d", status);
        return status;
    }

    // Check connection partially by getting the product id's
    sh2_ProductIds_t prodIds;
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK)
        return status;
    LOG_INF("Found %d products:", prodIds.numEntries);
    for (int n = 0; n < prodIds.numEntries; n++) {
        LOG_INF(
            "  [%d] Part: <%x> SW: %d.%d.%d", n + 1,
            prodIds.entry[n].swPartNumber, prodIds.entry[n].swVersionMajor,
            prodIds.entry[n].swVersionMinor, prodIds.entry[n].swVersionPatch);
    }

    // Register sensor listener
    sh2_setSensorCallback(hal_sensorHandler, (void*)dev);

    return 0;
}

static int bno08x_init(const struct device* dev) {
    int ret;
    const struct bno08x_config* const config = dev->config;

    ret = config->bus_init(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize sensor bus");
        return ret;
    }

    ret = bno08x_init_chip(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize chip");
        return ret;
    }

    // return 0; // TODO: ATW: DELETE.

    for (int i = 0; i < 30; ++i) {
        ret = bno08x_init_sh2(dev);
        if (ret < 0) {
            LOG_WRN("Failed to initialize SH2 HAL, attempt %d of 3", i + 1);
        } else
            break;
    }
    if (ret < 0) {
        LOG_ERR("Failed to initialize SH2 HAL");
        return ret;
    }

    ret = bno08x_init_interrupt(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize interrupt.");
        return ret;
    }

    // TODO: Something weird is going on with the reporting rates. They work
    // when used individually, but together they give weird results, sometimes
    // higher (usually accel) sometimes lower (rot).

    // TODO 2: This probably has to do with the internal pull-up resistance
    // being way too high (13k). It sorta works with the BNO085 I believe
    // because that board has 10k pull-ups, bringing the total resistance down
    // to ~5k, but that's still too high for even 100kHz operation. We might get
    // closer with the 100kHz I2C mode.

    static const uint32_t period_us = 20000;

    LOG_INF("Initializing accelerometer...");
    ret = hal_enable_report(SH2_ACCELEROMETER, period_us);
    if (ret != 0) {
        LOG_ERR("Failed to enable accelerometer report");
        return ret;
    }
    // ret = hal_enable_report(SH2_GYROSCOPE_CALIBRATED, period_us);
    // if (ret != 0)
    // {
    // 	LOG_ERR("Failed to enable gyroscope report");
    // 	return ret;
    // }
    // ret = hal_enable_report(SH2_MAGNETIC_FIELD_CALIBRATED, period_us);
    // if (ret != 0)
    // {
    // 	LOG_ERR("Failed to enable magnetic field report");
    // 	return ret;
    // }
    LOG_INF("Initializing 6DoF rotation vector...");
    ret = hal_enable_report(SH2_GAME_ROTATION_VECTOR, period_us);
    if (ret != 0) {
        LOG_ERR("Failed to enable rotation report");
        return ret;
    }

    // Set interrupt.
    struct sensor_trigger trig;
    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ALL;
    ret = bno08x_trigger_set(dev, &trig, bno08x_trigger_handler);
    if (ret < 0) {
        LOG_ERR("Failed to set trigger.");
        return ret;
    }

    return 0;
}

#define BNO08x_CFG_IRQ(inst)                             \
    .irq_dev_name = DT_INST_GPIO_LABEL(inst, irq_gpios), \
    .irq_pin = DT_INST_GPIO_PIN(inst, irq_gpios),        \
    .irq_flags = DT_INST_GPIO_FLAGS(inst, irq_gpios),

#define BNO08x_DEVICE_INIT(inst)                                             \
    DEVICE_DT_INST_DEFINE(                                                   \
        inst, bno08x_init, NULL, &bno08x_data_##inst, &bno08x_config_##inst, \
        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &bno08x_driver_api);

#define BNO08x_CONFIG_I2C(inst)                 \
    {.bus_init = bno08x_i2c_init,               \
     .bus_cfg.i2c = I2C_DT_SPEC_INST_GET(inst), \
     BNO08x_CFG_IRQ(inst)}

#define BNO08x_DEFINE_I2C(inst)                              \
    static struct bno08x_data bno08x_data_##inst;            \
    static const struct bno08x_config bno08x_config_##inst = \
        BNO08x_CONFIG_I2C(inst);                             \
    BNO08x_DEVICE_INIT(inst)

#define BNO08x_DEFINE(inst) BNO08x_DEFINE_I2C(inst)

DT_INST_FOREACH_STATUS_OKAY(BNO08x_DEFINE)
