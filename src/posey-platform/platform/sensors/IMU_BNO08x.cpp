#include "posey-platform/platform/sensors/IMU_BNO08x.hpp"
#include "platform.hpp"

#include "bno08x.h"

#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

LOG_MODULE_REGISTER(IMU_BNO08x);

static constexpr int AccelerometerReport = 0;
static constexpr int GyroscopeReport = 1;
static constexpr int MagnetometerReport = 2;
static constexpr int QuaternionReport = 3;

static volatile uint32_t report_count[4] = {0};

static volatile float acceleration[3] = {0.0f, 0.0f, 0.0f};
static volatile float gyroscope[3] = {0.0f, 0.0f, 0.0f};
static volatile float magnetometer[3] = {0.0f, 0.0f, 0.0f};
static volatile float quaternion[4] = {0.0f, 0.0f, 0.0f, 1.0f};

extern "C" {

constexpr bool using_accel = true;
constexpr bool using_gyro = false;
constexpr bool using_mag = false;
constexpr bool using_quat = true;

// constexpr uint32_t sample_period_us = 5000;  // 5ms -> 200 Hz
constexpr uint32_t sample_period_us = 10000;  // 10ms -> 100 Hz
// constexpr uint32_t sample_period_us = 100000;  // 100ms -> 10 Hz

static bool accel_enabled = false;
static bool gyro_enabled = false;
static bool mag_enabled = false;
static bool quat_enabled = false;

static int imu_enable_report(sh2_SensorId_t sensorId, uint32_t interval_us) {
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

static void imu_initialize_reports() {
    int ret = 0;

    if (using_accel && !accel_enabled) {
        LOG_INF("Initializing accelerometer...");
        k_msleep(50);
        ret = imu_enable_report(SH2_ACCELEROMETER, sample_period_us);
        if (ret != 0) {
            LOG_ERR("Failed to enable accelerometer report");
        }
    }

    if (using_gyro && !gyro_enabled) {
        LOG_INF("Initializing gyroscope...");
        k_msleep(50);
        ret = imu_enable_report(SH2_GYROSCOPE_CALIBRATED, sample_period_us);
        if (ret != 0) {
            LOG_ERR("Failed to enable gyroscope report");
        }
    }

    if (using_mag && !mag_enabled) {
        LOG_INF("Initializing magnetometer...");
        k_msleep(50);
        ret =
            imu_enable_report(SH2_MAGNETIC_FIELD_CALIBRATED, sample_period_us);
        if (ret != 0) {
            LOG_ERR("Failed to enable magnetic field report");
        }
    }

    if (using_quat && !quat_enabled) {
        LOG_INF("Initializing 9DoF rotation vector...");
        k_msleep(50);
        ret = imu_enable_report(SH2_ROTATION_VECTOR, sample_period_us);
        if (ret != 0) {
            LOG_ERR("Failed to enable rotation report");
        }
    }
}

static const char* sh2_SensorId_to_str(const sh2_SensorId_t id) {
    switch (id) {
        case SH2_RAW_ACCELEROMETER:
            return "SH2_RAW_ACCELEROMETER";
        case SH2_ACCELEROMETER:
            return "SH2_ACCELEROMETER";
        case SH2_LINEAR_ACCELERATION:
            return "SH2_LINEAR_ACCELERATION";
        case SH2_GRAVITY:
            return "SH2_GRAVITY";
        case SH2_RAW_GYROSCOPE:
            return "SH2_RAW_GYROSCOPE";
        case SH2_GYROSCOPE_CALIBRATED:
            return "SH2_GYROSCOPE_CALIBRATED";
        case SH2_GYROSCOPE_UNCALIBRATED:
            return "SH2_GYROSCOPE_UNCALIBRATED";
        case SH2_RAW_MAGNETOMETER:
            return "SH2_RAW_MAGNETOMETER";
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            return "SH2_MAGNETIC_FIELD_CALIBRATED";
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            return "SH2_MAGNETIC_FIELD_UNCALIBRATED";
        case SH2_ROTATION_VECTOR:
            return "SH2_ROTATION_VECTOR";
        case SH2_GAME_ROTATION_VECTOR:
            return "SH2_GAME_ROTATION_VECTOR";
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            return "SH2_GEOMAGNETIC_ROTATION_VECTOR";
        case SH2_PRESSURE:
            return "SH2_PRESSURE";
        case SH2_AMBIENT_LIGHT:
            return "SH2_AMBIENT_LIGHT";
        case SH2_HUMIDITY:
            return "SH2_HUMIDITY";
        case SH2_PROXIMITY:
            return "SH2_PROXIMITY";
        case SH2_TEMPERATURE:
            return "SH2_TEMPERATURE";
        case SH2_RESERVED:
            return "SH2_RESERVED";
        case SH2_TAP_DETECTOR:
            return "SH2_TAP_DETECTOR";
        case SH2_STEP_DETECTOR:
            return "SH2_STEP_DETECTOR";
        case SH2_STEP_COUNTER:
            return "SH2_STEP_COUNTER";
        case SH2_SIGNIFICANT_MOTION:
            return "SH2_SIGNIFICANT_MOTION";
        case SH2_STABILITY_CLASSIFIER:
            return "SH2_STABILITY_CLASSIFIER";
        case SH2_SHAKE_DETECTOR:
            return "SH2_SHAKE_DETECTOR";
        case SH2_FLIP_DETECTOR:
            return "SH2_FLIP_DETECTOR";
        case SH2_PICKUP_DETECTOR:
            return "SH2_PICKUP_DETECTOR";
        case SH2_STABILITY_DETECTOR:
            return "SH2_STABILITY_DETECTOR";
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
            return "SH2_PERSONAL_ACTIVITY_CLASSIFIER";
        case SH2_SLEEP_DETECTOR:
            return "SH2_SLEEP_DETECTOR";
        case SH2_TILT_DETECTOR:
            return "SH2_TILT_DETECTOR";
        case SH2_POCKET_DETECTOR:
            return "SH2_POCKET_DETECTOR";
        case SH2_CIRCLE_DETECTOR:
            return "SH2_CIRCLE_DETECTOR";
        case SH2_HEART_RATE_MONITOR:
            return "SH2_HEART_RATE_MONITOR";
        case SH2_ARVR_STABILIZED_RV:
            return "SH2_ARVR_STABILIZED_RV";
        case SH2_ARVR_STABILIZED_GRV:
            return "SH2_ARVR_STABILIZED_GRV";
        case SH2_GYRO_INTEGRATED_RV:
            return "SH2_GYRO_INTEGRATED_RV";
        case SH2_IZRO_MOTION_REQUEST:
            return "SH2_IZRO_MOTION_REQUEST";
        case SH2_RAW_OPTICAL_FLOW:
            return "SH2_RAW_OPTICAL_FLOW";
        case SH2_DEAD_RECKONING_POSE:
            return "SH2_DEAD_RECKONING_POSE";
        case SH2_WHEEL_ENCODER:
            return "SH2_WHEEL_ENCODER";
        default:
            return "UNKNOWN";
    }
}

static const char* sh2_ShtpEvent_to_str(const sh2_ShtpEvent_t event) {
    switch (event) {
        case SH2_SHTP_TX_DISCARD:
            return "SH2_SHTP_TX_DISCARD";
        case SH2_SHTP_SHORT_FRAGMENT:
            return "SH2_SHTP_SHORT_FRAGMENT";
        case SH2_SHTP_TOO_LARGE_PAYLOADS:
            return "SH2_SHTP_TOO_LARGE_PAYLOADS";
        case SH2_SHTP_BAD_RX_CHAN:
            return "SH2_SHTP_BAD_RX_CHAN";
        case SH2_SHTP_BAD_TX_CHAN:
            return "SH2_SHTP_BAD_TX_CHAN";
        case SH2_SHTP_BAD_FRAGMENT:
            return "SH2_SHTP_BAD_FRAGMENT";
        case SH2_SHTP_BAD_SN:
            return "SH2_SHTP_BAD_SN";
        case SH2_SHTP_INTERRUPTED_PAYLOAD:
            return "SH2_SHTP_INTERRUPTED_PAYLOAD";
        default:
            return "UNKNOWN";
    }
}

static void sh2_log_sensor_config(const sh2_SensorConfig_t config) {
    LOG_INF("  changeSensitivityEnabled: %d", config.changeSensitivityEnabled);
    LOG_INF(
        "  changeSensitivityRelative: %d", config.changeSensitivityRelative);
    LOG_INF("  wakeupEnabled: %d", config.wakeupEnabled);
    LOG_INF("  alwaysOnEnabled: %d", config.alwaysOnEnabled);
    LOG_INF("  sniffEnabled: %d", config.sniffEnabled);
    LOG_INF("  changeSensitivity: %d", config.changeSensitivity);
    LOG_INF("  reportInterval_us: %d", config.reportInterval_us);
    LOG_INF("  batchInterval_us: %d", config.batchInterval_us);
    LOG_INF("  sensorSpecific: %d", config.sensorSpecific);
}

void imu_event_handler(const struct device* dev, sh2_AsyncEvent_t* event) {
    // On a sensor reset, we need to reinitialize the reports.
    switch (event->eventId) {
        case SH2_RESET:
            LOG_INF("IMU reset!");
            imu_initialize_reports();
            break;

        case SH2_SHTP_EVENT:
            LOG_INF(
                "SHTP event: %d %s", event->shtpEvent,
                sh2_ShtpEvent_to_str(event->shtpEvent));
            break;

        case SH2_GET_FEATURE_RESP:
            LOG_INF(
                "GET_FEATURE_RESP: Sensor: %s",
                sh2_SensorId_to_str(event->sh2SensorConfigResp.sensorId));
            sh2_log_sensor_config(event->sh2SensorConfigResp.sensorConfig);
            break;

        default:
            LOG_WRN("Unknown sensor event ID: %d", event->eventId);
    }
}

void imu_sensor_handler(
    const struct device* dev, const sh2_SensorValue_t* sensor_value) {
    const sh2_SensorValue_t value = *sensor_value;
    if (value.sensorId == SH2_ROTATION_VECTOR) {
        quaternion[0] = value.un.rotationVector.i;
        quaternion[1] = value.un.rotationVector.j;
        quaternion[2] = value.un.rotationVector.k;
        quaternion[3] = value.un.rotationVector.real;
        report_count[QuaternionReport]++;
    } else if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
        quaternion[0] = value.un.gameRotationVector.i;
        quaternion[1] = value.un.gameRotationVector.j;
        quaternion[2] = value.un.gameRotationVector.k;
        quaternion[3] = value.un.gameRotationVector.real;
        report_count[QuaternionReport]++;
    } else if (value.sensorId == SH2_GYRO_INTEGRATED_RV) {
        quaternion[0] = value.un.gyroIntegratedRV.i;
        quaternion[1] = value.un.gyroIntegratedRV.j;
        quaternion[2] = value.un.gyroIntegratedRV.k;
        quaternion[3] = value.un.gyroIntegratedRV.real;
        report_count[QuaternionReport]++;
    } else if (value.sensorId == SH2_ACCELEROMETER) {
        acceleration[0] = value.un.accelerometer.x;
        acceleration[1] = value.un.accelerometer.y;
        acceleration[2] = value.un.accelerometer.z;
        report_count[AccelerometerReport]++;
    } else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        gyroscope[0] = value.un.gyroscope.x;
        gyroscope[1] = value.un.gyroscope.y;
        gyroscope[2] = value.un.gyroscope.z;
        report_count[GyroscopeReport]++;
    } else if (value.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
        magnetometer[0] = value.un.magneticField.x;
        magnetometer[1] = value.un.magneticField.y;
        magnetometer[2] = value.un.magneticField.z;
        report_count[MagnetometerReport]++;
    } else {
        LOG_WRN("Unhandled message: %x", value.sensorId);
    }
}
}

IMU_BNO08x::IMU_BNO08x() {}

bool IMU_BNO08x::setup() {
    _dev = DEVICE_DT_GET(DT_INST(0, ceva_bno08x));

    if (_dev == nullptr) {
        LOG_WRN("Could not get BNO08x device");
        return false;
    }

    struct ceva_bno08x_data* data = (struct ceva_bno08x_data*)_dev->data;
    data->user_event_handler = imu_event_handler;
    data->user_sensor_handler = imu_sensor_handler;

    k_msleep(50);
    sh2_devReset();
    k_msleep(50);

    return true;
}

static inline float Hz(const uint32_t n, const float dt) {
    return n / dt;
}

bool IMU_BNO08x::collect() {
    data.time_ms = Clock::get_msec<uint32_t>();

    data.Ax = acceleration[0];
    data.Ay = acceleration[1];
    data.Az = acceleration[2];

    data.Qi = quaternion[0];
    data.Qj = quaternion[1];
    data.Qk = quaternion[2];
    data.Qr = quaternion[3];

    static uint32_t t0 = data.time_ms;
    uint32_t t1 = data.time_ms;
    float dt = (t1 - t0) * 1.0e-3;
    if (dt >= 1.0) {
        // Determine data rates.
        float accel_rate = report_count[AccelerometerReport] / dt;
        float gyro_rate = report_count[GyroscopeReport] / dt;
        float mag_rate = report_count[MagnetometerReport] / dt;
        float quat_rate = report_count[QuaternionReport] / dt;

        if (using_accel) {
            LOG_INF(
                "Accel: %d samples in %.2f s (%.2f Hz)",
                report_count[AccelerometerReport], dt, accel_rate);
            LOG_INF(
                "     : X: %.2f Y: %.2f Z: %.2f", data.Ax, data.Ay, data.Az);
        }
        if (using_gyro) {
            LOG_INF(
                "Gyro: %d samples in %.2f s (%.2f Hz)",
                report_count[GyroscopeReport], dt, gyro_rate);
        }

        if (using_mag) {
            LOG_INF(
                "Mag: %d samples in %.2f s (%.2f Hz)",
                report_count[MagnetometerReport], dt, mag_rate);
        }

        if (using_quat) {
            LOG_INF(
                "Quat: %d samples in %.2f s (%.2f Hz)",
                report_count[QuaternionReport], dt, quat_rate);
            LOG_INF(
                "    : i: %.2f j: %.2f k: %.2f r: %.2f", data.Qi, data.Qj,
                data.Qk, data.Qr);
        }

        t0 = t1;
        for (int i = 0; i < 4; ++i)
            report_count[i] = 0;

        // Check for consecutive data misses.
        constexpr float LowRateThreshold = 0.95;
        bool low_accel_rate =
            using_accel && (accel_rate < (100 * LowRateThreshold));
        bool low_gyro_rate =
            using_gyro && (gyro_rate < (100 * LowRateThreshold));
        bool low_mag_rate = using_mag && (mag_rate < (100 * LowRateThreshold));
        bool low_quat_rate =
            using_quat && (quat_rate < (100 * LowRateThreshold));
        bool low_rates =
            low_accel_rate || low_gyro_rate || low_mag_rate || low_quat_rate;

        static uint32_t consecutive_low_rates = 0;
        if (low_rates) {
            consecutive_low_rates++;

            // If we miss ~60 seconds of data, reboot the system.
            if (consecutive_low_rates > 60) {
                LOG_ERR(
                    "Low IMU rates for too long (%d s), rebooting",
                    consecutive_low_rates);

                // Flush the log buffer before rebooting.
                if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
                    while (log_process())
                        ;
                Clock::delay_msec(1000);

                sys_reboot(SYS_REBOOT_COLD);
            }

            // If we miss 10 seconds of data, try to reset the device and
            // reinitialize the reports.
            else if (consecutive_low_rates % 10 == 0) {
                LOG_WRN(
                    "Low rates for too long (%d s), resetting IMU",
                    consecutive_low_rates);
                accel_enabled = false;
                gyro_enabled = false;
                mag_enabled = false;
                quat_enabled = false;
                sh2_devReset();
            }
        } else {
            consecutive_low_rates = 0;
        }
    }

    return true;
}
