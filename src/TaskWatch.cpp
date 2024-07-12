#include "platform.hpp"

#include "TaskWatch.hpp"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

#include "posey-platform/platform/io/NordicNUSDriver.h"
#include "posey-platform/platform/io/NordicSAADACDriver.h"

#define LOG_MODULE_NAME posey_task_watch
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static constexpr float freq = 200;
static constexpr uint32_t freqi = (uint32_t)freq;

bool TaskWatch::setup() {
    bool success = imu.setup();
    return success;
}

void TaskWatch::loop() {
    static uint32_t loop_time = 1.0e3 / freq;
    static uint32_t iter = 0;

    tm.message.t_start_ms = Clock::get_msec<uint32_t>();

    // Update sensor data.
    imu.collect();

    // Send sensor telemetry.
    imu.write_telemetry(writer);

    static float Vbatt = 0;
    static int lowbat = 0;

#if defined(CONFIG_LOG)
    Vbatt += read_Vbatt(false);
    if (iter % freqi == 0) {
        Vbatt /= freq * 1.0;

        read_Vbatt(true);

        if (Vbatt < 3.3) {
            LOG_WRN("LOW Average Vbat: %.2f V", Vbatt);
            if (lowbat++ >= (int)(freq * 60 * 1)) {
                LOG_ERR(
                    "Low battery: %.2f V! Sleeping for 5 minutes...", Vbatt);

                // Flush the log buffer before rebooting.
                if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
                    while (log_process())
                        ;

                // Disable scanning and close connections.
                disable_scanning();
                close_connections();

                deep_sleep();

                // Reboot to start fresh scanning.
                LOG_INF("Rebooting...");
                sys_reboot(SYS_REBOOT_COLD);
                lowbat = 0;
            }
        } else {
            // With normal voltages only log every 10s so it's not annoying.
            if (iter % (freqi * 10) == 0)
                LOG_INF("NORMAL Average Vbat: %.2f V", Vbatt);
            lowbat = 0;
        }

        if (Vbatt < 3.2)
            Vbatt = 3.2;
        if (Vbatt > 4.2)
            Vbatt = 4.2;
        tm.message.Vbatt = (read_Vbatt(false) - 3.2) / 4.2 * 255;
        tm.serialize();
        writer.write(tm.buffer, writer.SendNow);
    }
#endif
    ++iter;

    tm.message.t_end_ms = Clock::get_msec<uint32_t>();
    if (tm.message.t_end_ms - tm.message.t_start_ms > loop_time) {
        tm.message.missed_deadline++;
    }
}
