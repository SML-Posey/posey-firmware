#include "platform.hpp"
#include "posey-platform/platform/sensors/IMU_BNO08x.hpp"

#include "bno08x.h"
#include "imu_reset.h"

#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(IMU_BNO08x);

IMU_BNO08x::IMU_BNO08x()
{
}

bool IMU_BNO08x::setup()
{
    _dev = device_get_binding(DT_LABEL(DT_INST(0, ceva_bno08x)));

	if (_dev == nullptr) {
		LOG_WRN("Could not get BNO08x device");
        imu_reset();
        return false;
	}

    _data = reinterpret_cast<bno08x_data *>(_dev->data);

    return true;
}

static inline float Hz(const uint32_t n, const float dt)
{
	return n/dt;
}

bool IMU_BNO08x::collect()
{
    data.time_ms = Clock::get_msec<uint32_t>();

    // data.An = _data->an;
    // data.Gn = _data->gn;
    // data.Mn = _data->mn;
    // data.Qn = _data->qn;

    data.Ax = _data->ax;
    data.Ay = _data->ay;
    data.Az = _data->az;

    // data.Gx = _data->gx;
    // data.Gy = _data->gy;
    // data.Gz = _data->gz;

    // data.Mx = _data->mx;
    // data.My = _data->my;
    // data.Mz = _data->mz;

    data.Qi = _data->qi;
    data.Qj = _data->qj;
    data.Qk = _data->qk;
    data.Qr = _data->qr;
    // data.Qacc = _data->qacc;

    static int iter = 0;
    if (iter++ % 300 == 0) {
        LOG_INF("A: [%.2f %.2f %.2f] Q: [%.2f %.2f %.2f %.2f]",
            data.Ax, data.Ay, data.Az,
            data.Qi, data.Qj, data.Qk, data.Qr);
    }

    // Check for missed data and reboot the system if we miss too much.
    static uint32_t consecutive_misses = 0;
    static uint32_t last_An = 0;
    static uint32_t last_Qn = 0;
    if ((last_An == _data->an) || (last_Qn == _data->qn)) {
        consecutive_misses++;
        // If we miss 10 seconds of data, reboot the system.
        if (consecutive_misses > 500) {
            LOG_ERR("Too many consecutive misses (%d), rebooting", consecutive_misses);

            // Flush the log buffer before rebooting.
            if (IS_ENABLED(CONFIG_LOG_MODE_DEFERRED))
                while (log_process());
            Clock::delay_msec(1000);
            
            sys_reboot(SYS_REBOOT_COLD);
        }
    } else {
        consecutive_misses = 0;
    }
    last_An = _data->an;
    last_Qn = _data->qn;

    return true;
}
