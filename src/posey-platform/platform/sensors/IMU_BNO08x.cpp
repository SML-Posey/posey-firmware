#include "platform.hpp"
#include "posey-platform/platform/sensors/IMU_BNO08x.hpp"

#include "bno08x.h"

LOG_MODULE_REGISTER(IMU_BNO08x);

IMU_BNO08x::IMU_BNO08x()
{
}

bool IMU_BNO08x::setup()
{
    _dev = device_get_binding(DT_LABEL(DT_INST(0, ceva_bno08x)));

	if (_dev == nullptr) {
		LOG_WRN("Could not get BNO08x device\n");
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
    data.time = Clock::get_usec<uint32_t>();

    data.An = _data->an;
    // data.Gn = _data->gn;
    // data.Mn = _data->mn;
    data.Qn = _data->qn;

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
    data.Qacc = _data->qacc;

    #ifdef CONFIG_LOG
	static uint32_t t0 = k_cyc_to_ms_floor32(sys_clock_tick_get());
	static uint32_t irqn0 = 0, an0 = 0, gn0 = 0, mn0 = 0, qn0 = 0;
    static uint32_t iter = 0;

    if (iter % 50 == 0)
    {
        uint32_t t1 = k_cyc_to_ms_floor32(sys_clock_tick_get());
        uint32_t
            irqn1 = _data->irqn,
            an1 = _data->an,
            gn1 = _data->gn,
            mn1 = _data->mn,
            qn1 = _data->qn;
        float dt = (t1 - t0)*1.0e-3;
        uint32_t dirqn = irqn1 - irqn0, dan = an1 - an0;
        // uint32_t dgn = gn1 - gn0, dmn = mn1 - mn0;
        uint32_t dqn = qn1 - qn0;
        t0 = t1;
        irqn0 = irqn1;
        an0 = an1;
        gn0 = gn1;
        mn0 = mn1;
        qn0 = qn1;

        LOG_INF("irq: %3d (%.1fHz)", _data->irqn, Hz(dirqn, dt));
        LOG_INF("accel: %3d (%.1fHz) < %.2f, %.2f, %.2f >",
            _data->an, Hz(dan, dt),
            _data->ax, _data->ay, _data->az);
        // LOG_INF("gyro : %3d (%.1fHz) < %.2f, %.2f, %.2f >",
        //     _data->gn, Hz(dgn, dt),
        //     _data->gx, _data->gy, _data->gz);
        // LOG_INF("mag  : %3d (%.1fHz) < %.2f, %.2f, %.2f >",
        //     _data->mn, Hz(dmn, dt),
        //     _data->mx, _data->my, _data->mz);
        LOG_INF("rot  : %3d (%.1fHz) < %.2fi, %.2fj, %.2fk, %.2f > acc: %.2f",
            _data->qn, Hz(dqn, dt),
            _data->qi, _data->qj, _data->qk, _data->qr, _data->qacc);
    }
    ++iter;
    #endif

    return true;
}
