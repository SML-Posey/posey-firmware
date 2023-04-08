#include "platform.hpp"
#include "posey-platform/platform/sensors/IMU_Stub.hpp"

IMU_Stub::IMU_Stub()
{
}

bool IMU_Stub::setup()
{
    return true;
}

bool IMU_Stub::collect()
{
    data.time_ms = Clock::get_msec<uint32_t>();

    static float Ax = 0;
    Ax += 1.0;
    data.Ax = Ax;
    data.Ay = 2;
    data.Az = 3;
    // data.Gx = 11;
    // data.Gy = 12;
    // data.Gz = 13;
    // data.Mx = 111;
    // data.My = 112;
    // data.Mz = 113;
    data.Qr = 1111;
    data.Qi = 1112;
    data.Qj = 1113;
    data.Qk = 1114;
    // data.Qacc = 1115;

    return true;
}
