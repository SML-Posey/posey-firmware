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
    data.time = Clock::get_usec<uint32_t>();

    data.Ax = 1;
    data.Ay = 2;
    data.Az = 3;
    data.Gx = 11;
    data.Gy = 12;
    data.Gz = 13;
    data.Mx = 111;
    data.My = 112;
    data.Mz = 113;

    return true;
}
