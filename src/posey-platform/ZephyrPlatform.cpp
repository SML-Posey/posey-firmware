#include "platform.hpp"

#include <settings/settings.h>

#include "platform/io/NordicNUSDriver.h"

NordicNUSWriter writer;
NordicNUSReader reader;

IMU_BNO08x imu;
BLE_Zephyr ble;

bool init_platform()
{
    return init_nus() == 0;
}
