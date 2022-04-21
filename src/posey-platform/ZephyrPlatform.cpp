#include "platform.hpp"

#include <settings/settings.h>

#if CONFIG_BT_NUS_ENABLED
#include "platform/io/NordicNUSDriver.h"

NordicNUSWriter writer;
NordicNUSReader reader;
#else
#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>

const device * uart = device_get_binding("UART_0");

ZephyrSerialWriter writer(uart);
ZephyrSerialReader reader(uart);
#endif

IMU_BNO08x imu;
BLE_Zephyr ble;

bool init_platform()
{
    #if CONFIG_BT_NUS_ENABLED
    return init_nus() == 0;
    #else
    return true;
    #endif
}
