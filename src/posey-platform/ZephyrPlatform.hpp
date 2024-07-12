#pragma once

#define PLATFORM_POSEY

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "platform/BaseRateLimiter.hpp"
#include "platform/BaseRateTask.hpp"

#include "posey-platform/platform/ZephyrClock.hpp"

using Clock = ZephyrClock;

typedef BaseRateLimiter<Clock, unsigned long, float> RateLimiter;

typedef BaseRateTask<RateLimiter> RateTask;

#include "posey-platform/platform/io/NordicNUSReader.hpp"
#include "posey-platform/platform/io/NordicNUSWriter.hpp"

extern NordicNUSReader reader;
extern NordicNUSWriter writer;

#ifdef CONFIG_BNO08X
    #include "posey-platform/platform/sensors/IMU_BNO08x.hpp"
extern IMU_BNO08x imu;
#else
    #include "posey-platform/platform/sensors/IMU_Stub.hpp"
extern IMU_Stub imu;
#endif

#if defined(CONFIG_ROLE_HUB)
    #include "posey-platform/platform/sensors/BLE_Zephyr.hpp"
extern BLE_Zephyr ble;

    #include "TaskWaist.hpp"
#elif defined(CONFIG_ROLE_WATCH)
    #include "TaskWatch.hpp"
#elif defined(CONFIG_ROLE_RING)
    #include "TaskRing.hpp"
#else
    #error "No valid role defined!"
#endif

bool init_platform();
