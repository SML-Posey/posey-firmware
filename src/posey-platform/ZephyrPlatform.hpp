#pragma once

#define PLATFORM_POSEY

#include <zephyr.h>
#include <logging/log.h>

#include "platform/BaseRateLimiter.hpp"
#include "platform/BaseRateTask.hpp"

#include "posey-platform/platform/ZephyrClock.hpp"
#include "posey-platform/platform/sensors/IMU_Stub.hpp"

using Clock = ZephyrClock;

typedef BaseRateLimiter<
    Clock,
    unsigned long,
    float> RateLimiter;

typedef BaseRateTask<RateLimiter> RateTask;

#if CONFIG_BT_NUS_ENABLED
#include "posey-platform/platform/io/NordicNUSReader.hpp"
#include "posey-platform/platform/io/NordicNUSWriter.hpp"

extern NordicNUSReader reader;
extern NordicNUSWriter writer;
#else
#include "posey-platform/platform/io/ZephyrSerialReader.hpp"
#include "posey-platform/platform/io/ZephyrSerialWriter.hpp"

extern ZephyrSerialReader reader;
extern ZephyrSerialWriter writer;
#endif

extern IMU_Stub imu;

bool init_platform();
