#pragma once

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include "platform/sensors/BaseIMU.hpp"

#include "bno08x.h"

class IMU_BNO08x : public BaseIMU
{
    public:
        IMU_BNO08x();

        bool setup() override;
        bool collect() override;

    private:
        const device * _dev = nullptr;
        bno08x_data * _data = nullptr;
};
