#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "platform/sensors/BaseIMU.hpp"

class IMU_BNO08x : public BaseIMU {
    public:
        IMU_BNO08x();

        bool setup() override;
        bool collect() override;

    private:
        const device* _dev = nullptr;
};
