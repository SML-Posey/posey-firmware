#pragma once

#include "platform/sensors/BaseIMU.hpp"

class IMU_Stub : public BaseIMU
{
    public:
        IMU_Stub();

        bool setup() override;
        bool collect() override;
};
