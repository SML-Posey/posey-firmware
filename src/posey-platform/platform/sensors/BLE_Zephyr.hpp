#pragma once

#include "platform/sensors/BaseBLE.hpp"

class BLE_Zephyr : public BaseBLE
{
    public:
        // This device needs to be a singleton.
        static BLE_Zephyr * reference;

    public:
        BLE_Zephyr();

        bool setup() override;
};
