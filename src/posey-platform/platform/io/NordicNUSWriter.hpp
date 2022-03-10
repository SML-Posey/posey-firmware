#pragma once

#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/services/nus.h>

#include "platform/io/BaseMessageWriter.hpp"

class NordicNUSWriter : public BaseMessageWriter
{
    public:
        NordicNUSWriter(bt_conn * const ble = nullptr) : ble(ble) {}

        uint16_t write(
            const uint8_t * buffer,
            const uint16_t size) override
        {
            bt_nus_send(ble, buffer, size);
            return size;
        }

    private:
        bt_conn * ble;
};
