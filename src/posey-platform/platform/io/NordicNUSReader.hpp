#pragma once

#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/services/nus.h>

#include "platform/io/BaseMessageReader.hpp"

class NordicNUSReader : public BaseMessageReader
{
    public:
        NordicNUSReader(const bt_conn * const ble = nullptr) : ble(ble) {}

        uint16_t read_to(
            uint8_t * dst_buffer,
            const uint16_t max_size) override
        {
            // TODO: Stub for now.
            return 0;
        }

    private:
        const bt_conn * ble;
};
