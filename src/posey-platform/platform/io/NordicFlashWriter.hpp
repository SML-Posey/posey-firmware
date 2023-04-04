#pragma once

#include <zephyr/kernel.h>

#include "platform/io/BaseMessageWriter.hpp"
#include "platform/sensors/BaseFlashBlock.hpp"

class NordicFlashWriter : public BaseMessageWriter
{
    public:
        NordicFlashWriter(const uint8_t slot)
        {
            block.data.slot = slot;
        }

        uint16_t write(
            const uint8_t * src_buffer,
            const uint16_t src_size,
            const bool immediate = false) override
        {
            LOG_DBG("Write %d bytes to flash from slot %d (RSSI: %d)",
                src_size, block.data.slot, block.data.rssi);
            return src_size;
        }

        uint16_t write(
            const uint8_t * src_buffer,
            const uint16_t src_size,
            const uint8_t rssi)
        {
            uint16_t written = 0;

            // Write header.
            block.data.time_ms = Clock::get_msec<uint32_t>();
            block.data.rssi = rssi;
            block.data.block_bytes = src_size;
            written += block.write_telemetry(*this);

            // Write data.
            written += write(src_buffer, src_size);

            return written;
        }

    private:
        BaseFlashBlock block;
};
