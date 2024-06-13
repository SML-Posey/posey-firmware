#pragma once

#include <zephyr.h>
#include "platform/io/BaseMessageWriter.hpp"

class ZephyrSerialWriter : public BaseMessageWriter {
    public:
        ZephyrSerialWriter(const device* const uart) : uart(uart) {}

        uint16_t write(const uint8_t* buffer, const uint16_t size) override {
            for (uint16_t bi = 0; bi < size; ++bi)
                uart_poll_out(uart, buffer[bi]);
            return size;
        }

    private:
        const device* uart = nullptr;
};
