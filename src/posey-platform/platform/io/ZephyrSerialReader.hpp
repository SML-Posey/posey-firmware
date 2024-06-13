#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "platform/io/BaseMessageReader.hpp"

class ZephyrSerialReader : public BaseMessageReader {
    public:
        ZephyrSerialReader(const device* const uart) : uart(uart) {}

        uint16_t read_to(
            uint8_t* dst_buffer, const uint16_t max_size) override {
            uint16_t bi = 0;
            for (; bi < max_size; ++bi) {
                if (uart_poll_in(uart, &dst_buffer[bi]) != 0)
                    break;
            }
            return bi;
        }

    private:
        const device* uart = nullptr;
};
